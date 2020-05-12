/*
Phil Culverhouse Oct 2016 (c) Plymouth University
James Rogers Jan 2020     (c) Plymouth University

This demo code will simulate a human attention system as discribed in Itti & Koch’s bottom-up Saccadic model.
The following feature maps are currently implemented:
    -DoG edge detection
    -Fovea (bias for central field targets)
    -Familiarity (bias for targets that havent been observed often)

You will need to add at least two more feature maps, as well as a distance estimation to the current target



Use this code as a base for your assignment.

*/

#define PI 3.14159265

#include <iostream>
#include <fstream>
#include <math.h>
#include <string>

#include <sys/types.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include "owl-pwm.h"
#include "owl-comms.h"
#include "owl-cv.h"

#include "opencv2/calib3d.hpp"

#define PX2DEG  0.0768
#define DEG2PWM 10.730
#define IPD 58.3

#define xOffset  -0.3815
#define yOffset -54.6682
#define xScale   26.4842
#define yScale   91.4178

using namespace std;
using namespace cv;

Mat DoGFilter(Mat src, int k, int g);
Mat StrongColour(Mat src);

//Default feature map weights
static int ColourWeight    = 60 ; //Saturation and Brightness
static int DoGHighWeight    = 60 ; //Groups of edges in a small area
static int DoGLowWeight     = 30 ; //DoG edge detection
static int FamiliarWeight = 5  ; //Familiarity of the target, how much has the owl focused on this before
static int foveaWeight    = 50 ; //Distance from fovea (center)
static int CannyLowThreshold = 200;
static int CannyHighThreshold = 300;

static int Sample = 3;
String ImagePath = "../../Data/Task 3 Salient Targets/Sample3.jpg";

int main(int argc, char *argv[])
{
    //==========================================Initialize Variables=================================
    Mat Left = imread(ImagePath);
    Mat LeftDisplay;
    Left.copyTo(LeftDisplay);
    Mat LeftGrey;
    cvtColor(Left, LeftGrey, COLOR_BGR2GRAY);
    Mat familiar(Left.size(),CV_8U,Scalar(255));
    Point Gaze(Left.size().width/2,Left.size().height/2);

    while (1){//Main processing loop

        // ======================================CALCULATE FEATURE MAPS ====================================
        //============================================DoG low bandpass Map==================================
        Mat DoGLow = DoGFilter(LeftGrey,3,51);
        Mat DoGLow8;
        normalize(DoGLow, DoGLow8, 0, 255, CV_MINMAX, CV_8U);
        imshow("DoG Low", DoGLow8);

        //=================================================Fovea Map========================================
        //Local Feature Map  - implements FOVEA as a bias to the saliency map to central targets, rather than peripheral targets
        Mat fovea(Left.size(),CV_8U,Scalar(0));
        circle(fovea, Gaze, 150, 255, -1);
        cv::blur(fovea, fovea, Size(301,301));
        fovea.convertTo(fovea, CV_32FC1);
        fovea*=foveaWeight;

        //=======================================Canny Edge Detection=======================================
        Mat edges;
        Canny(LeftGrey, edges, CannyLowThreshold, CannyHighThreshold);
        imshow("Canny", edges);

        //=========================================Strong Colour Map=========================================
        Mat colourMap = StrongColour(Left);
        imshow("Strong Colour Map", colourMap);

        //====================================Combine maps into saliency map================================
        //Convert 8-bit Mat to 32bit floating point
        DoGLow.convertTo(DoGLow, CV_32FC1);
        DoGLow*=DoGLowWeight;
        Mat familiarFloat;
        familiar.convertTo(familiarFloat, CV_32FC1);

        edges.convertTo(edges, CV_32FC1);
        edges*=DoGHighWeight;
        colourMap.convertTo(colourMap, CV_32FC1);
        colourMap *= ColourWeight;

        // Linear combination of feature maps to create a salience map
        Mat Salience=cv::Mat(Left.size(),CV_32FC1,0.0); // init map
        add(Salience,DoGLow,Salience);
        add(Salience,fovea,Salience);

        add(Salience,edges,Salience);
        add(Salience,colourMap,Salience);

        Salience=Salience.mul(familiarFloat);
        normalize(Salience, Salience, 0, 255, CV_MINMAX, CV_32FC1);

        //imshow("SalienceNew",Salience);
        //=====================================Find & Move to Most Salient Target=========================================
        double minVal; double maxVal; Point minLoc;
        minMaxLoc(Salience, &minVal, &maxVal, &minLoc, &Gaze);

        //Draw gaze path on screen
        static Point GazeOld=Gaze;
        line(LeftDisplay,Gaze,GazeOld,Scalar(0,255,255));
        circle(LeftDisplay,GazeOld,5,Scalar(0,255,255),-1);
        circle(LeftDisplay,Gaze,5,Scalar(0,0,255),-1);
        GazeOld=Gaze;

        // Update Familarity Map //
        // Familiar map to inhibit salient targets once observed (this is a global map)
        Mat familiarNew=familiar.clone();
        circle(familiarNew, Gaze, 60, 0, -1);
        cv::blur(familiarNew, familiarNew, Size(151,151)); //Blur used to save on processing
        normalize(familiarNew, familiarNew, 0, 255, CV_MINMAX, CV_8U);
        addWeighted(familiarNew, (static_cast<double>(FamiliarWeight)/100), familiar, (100-static_cast<double>(FamiliarWeight))/100, 0, familiar);
        imshow("Familiar",familiar);

        //=================================Convert Saliency into Heat Map=====================================
        //this is just for visuals
        Mat SalienceHSVnorm;
        Salience.convertTo(Salience, CV_8UC1);
        normalize(Salience, SalienceHSVnorm, 130, 255, CV_MINMAX, CV_8U);
        normalize(Salience, Salience, 0, 255, CV_MINMAX, CV_8U);
        Mat SalienceHSV;
        cvtColor(Left, SalienceHSV, COLOR_BGR2HSV);

        for(int y=0;y<Left.size().height;y++){
            for(int x=0; x<Left.size().width;x++){
                SalienceHSV.at<Vec3b>(y,x)=Vec3b(255-SalienceHSVnorm.at<uchar>(y,x),255,255);
            }
        }
        cvtColor(SalienceHSV, SalienceHSV, COLOR_HSV2BGR);


        //=======================================Update Global View===========================================
        imshow("LeftDisplay",LeftDisplay);
        imshow("SalienceHSV",SalienceHSV);

        //=========================================Control Window for feature weights =============================================
        //cout<<"Control Window"<<endl;
        namedWindow("Control", CV_WINDOW_AUTOSIZE);
        cvCreateTrackbar("LowFreq"  , "Control", &DoGLowWeight  , 100);
        cvCreateTrackbar("FamiliarW", "Control", &FamiliarWeight, 100);
        cvCreateTrackbar("foveaW"   , "Control", &foveaWeight   , 100);

        cvCreateTrackbar("CannyLowT", "Control", &CannyLowThreshold, 400);
        cvCreateTrackbar("CannyHighT", "Control", &CannyHighThreshold, 400);

        waitKey(10);
    }
}

// create DoG bandpass filter, with g being odd always and above 91 for low pass, and >9 for high pass
// k is normally 3 or 5
Mat DoGFilter(Mat src, int k, int g){
    Mat srcC;
    src.convertTo(srcC,CV_32FC1);
    Mat g1, g2;
    GaussianBlur(srcC, g1, Size(g,g), 0);
    GaussianBlur(srcC, g2, Size(g*k,g*k), 0);
    srcC = (g1 - g2)*2;
    return srcC;

}

Mat StrongColour(Mat src) {
    //Convert to an HSV image
    Mat src2;
    cvtColor(src, src2, CV_BGR2HSV);

    //Split HSV channels into Hue, Saturation, and Value images
    Mat hsv[3];
    split(src2,hsv);

    //Convert Saturation and Value images to fit bigger values
    Mat s,v;
    hsv[1].convertTo(s,CV_32FC1);
    hsv[2].convertTo(v,CV_32FC1);

    //Multiply Saturation and Value to get intensity
    Mat dst = s.mul(v);

    //Fits value into specified range
    normalize(dst, dst, 0, 255, CV_MINMAX, CV_8U);
    return dst;
}
















