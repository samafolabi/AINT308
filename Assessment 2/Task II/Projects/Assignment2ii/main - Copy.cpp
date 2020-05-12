/*
Phil Culverhouse Oct 2016 (c) Plymouth University
James Rogers Jan 2020     (c) Plymouth University

This demo code will compare the left and right images to produce a disparity map.

Before trying this code, you will need to calibrate servos so the cameras are parallel,
and calibrate the cameras using the stereo calibration program in the tools folder.

Use this code as a base for your assignment.

*/

#include <iostream>
#include <fstream>

#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/utility.hpp"

#include <stdio.h>

using namespace cv;
using namespace std;

int Distance=30;
int targetType=1;

int main(int argc, char** argv)
{

    string intrinsic_filename = "../../Data/intrinsics.xml";
    string extrinsic_filename = "../../Data/extrinsics.xml";

    int SADWindowSize=3;
    int numberOfDisparities=256;
    double scale = 1;

    //Check input variables
    if ( numberOfDisparities < 1 || numberOfDisparities % 16 != 0 ){
        printf("The max disparity must be a positive integer divisible by 16\n");
        return -1;
    }

    if (scale < 0){
        printf("The scale factor must be a positive floating-point number\n");
        return -1;
    }

    if (SADWindowSize < 1 || SADWindowSize % 2 != 1)
    {
        printf("The SADWindowSize must be a positive odd number\n");
        return -1;
    }

    // reading calibration data
    Rect roi1, roi2;
    Mat Q;
    Size img_size = {640,480};

    FileStorage fs(intrinsic_filename, FileStorage::READ);
    if(!fs.isOpened()){
        printf("Failed to open file %s\n", intrinsic_filename.c_str());
        return -1;
    }

    Mat M1, D1, M2, D2;
    fs["M1"] >> M1;
    fs["D1"] >> D1;
    fs["M2"] >> M2;
    fs["D2"] >> D2;

    M1 *= scale;
    M2 *= scale;

    fs.open(extrinsic_filename, FileStorage::READ);
    if(!fs.isOpened())
    {
        printf("Failed to open file %s\n", extrinsic_filename.c_str());
        return -1;
    }
    bool ran = false;
    Mat R, T, R1, P1, R2, P2;
    fs["R"] >> R;
    fs["T"] >> T;

    stereoRectify( M1, D1, M2, D2, img_size, R, T, R1, R2, P1, P2, Q, CALIB_ZERO_DISPARITY, -1, img_size, &roi1, &roi2 );

    Mat map11, map12, map21, map22;
    initUndistortRectifyMap(M1, D1, R1, P1, img_size, CV_16SC2, map11, map12);
    initUndistortRectifyMap(M2, D2, R2, P2, img_size, CV_16SC2, map21, map22);


    Mat Frame,Left,Right, disp, disp8;
    Ptr<StereoSGBM> sgbm = StereoSGBM::create(0,16,3);

    ofstream centerVals("center_vals.txt");
    int check = 0;

    while (1){

        //Load images from file
        String LeftPath ="../../Data/Task 2 Distance Targets/Target"+to_string(targetType)+"/left" +to_string(Distance)+"cm.jpg";
        String RightPath="../../Data/Task 2 Distance Targets/Target"+to_string(targetType)+"/right"+to_string(Distance)+"cm.jpg";

        Left =imread(LeftPath );
        Right=imread(RightPath);

        cout<<"Distance: "<<Distance<<"cm   \t Target: "<<targetType<<endl;

        //Distort image to correct for lens/positional distortion
        remap(Left, Left, map11, map12, INTER_LINEAR);
        remap(Right, Right, map21, map22, INTER_LINEAR);

        //Match left and right images to create disparity image
        numberOfDisparities = numberOfDisparities > 0 ? numberOfDisparities : ((img_size.width/8) + 15) & -16;
        int cn = Left.channels();
        int sgbmWinSize = SADWindowSize > 0 ? SADWindowSize : 3;

        sgbm->setBlockSize(sgbmWinSize);
        sgbm->setPreFilterCap(63);
        sgbm->setP1(8*cn*sgbmWinSize*sgbmWinSize);
        sgbm->setP2(32*cn*sgbmWinSize*sgbmWinSize);
        sgbm->setMinDisparity(0);
        sgbm->setNumDisparities(numberOfDisparities);
        sgbm->setUniquenessRatio(10);
        sgbm->setSpeckleWindowSize(100);
        sgbm->setSpeckleRange(32);
        sgbm->setDisp12MaxDiff(1);
        sgbm->setMode(StereoSGBM::MODE_SGBM);

        sgbm->compute(Left, Right, disp);

        Mat depth(img_size, CV_16S);
        //depth = (M1.at<double>(0,0) * 65)/disp;

        for (int i = 0; i < depth.rows; i++) {
            for (int j = 0; j < depth.cols; j++) {
                ushort val = disp.at<ushort>(i,j);
                val = val == 0 ? 1 : val;
                depth.at<ushort>(i,j) = (M1.at<double>(0,0)*65)/val;
            }
        }
        /*
        ushort maxD = 0;

        for (int i = 0; i < depth.rows; i++) {
            for (int j = 0; j < depth.cols; j++) {
                ushort val = disp.at<ushort>(i,j);
                if (val == 0) {
                    depth.at<ushort>(i,j) = 255;
                    continue;
                }
                ushort d = (M1.at<double>(0,0)*65)/val;
                maxD = d > maxD ? d : maxD;
                depth_1.at<ushort>(i,j) = d;
            }
        }

        for (int i = 0; i < depth.rows; i++) {
            for (int j = 0; j < depth.cols; j++) {
                ushort val = depth_1.at<ushort>(i,j);
                if (val == 255) {
                    continue;
                }
                depth.at<ushort>(i,j) = 255-((val*255)/maxD);
            }
        }

        */

        Mat depthNorm;
        //normalize(depth, depthNorm, 0, 1, cv::NORM_MINMAX);

        //Convert disparity map to an 8-bit greyscale image so it can be displayed
        disp.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));
        depth.convertTo(depthNorm, CV_8U);
        imshow("left", Left);
        imshow("right", Right);
        imshow("disparity", disp8);
        imshow("depth", depthNorm);
        if (check < 39) {
            centerVals << "Disparity at center: " << disp.at<ushort>(disp.rows/2, disp.cols/2) <<endl;
            targetType += Distance == 150 ? 1 : 0;
            Distance = Distance == 150 ? 30 : Distance + 10;
            if (check == 38) {
                centerVals.close();
            }
            check++;
        } else {

        //keyboard Controls
        int key=waitKey(10);

        switch(key){
            case 'w': Distance+=10; break;
            case 's': Distance-=10; break;
            case 'a': targetType++; break;
            case 'd': targetType--; break;
        }

        if(Distance>150){
            Distance=30;
        }else if(Distance<30){
            Distance=150;
        }

        if(targetType>3){
            targetType=1;
        }else if(targetType<1){
            targetType=3;
        }
        }

    }

    return 0;
}





