/*
Phil Culverhouse Oct 2016 (c) Plymouth University
James Rogers Jan 2020     (c) Plymouth University

This demo code will move eye and neck servos with kepresses.
Use this code as a base for your assignment.

*/
#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <ctime>
#include <math.h>
#include <string>
#include <stdlib.h>
#include <sys/types.h>
#ifndef _WIN32
#include <unistd.h>
#endif
#include "owl-pwm.h"
#include "owl-comms.h"
#include "owl-cv.h"

using namespace std;
using namespace cv;

// set up comms
string RxPacket;
ostringstream CMDstream; // string packet
string CMD;
SOCKET u_sock;

// Send the current servo positions in code to the OWL
void sendCommand() {
	CMDstream.str("");
	CMDstream.clear();
	CMDstream << Rx << " " << Ry << " " << Lx << " " << Ly << " " << Neck;
	CMD = CMDstream.str();
	RxPacket= OwlSendPacket (u_sock, CMD.c_str());
}

int main(int argc, char *argv[])
{
    //Setup TCP coms
    string PiADDR = "10.0.0.10";
    int PORT=12345;
    u_sock = OwlCommsInit(PORT, PiADDR);

    //Set servo positions to their center-points
    Rx = RxC; Lx = LxC;
    Ry = RyC; Ly = LyC;
    Neck= NeckC;

    // move servos to centre of field
    sendCommand();

    Mat Frame, Left, Right;

    //Open video feed
    string source = "http://10.0.0.10:8080/stream/video.mjpeg";
    VideoCapture cap (source);
    if (!cap.isOpened())
    {
        cout  << "Could not open the input video: " << source << endl;
        return -1;
    }

    //main program loop
    while (1){
        if (!cap.read(Frame))
        {
            cout  << "Could not open the input video: " << source << endl;
            break;
        }

        //flip input image as it comes in reversed
        Mat FrameFlpd;
        flip(Frame,FrameFlpd,1);

        // Split into LEFT and RIGHT images from the stereo pair sent as one MJPEG iamge
        Left= FrameFlpd(Rect(0, 0, 640, 480)); // using a rectangle
        Right=FrameFlpd(Rect(640, 0, 640, 480)); // using a rectangle

        //Draw a circle in the middle of the left and right image (usefull for aligning both cameras)
        circle(Left,Point(Left.size().width/2,Left.size().height/2),10,Scalar(255,255,255),1);
        circle(Right,Point(Right.size().width/2,Right.size().height/2),10,Scalar(255,255,255),1);

        //Display left and right images
        imshow("Left",Left);
        imshow("Right", Right);

        char x = 0; // counter for chameleon task

        // Time and Frequency for tasks
        double T = 4.0;
        double F = 1/T;

        // Exact middle for servo ranges
        int RxRangeC = RxRangeM/2 + RxLm;
        int RyRangeC = RyRangeM/2 + RyBm;
        int LxRangeC = LxRangeM/2 + LxLm;
        int LyRangeC = LyRangeM/2 + LyTm;
        int NeckRangeC = NeckRange/2 + NeckR;

        // Wait for key to perform task
        int key = waitKey(10);
        switch (key) {
        case 's': // Move Neck in a sinusoidal manner
            for (double i = 0; i < T; i += 0.01) {
                Neck = (NeckRange/2) * sin (2 * M_PI * F * i) + NeckRangeC;
                sendCommand();
                Sleep(10); // 100 Hz roughly
            }
            break;
		case 'h': // Keep eyes parallel
			for (double i = 0; i < T; i += 0.01)
			{
				Rx = (RxRangeM/2) * sin (2 * M_PI * F * i) + RxRangeC;
				Lx = (LxRangeM/2) * sin (2 * M_PI * F * i) + LxRangeC;
                sendCommand();
				Sleep(10); // 100 Hz roughly
			}
			 break;
        case 'c': // Move eyes to random positions like a chameleon
        	// using a combination of using 1 second to move and 1 second to rest
        	// it takes 2s roughly to do 1 action and perform 5 in 10s
            while(x < 5) {
                srand (time(NULL)); // seed the rand function

                // Randomise the servo position from the range
                int aRx = rand() % RxRangeM + RxLm;
                int aRy = rand() % RyRangeM + RyBm;
                int aLx = rand() % LxRangeM + LxLm;
                int aLy = rand() % LyRangeM + LyTm;

                // Calculate the servo increments
                int RxChange = (aRx - Rx)/20;
                int RyChange = (aRy - Ry)/20;
                int LxChange = (aLx - Lx)/20;
                int LyChange = (aLy - Ly)/20;

                for (int i = 0; i < 20; i++) {
                    Rx+=RxChange;
                    Ry+=RyChange;
                    Lx+=LxChange;
                    Ly+=LyChange;
                    //Send new motor positions to the owl servos
                    sendCommand();
                    Sleep(50); // 20 increments, 50ms each, add up to 1s
                }
                x++;
                Sleep(1000);
            }
            break;
		case 'e': // Keep eyes stable as neck moves
			for (double i = 0; i < T; i += 0.01)
			{
                Neck = (NeckRange/2) * sin (2 * M_PI * F * i) + NeckRangeC;
				Rx = (RxRangeM/2) * sin (2 * M_PI * F * i) + RxRangeC;
				Lx = (LxRangeM/2) * sin (2 * M_PI * F * i) + LxRangeC;
                sendCommand();
				Sleep(10); // 100 Hz roughly
			}
			 break;
        case 'r': // Perform an eye roll
        	// reset servos to the bottom left
            Rx = RxLm;
            Lx = LxLm;
            Ry = RyBm;
            Ly = LyTm;
            sendCommand();

            // Instead of the full sine function, use half to keep it positive
            for (double i = 0; i < T/2; i += 0.025) {
            	// Sinusoidally increment the y-axis
                Ry = RyRangeM * sin (2 * M_PI * F * i) + RyBm;
                Ly = LyRangeM * sin (2 * M_PI * F * i) + LyBm;

                // Step increment the x-axis
                Rx += RxRangeM/80;
                Lx += LxRangeM/80;
                sendCommand();
                Sleep(25); // 40 Hz roughly
            }
            break;
        }

    } // END cursor control loop

    // close windows down
    destroyAllWindows();


#ifdef _WIN32
    RxPacket= OwlSendPacket (u_sock, CMD.c_str());
    closesocket(u_sock);
#else
    OwlSendPacket (clientSock, CMD.c_str());
    close(clientSock);
#endif
    exit(0); // exit here for servo testing only
}
