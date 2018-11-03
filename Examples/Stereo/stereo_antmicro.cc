/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include <sys/time.h>
#include<iostream>
#include<algorithm>
#include<fstream>
#include<iomanip>
#include<chrono>

#include<opencv2/core/core.hpp>
#include<opencv2/videoio.hpp>

#include<System.h>
#include <unistd.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./stereo_antmicro path_to_vocabulary path_to_settings path_to_cam_right path_to_cam_left" << endl;
        return 1;
    }

    // Retrieve paths to images
    VideoCapture cap1;
    cap1.open("v4l2src device=\"" + String(argv[3]) +
            "\" ! video/x-raw,format=RGB16,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! appsink");
    if (!cap1.isOpened()) {
        cerr << "ERROR! Unable to open camera1\n";
        return -1;
    }

    VideoCapture cap2;
    cap2.open("v4l2src device=\"" + String(argv[4]) +
            "\" ! video/x-raw,format=RGB16,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! appsink");
    if (!cap2.isOpened()) {
        cerr << "ERROR! Unable to open camera1\n";
        return -1;
    }

#define WIDTH 640
#define HEIGHT 480
#if 1
    Mat split(HEIGHT, WIDTH*2, CV_8UC3);
    Mat imFull1; // Copy constructor
    Mat imFull2; // Copy constructor

    //cout << "Split Size: " << left.cols << "x" << left.rows << endl;
    Mat left(split, Rect(0, 0, WIDTH, HEIGHT)); // Copy constructor
    Mat right(split, Rect(WIDTH, 0, WIDTH, HEIGHT)); // Copy constructor
    while (1) {
        cap1.read(imFull1);
        cap2.read(imFull2);

        imFull1.copyTo(left);
        imFull2.copyTo(right);

        cv::imshow("Image1", split);
        cv::waitKey(1);
    }
    return 0;
#else
    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    cv::Mat imLeft, imRight;
    while (1)
    {
        cap1.read(imFull1);
        cap2.read(imFull2);

        cv::imshow("Image1", imFull1);
        cv::imshow("Image2", imFull2);
        cv::waitKey(0);

        cv::cvtColor(imFull1, imRight,  cv::COLOR_BGR2GRAY);
        cv::cvtColor(imFull2, imLeft,  cv::COLOR_BGR2GRAY);

        // Read left and right images from file
        struct timeval  tv;
        gettimeofday(&tv, NULL);
        double tframe = tv.tv_sec + tv.tv_usec/1000000 ;

        if(imLeft.empty())
        {
            cout << "No lef timage received\n";
            return 1;
        }

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeft,imRight,tframe);

#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif
    }

    // Stop all threads
    SLAM.Shutdown();

//    // Tracking time statistics
//    sort(vTimesTrack.begin(),vTimesTrack.end());
//    float totaltime = 0;
//    for(int ni=0; ni<nImages; ni++)
//    {
//        totaltime+=vTimesTrack[ni];
//    }
//    cout << "-------" << endl << endl;
//    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
//    cout << "mean tracking time: " << totaltime/nImages << endl;
//
//    // Save camera trajectory
//    SLAM.SaveTrajectoryKITTI("CameraTrajectory.txt");

    return 0;
#endif
}
