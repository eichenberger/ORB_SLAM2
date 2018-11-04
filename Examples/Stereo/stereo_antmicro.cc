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

    // Read rectification parameters
    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        cerr << "ERROR: Wrong path to settings" << endl;
        return -1;
    }

    cv::Mat K_l, K_r, P_l, P_r, R_l, R_r, D_l, D_r;
    fsSettings["LEFT.K"] >> K_l;
    fsSettings["RIGHT.K"] >> K_r;

    fsSettings["LEFT.P"] >> P_l;
    fsSettings["RIGHT.P"] >> P_r;

    fsSettings["LEFT.R"] >> R_l;
    fsSettings["RIGHT.R"] >> R_r;

    fsSettings["LEFT.D"] >> D_l;
    fsSettings["RIGHT.D"] >> D_r;

    int rows_l = fsSettings["LEFT.height"];
    int cols_l = fsSettings["LEFT.width"];
    int rows_r = fsSettings["RIGHT.height"];
    int cols_r = fsSettings["RIGHT.width"];

    if(K_l.empty() || K_r.empty() || P_l.empty() || P_r.empty() || R_l.empty() || R_r.empty() || D_l.empty() || D_r.empty() ||
            rows_l==0 || rows_r==0 || cols_l==0 || cols_r==0)
    {
        cerr << "ERROR: Calibration parameters to rectify stereo are missing!" << endl;
        return -1;
    }

    cv::Mat M1l,M2l,M1r,M2r;
    cv::initUndistortRectifyMap(K_l,D_l,R_l,P_l.rowRange(0,3).colRange(0,3),cv::Size(cols_l,rows_l),CV_32F,M1l,M2l);
    cv::initUndistortRectifyMap(K_r,D_r,R_r,P_r.rowRange(0,3).colRange(0,3),cv::Size(cols_r,rows_r),CV_32F,M1r,M2r);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    while (1)
    {
        cv::Mat imLeft, imRight, imFull1, imFull2, imLeftRect, imRightRect;
        cap1.read(imFull1);
        cap2.read(imFull2);

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


        cv::remap(imLeft,imLeftRect,M1l,M2l,cv::INTER_LINEAR);
        cv::remap(imRight,imRightRect,M1r,M2r,cv::INTER_LINEAR);

        // Pass the images to the SLAM system
        SLAM.TrackStereo(imLeftRect,imRightRect,tframe);
    }

    // Stop all threads
    SLAM.Shutdown();

    return 0;
}
