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
        cerr << endl << "Usage: ./stereo_antmicro path_to_cam_right path_to_cam_left right_file left_file" << endl;
        return 1;
    }

    // Retrieve paths to images
    VideoCapture cap1;
    cap1.open("v4l2src device=\"" + String(argv[1]) +
            "\" ! video/x-raw,format=RGB16,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! appsink");
    if (!cap1.isOpened()) {
        cerr << "ERROR! Unable to open camera1\n";
        return -1;
    }

    VideoCapture cap2;
    cap2.open("v4l2src device=\"" + String(argv[2]) +
            "\" ! video/x-raw,format=RGB16,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=BGR ! appsink");
    if (!cap2.isOpened()) {
        cerr << "ERROR! Unable to open camera1\n";
        return -1;
    }

#define HEIGHT 480
#define WIDTH 640

    Mat split(HEIGHT, WIDTH*2, CV_8UC3);
    Mat imFull1; // Copy constructor
    Mat imFull2; // Copy constructor

    int codec = VideoWriter::fourcc('M', 'J', 'P', 'G');  // select desired codec (must be available at runtime)

    VideoWriter outputVideo1;
    outputVideo1.open(argv[3], codec, 25.0, Size(WIDTH, HEIGHT), true);
    VideoWriter outputVideo2;
    outputVideo2.open(argv[4], codec, 25.0, Size(WIDTH, HEIGHT), true);

    //cout << "Split Size: " << left.cols << "x" << left.rows << endl;
    Mat left(split, Rect(0, 0, WIDTH, HEIGHT)); // Copy constructor
    Mat right(split, Rect(WIDTH, 0, WIDTH, HEIGHT)); // Copy constructor
    char key = 0;
    while (key != 'q') {
        cap1.read(imFull1);
        cap2.read(imFull2);

        imFull1.copyTo(left);
        imFull2.copyTo(right);

        cv::imshow("Image", split);

        outputVideo1.write(imFull1);
        outputVideo2.write(imFull2);
        key = cv::waitKey(1);
    }
    return 0;
}
