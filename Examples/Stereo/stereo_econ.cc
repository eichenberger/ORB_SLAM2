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

#include "xunit_lib_tara.h"

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    if(argc != 4)
    {
        cerr << endl << "Usage: ./stereo_econ path_to_vocabulary path_to_settings path_to_video" << endl;
        return 1;
    }

    // Retrieve paths to images
    VideoCapture cap;
    cap.open(String(argv[3]));
    if (!cap.isOpened()) {
        cerr << "ERROR! Unable to open camera\n";
        return -1;
    }

    cap.set(CAP_PROP_FRAME_WIDTH, 752);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);

    BOOL ret = InitExtensionUnit("usb-0000:00:14.0-1/input2");
    if (ret != SUCCESS){
        cerr << "ERROR: init extension unit" << endl;
        return -1;
    }
    ret = SetAutoExposureStereo();
    if (ret != SUCCESS){
        cerr << "ERROR: set auto exposure" << endl;
        return -1;
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    cv::Mat imFull;
    cv::Mat imLeft, imRight;
    while (1)
    {
        cap.read(imFull);
        // Read left and right images from file
        cv::extractChannel(imFull, imLeft, 1);
        cv::extractChannel(imFull, imRight, 2);
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

    return 0;
}
