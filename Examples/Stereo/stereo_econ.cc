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

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps);

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

    cap.set(CAP_PROP_FPS, 30);
    cap.set(CAP_PROP_FRAME_WIDTH, 752);
    cap.set(CAP_PROP_FRAME_HEIGHT, 480);
//    cap.set(CAP_PROP_AUTO_EXPOSURE, 1);
    cap.set(CAP_PROP_EXPOSURE, 5);

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
    ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::STEREO,true);

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    // Main loop
    cv::Mat imLeft, imRight, imFull;
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
}

void LoadImages(const string &strPathToSequence, vector<string> &vstrImageLeft,
                vector<string> &vstrImageRight, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";
    string strPrefixRight = strPathToSequence + "/image_1/";

    const int nTimes = vTimestamps.size();
    vstrImageLeft.resize(nTimes);
    vstrImageRight.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageLeft[i] = strPrefixLeft + ss.str() + ".png";
        vstrImageRight[i] = strPrefixRight + ss.str() + ".png";
    }
}
