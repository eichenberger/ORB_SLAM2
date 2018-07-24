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


/* This is a wrapper that has no dependencies to other header files */


#ifndef ORBSLAM_H
#define ORBSLAM_H

#include <string>
#include <thread>
#include<opencv2/core/core.hpp>

namespace ORB_SLAM2 {
class MapPoint;
class System;
class KeyFrame;
}

using namespace std;
using namespace ORB_SLAM2;

class OrbKeyPoint
{
public:
    OrbKeyPoint(cv::Mat &_position, cv::Mat &_descriptors) :
        position(_position), descriptors(_descriptors)
    {
    }

    OrbKeyPoint()
    {}

    cv::Mat position;
    cv::Mat descriptors;
};

class OrbKeyFrame
{
public:
//    OrbKeyFrame(cv::Mat &_pose, cv::Mat &_cameraCenter) :
//        pose(_pose.clone()), cameraCenter(_cameraCenter.clone())
//    {
//    }

    OrbKeyFrame(cv::Mat &_pose, cv::Mat &_cameraCenter,
            cv::Mat &_kps, cv::Mat &_descriptors) :
        pose(_pose.clone()), cameraCenter(_cameraCenter.clone()),
        keyPoints(_kps.clone()), descriptors(_descriptors.clone())
    {
    }

    OrbKeyFrame()
    {}

    cv::Mat pose;
    cv::Mat cameraCenter;
    cv::Mat descriptors;
    cv::Mat keyPoints;
};

class OrbSlam
{
public:
    // Input sensor
    enum eSensor{
        MONOCULAR=0,
        STEREO=1,
        RGBD=2
    };

public:

    // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
    OrbSlam(const string &strVocFile, const string &strSettingsFile, const eSensor sensor, const bool bUseViewer = true);
    ~OrbSlam();

    // Proccess the given monocular frame
    // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
    // Returns the camera pose (empty if tracking fails).
    cv::Mat TrackMonocular(cv::Mat const &im, const double &timestamp);

    // This stops local mapping thread (map building) and performs only camera tracking.
    void ActivateLocalizationMode();
    // This resumes local mapping thread and performs SLAM again.
    void DeactivateLocalizationMode();

    // Returns true if there have been a big map change (loop closure, global BA)
    // since last call to this function
    bool MapChanged();

    // Reset the system (clear map)
    void Reset();

    // All threads will be requested to finish.
    // It waits until all threads have finished.
    // This function must be called before saving the trajectory.
    void Shutdown();

//    // Save camera trajectory in the TUM RGB-D dataset format.
//    // Only for stereo and RGB-D. This method does not work for monocular.
//    // Call first Shutdown()
//    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
//    void SaveTrajectoryTUM(const string &filename);
//
//    // Save keyframe poses in the TUM RGB-D dataset format.
//    // This method works for all sensor input.
//    // Call first Shutdown()
//    // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
//    void SaveKeyFrameTrajectoryTUM(const string &filename);
//
//    // Save camera trajectory in the KITTI dataset format.
//    // Only for stereo and RGB-D. This method does not work for monocular.
//    // Call first Shutdown()
//    // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
//    void SaveTrajectoryKITTI(const string &filename);

    // TODO: Save/Load functions
    // SaveMap(const string &filename);
    // LoadMap(const string &filename);

    // Information from most recent processed frame
    // You can call this right after TrackMonocular (or stereo or RGBD)
    std::vector<OrbKeyPoint> GetKeyPoints();
    std::vector<OrbKeyFrame> GetKeyFrames();

    bool NewKeyFrameSinceLastCall();

private:
    System *system;
    KeyFrame* lastKeyFrame;
};


#endif // ORBSLAM_H
