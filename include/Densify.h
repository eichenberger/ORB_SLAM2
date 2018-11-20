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

#ifndef DENSIFY_H
#define DENSIFY_H

#include <vector>
#include <list>
#include <mutex>
#include <opencv/cv.h>
#include <opencv2/features2d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include "KeyFrame.h"
#include "MapDrawer.h"
#include "Depth.h"

using namespace std;
using namespace cv;


namespace ORB_SLAM2
{

typedef struct
{
    Mat position;
    uint8_t brightness;
} DensePoint;

class StereoImage
{
public:
    StereoImage(const Mat _imLeft, Mat _imRight, KeyFrame *_kf):
        imLeft(_imLeft),
        imRight(_imRight),
        kf(_kf) {}
    Mat imLeft;
    Mat imRight;
    KeyFrame *kf;
};

class Densify
{
public:
    Densify(const string &strSettingPath, MapDrawer* pMapDrawer);
    ~Densify(){}

    void InsertKeyFrame(KeyFrame *kf,const Mat *imLeft,const Mat *imRight);

    void Run();
    void RequestFinish();

    Mat getDepthImage();
    Mat getDenseCloud();

protected:
    bool CheckFinish();
    bool mbFinished;
    std::mutex mMutexFinish;

    void GenerateDenseCloud();

    MapDrawer* mpMapDrawer;
    bool mbFinishRequested;

    float fx;
    float bf;

    Ptr<Depth> mDepth;
    std::mutex mDepthLock;


    vector<Mat> denseCloud;
    std::mutex mDenseLock;

    std::mutex mLock;
};

} //namespace ORB_SLAM

#endif

