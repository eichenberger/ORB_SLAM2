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
#include "Depth.h"
#include "Map.h"

using namespace std;
using namespace cv;

namespace ORB_SLAM2
{

class StereoImage
{
public:
    StereoImage(Mat _imLeft, Mat _imRight, Mat _Q,
            long unsigned int _id, bool _verified):
        imLeft(_imLeft),
        imRight(_imRight),
        Q(_Q),
        id(_id),
        verified(_verified){}
    Mat imLeft;
    Mat imRight;
    Mat Q;
    long unsigned int id;
    bool verified;
};

class PointXYZI
{
public:
    float x;
    float y;
    float z;
    float intensity;
};

#define PointCloud vector

class Densify
{
public:
    Densify(const string &strSettingPath);
    ~Densify(){}

    void InsertKeyFrame(KeyFrame *kf,const Mat *imLeft,const Mat *imRight);
    void SetVerified(long unsigned int id);
    void RemoveKeyFrame(long unsigned int id);

    void Run();
    void RequestFinish();

    Mat getDepthImage();
    PointCloud<PointXYZI> getDenseCloud();

    void Reset();

    int enabled;

protected:
    bool CheckFinish();
    bool mbFinished;
    std::mutex mMutexFinish;

    void GenerateDenseCloud(Mat Q, const Mat &image);

    bool mbFinishRequested;

    float fx;
    float fy;
    float cx;
    float cy;

    int topmargin;
    int bottommargin;
    int leftmargin;
    int rightmargin;

    Ptr<Depth> mDepth;
    std::mutex mDepthLock;

    PointCloud<PointXYZI> denseCloud;
    vector<int> mDenseMatches;
    std::mutex mDenseLock;

    std::mutex mLock;
    list<StereoImage> stereoImages;

};

} //namespace ORB_SLAM

#endif

