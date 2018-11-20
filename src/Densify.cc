#include <unistd.h>
#include <iostream>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/calib3d.h>

#include "Densify.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

Densify::Densify(const string &strSettingPath, MapDrawer* pMapDrawer):
    mpMapDrawer(pMapDrawer),
    mbFinishRequested(false)
{
    cv::FileStorage fsSettings(strSettingPath, cv::FileStorage::READ);

    fx = fsSettings["Camera.fx"];
    bf = fsSettings["Camera.bf"];

    mDepth = makePtr<Depth>(bf);

    mDepth->setBlockSize(fsSettings["stereobm.blockSize"]);
    mDepth->setNumDisparities(fsSettings["stereobm.numDisparities"]);
    mDepth->setPreFilterSize(fsSettings["stereobm.preFilterSize"]);
    mDepth->setPreFilterCap(fsSettings["stereobm.preFilterCap"]);
    mDepth->setMinDisparity(fsSettings["stereobm.minDisparity"]);
    mDepth->setTextureThreshold(fsSettings["stereobm.textureThreshold"]);
    mDepth->setUniquenessRatio(fsSettings["stereobm.uniquenessRatio"]);
    mDepth->setSpeckleWindowSize(fsSettings["stereobm.speckleWindowSize"]);
    mDepth->setSpeckleRange(fsSettings["stereobm.speckleRange"]);
    mDepth->setDisp12MaxDiff(fsSettings["stereobm.disp12MaxDiff"]);
}

void Densify::Run()
{
    mbFinished = false;
    while(1) {
        Mat imLeft, imRight;
        KeyFrame *kf;
        // printf("Run densify\n");

        mLock.lock();
        if (stereoImages.size() > 0) {
            imLeft = stereoImages.front().imLeft.clone();
            imRight = stereoImages.front().imRight.clone();
            kf = stereoImages.front().kf;
            stereoImages.pop_front();
            mLock.unlock();
        }
        else {
            mLock.unlock();
            usleep(50000);
            continue;
        }


        mDepthLock.lock();
        mDepth->calculateDepth(imLeft, imRight);
        mDepthLock.unlock();

        GenerateDenseCloud(kf);

        if(CheckFinish())
            break;
        usleep(50000);
    }
}

void Densify::GenerateDenseCloud(const KeyFrame *kf)
{
    unique_lock<mutex> lock(mDenseLock);
    Mat Q = kf->GetPose();
    vector<Mat> cloud;
    reprojectImageTo3D(mDepth->getDepthImage(), cloud, Q);
    dense_cloud.insert(dense_cloud.end(), cloud.begin(), cloud.end());
}

void Densify::InsertKeyFrame(KeyFrame *kf, const Mat *imLeft, const Mat *imRight)
{
    unique_lock<mutex> lock(mLock);
    stereoImages.push_back(StereoImage(imLeft->clone(), imRight->clone(), kf));
}

bool Densify::CheckFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
}

void Densify::RequestFinish()
{
    unique_lock<mutex> lock(mMutexFinish);
    mbFinishRequested = true;
}

Mat Densify::getDepthImage()
{
    unique_lock<mutex> lock(mDepthLock);
    return mDepth->getDepthImage().clone();
}

Mat Densify::getDenseCloud()
{
    unique_lock<mutex> lock(mDenseLock);

}
