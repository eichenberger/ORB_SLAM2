#include <unistd.h>
#include <iostream>
#include <mutex>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/calib3d.hpp>

#include <pcl/kdtree/kdtree_flann.h>

#include "Densify.h"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

Densify::Densify(const string &strSettingPath):
    mbFinishRequested(false)
{
    cv::FileStorage fsSettings(strSettingPath, cv::FileStorage::READ);

    fx = fsSettings["Camera.fx"];
    fy = fsSettings["Camera.fx"];
    cx = fsSettings["Camera.cx"];
    cy = fsSettings["Camera.cy"];

    mDepth = makePtr<Depth>(fsSettings);
}

void Densify::Run()
{
#define SLEEP_TIME 50000
    mbFinished = false;
    while(1) {
        Mat imLeft, imRight, Q;

        usleep(SLEEP_TIME);
        if (stereoImages.size() <= 0) {
            continue;
        }

        {
            unique_lock<mutex> lock(mLock);
            list<StereoImage>::iterator stereoImage = stereoImages.begin();

            // Search the oldest verified image
            for (;stereoImage != stereoImages.end(); stereoImage++) {
                if (stereoImage->verified)
                    break;
            }

            if (stereoImage == stereoImages.end())
                continue;

            printf("Update dense cloud\n");
            if (stereoImage->imLeft.rows == 0 || stereoImage->imRight.rows == 0) {
                cout << "WTF0.1" << endl;
            }
            else {
                imLeft = stereoImage->imLeft.clone();
                imRight = stereoImage->imRight.clone();
                Q = stereoImage->Q.clone();
            }

            if (imLeft.rows == 0 || imRight.rows == 0)
                cout << "WTF1" << endl;

            // Remove all images that aren't verified yet
            for (list<StereoImage>::iterator it = stereoImages.begin();
                    stereoImage != stereoImage; it++) {
                stereoImages.pop_front();
            }
            stereoImages.pop_front();
        }

        {
            unique_lock<mutex> lock(mDepthLock);
            if (imLeft.rows >0 && imRight.rows > 0)
                mDepth->calculateDepth(imLeft, imRight);
            else
                cout << "WTF: " << imLeft.rows << "," << imRight.rows << endl;
        }

        if (imLeft.rows >0 && imRight.rows > 0)
            GenerateDenseCloud(Q, imLeft);

        if(CheckFinish())
            break;
    }
}

void Densify::GenerateDenseCloud(Mat Q, const Mat &image)
{
    unique_lock<mutex> lock(mDenseLock);
    Mat cloud;
    Mat depth = mDepth->getDepthImage();
    Mat QInv = Q.inv();
    bool validKdtree = false;

    pcl::KdTreeFLANN<PointXYZI> kdtree;
    if (denseCloud.size() > 0) {
        kdtree.setInputCloud(denseCloud.makeShared());
        validKdtree = true;
    }

    for (int x = 0; x < depth.cols; x++) {
        for (int y = 0; y < depth.rows; y++) {
            float currentDepth = depth.at<float>(y, x);
            if ( currentDepth > 0) {
                Mat pos(4, 1, CV_32F);
                // TODO: x, y are dependend on baseline
                pos.at<float>(0,0) = (x-cx)/fx*currentDepth;
                pos.at<float>(1,0) = (y-cy)/fy*currentDepth;
                pos.at<float>(2,0) = currentDepth;
                //pos.at<float>(0,3) = (float)image.at<uint8_t>(y, x)/255.0;
                pos.at<float>(3,0) = 1.0;

                // Transform point position to global coordinates
                pos = Q * pos;

                PointXYZI point;
                point.x = pos.at<float>(0,0);
                point.y = pos.at<float>(1,0);
                point.z = pos.at<float>(2,0);

                std::vector<int> index(1);
                std::vector<float> sqdist(1);

                PointXYZI *thePoint  = NULL;
                int *matches = NULL;
                if (validKdtree) {
                    // Growing radius with depth²
                    kdtree.nearestKSearch(point, 1, index, sqdist);

                    float rad_z = currentDepth*0.02;
                    if (abs(sqdist[0]) < rad_z) {
                        // Growing radius with depth
                        float rad_x = currentDepth*0.01;
                        float rad_y = currentDepth*0.01;

                        thePoint = &denseCloud.at(index[0]);

                        Mat cloudPos(4, 1, CV_32F);
                        cloudPos.at<float>(0,0) = thePoint->x;
                        cloudPos.at<float>(1,0) = thePoint->y;
                        cloudPos.at<float>(2,0) = thePoint->z;
                        cloudPos.at<float>(3,0) = 1.0;

                        Mat diff = cloudPos - pos;
                        diff = QInv*diff;
                        diff = diff / diff.at<float>(3,0);
                        if ((abs(diff.at<float>(0,0)) > rad_x) ||
                            (abs(diff.at<float>(1,0)) > rad_y) ||
                            (abs(diff.at<float>(2,0)) > rad_z))
                            thePoint = NULL;
                        else
                            matches = &mDenseMatches[index[0]];
                    }
                }

                if (thePoint) {
                    // Update position and intensiti with new values
                    thePoint->x = thePoint->x * *matches + point.x;
                    thePoint->y = thePoint->y * *matches + point.y;
                    thePoint->z = thePoint->z * *matches + point.z;
                    thePoint->intensity = thePoint->intensity * *matches + point.intensity;
                    *matches++;
                    thePoint->x /= *matches;
                    thePoint->y /= *matches;
                    thePoint->z /= *matches;
                    thePoint->intensity /= *matches;
                }
                else {
                    point.intensity = (float)image.at<uint8_t>(y, x)/255.0;

                    denseCloud.push_back(point);
                    mDenseMatches.push_back(1);
                }
        }
        }
    }

//    VoxelGrid<PointXYZI> spareFilter;
//    spareFilter.setInputCloud(denseCloud.makeShared());
//    spareFilter.setLeafSize (0.2f, 0.2f, 0.2f);
//    spareFilter.filter(denseCloud);
}

void Densify::InsertKeyFrame(KeyFrame *kf, const Mat *imLeft, const Mat *imRight)
{
    unique_lock<mutex> lock(mLock);
    if (imLeft->rows == 0 || imRight->rows == 0)
        cout << "WTF0" << endl;
    stereoImages.push_back(StereoImage(imLeft->clone(),
                imRight->clone(),
                kf->GetPoseInverse().clone(),
                kf->mnFrameId,
                false));
    cout << "rows left " << stereoImages.back().imLeft.rows << endl;
    cout << "rows right " << stereoImages.back().imRight.rows << endl;
}

void Densify::RemoveKeyFrame(long unsigned int id)
{
    unique_lock<mutex> lock(mLock);
    printf("Remove frame %d\n", id);
    for (auto it = stereoImages.begin(); it != stereoImages.end(); it++) {
        if (it->id == id) {
            printf("Found remove now %d\n", id);
            stereoImages.erase(it);
            break;
        }
    }
}

void Densify::SetVerified(long unsigned int id)
{
    unique_lock<mutex> lock(mLock);
    for (auto img = stereoImages.begin(); img != stereoImages.end(); img++) {
        if (img->id == id) {
            img->verified = true;
            break;
        }
    }
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

PointCloud<PointXYZI> Densify::getDenseCloud()
{
    unique_lock<mutex> lock(mDenseLock);
    return denseCloud;
}

void Densify::Reset()
{
    unique_lock<mutex> depthLock(mDepthLock);
    unique_lock<mutex> denseLock(mDenseLock);
    unique_lock<mutex> loopLock(mLock);
    denseCloud.clear();
    stereoImages.clear();
}
