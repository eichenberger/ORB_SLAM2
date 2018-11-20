#include <iostream>
#include <mutex>
#include "Depth.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/core/ocl.hpp>

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

Depth::Depth(double baseline): m_baseline(baseline)
{
    left_matcher = StereoBM::create();
}

void Depth::setBlockSize(int value)
{
    left_matcher->setBlockSize(value);
}

void Depth::setNumDisparities(int value)
{
    left_matcher->setNumDisparities(value);
}

void Depth::setPreFilterSize(int value)
{
    left_matcher->setPreFilterSize(value);
}

void Depth::setPreFilterCap(int value)
{
    left_matcher->setPreFilterCap(value);
}

void Depth::setMinDisparity(int value)
{
    left_matcher->setMinDisparity(value);
}

void Depth::setTextureThreshold(int value)
{
    left_matcher->setTextureThreshold(value);
}

void Depth::setUniquenessRatio(int value)
{
    left_matcher->setUniquenessRatio(value);
}

void Depth::setSpeckleWindowSize(int value)
{
    left_matcher->setSpeckleWindowSize(value);
}

void Depth::setSpeckleRange(int value)
{
    left_matcher->setSpeckleRange(value);
}

void Depth::setDisp12MaxDiff(int value)
{
    left_matcher->setDisp12MaxDiff(value);
}


void Depth::calculateDepth(const Mat &imLeft, const Mat &imRight)
{
    Mat tmpDisparity;
    left_matcher->compute(imLeft, imRight, tmpDisparity);

    Mat disparityFractional;
    tmpDisparity.convertTo(disparityFractional, CV_32F, 1.0/15.0);

    Mat tmpDepth;
    tmpDepth = m_baseline/disparityFractional;
    Mat filter;
    // TODO: replace 10.0 with dynamic value
    cv::threshold(tmpDepth, filter, 10.0, 1.0, THRESH_BINARY_INV);

    mDepth = filter.mul(tmpDepth);
}

const Mat& Depth::getDepthImage()
{
    return mDepth;
}
