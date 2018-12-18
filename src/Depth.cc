#include <iostream>
#include <mutex>
#include "Depth.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/core/ocl.hpp>

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

Depth::Depth(const cv::FileStorage &fsSettings)
{
    left_matcher = StereoSGBM::create();

    float cn = fsSettings["stereosgbm.cn"];
    float sgbm_preFilterCap = fsSettings["stereosgbm.preFilterCap"];
    float sgbm_SADWindowSize = fsSettings["stereosgbm.SADWindowSize"];
    float sgbm_minDisparity = fsSettings["stereosgbm.minDisparity"];
    float sgbm_speckleRange = fsSettings["stereosgbm.speckleRange"];
    float sgbm_disp12MaxDiff = fsSettings["stereosgbm.disp12MaxDiff"];
    float sgbm_uniquenessRatio  = fsSettings["stereosgbm.uniquenessRatio "];
    float sgbm_speckleWindowSize  = fsSettings["stereosgbm.speckleWindowSize "];
    float sgbm_numberOfDisparities = fsSettings["stereosgbm.numberOfDisparities"];

    m_baseline = fsSettings["Camera.bf"];

    left_matcher->setPreFilterCap(sgbm_preFilterCap);
    left_matcher->setBlockSize (sgbm_SADWindowSize > 0 ? sgbm_SADWindowSize : 3);
    left_matcher->setP1(8 * cn * sgbm_SADWindowSize * sgbm_SADWindowSize);
    left_matcher->setP2(32 * cn * sgbm_SADWindowSize * sgbm_SADWindowSize);
    left_matcher->setNumDisparities(sgbm_numberOfDisparities);
    left_matcher->setMinDisparity(sgbm_minDisparity);
    left_matcher->setUniquenessRatio(sgbm_uniquenessRatio);
    left_matcher->setSpeckleWindowSize(sgbm_speckleWindowSize);
    left_matcher->setSpeckleRange(sgbm_speckleRange);
    left_matcher->setDisp12MaxDiff(sgbm_disp12MaxDiff);

    left_matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);

    right_matcher = ximgproc::createRightMatcher(left_matcher.dynamicCast<StereoMatcher>());
    wls_filter = ximgproc::createDisparityWLSFilter(left_matcher);
    wls_filter->setLambda(4000.0);
    wls_filter->setSigmaColor(1.0);
}

void Depth::calculateDepth(const Mat &imLeft, const Mat &imRight)
{
    Mat disparityL, disparityR, disparityFiltered;

    left_matcher->compute(imLeft, imRight, disparityL);
    right_matcher->compute(imRight, imLeft, disparityR);

    wls_filter->filter(disparityL, imLeft, disparityFiltered, disparityR);
    Mat confidence = wls_filter->getConfidenceMap();

    Mat filter;

    // threshold(confidence, filter, 100, 1.0, THRESH_BINARY);


    Mat disparityFractional;
    // Somehow baseline is of by x10, therefore divide depth by 10
    disparityFiltered.convertTo(disparityFractional, CV_32F, 1.0/10.0);
    //disparityFiltered.convertTo(disparityFractional, CV_32F, 1.0);

    mDepth = m_baseline/disparityFractional;

    // Remove uncertinty
    // mDepth = mDepth.mul(filter, 1.0);
}

const Mat& Depth::getDepthImage()
{
    return mDepth;
}
