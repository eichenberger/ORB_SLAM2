#include <iostream>
#include "ORBextractorOCV.h"
#include "opencv2/imgproc.hpp"

using namespace std;
using namespace cv;
using namespace ORB_SLAM2;

ORBextractorOCV::ORBextractorOCV(int _nfeatures, float _scaleFactor, int _nlevels,
             int _iniThFAST, int _minThFAST): ORBextractor(_nfeatures, _scaleFactor, _nlevels,
                 _iniThFAST, _minThFAST)

{
    orb = ORB::create(nfeatures, scaleFactor, nlevels);
}


// Compute the ORB features and descriptors on an image.
// ORB are dispersed on the image using an octree.
// Mask is ignored in the current implementation.
void ORBextractorOCV::operator()( cv::InputArray image, cv::InputArray mask,
  std::vector<cv::KeyPoint>& keypoints,
  cv::OutputArray descriptors)
{
    (void) mask;
    if(image.empty())
        return;
    orb->detect(image, keypoints);
    orb->compute(image, keypoints, descriptors);
    computeImagePyramid(image);
}

void ORBextractorOCV::computeImagePyramid(cv::InputArray image)
{
    mvImagePyramid[0] = image.getMat();
    for (int i = 1; i < nlevels; i++) {
        cv::resize(mvImagePyramid[0], mvImagePyramid[i], Size(), 1/mvScaleFactor[i], 1/mvScaleFactor[i]);
    }
}


