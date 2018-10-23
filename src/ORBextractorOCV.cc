#include <iostream>
#include <mutex>
#include "ORBextractorOCV.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/core/ocl.hpp>

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
    static ocl::Context &oclContext = ocl::Context::getDefault();
    static const ocl::Device &oclDevice = oclContext.device(0);
    static bool isIntel = oclDevice.isIntel();
    static std::mutex gpulock;
    if(image.empty())
        return;

    /* Unfortunately libpciaccess is not multithreading capable therefore
     * we need to lock access to Intel Graphic cards.
     * However, this works with vivante */
    if (isIntel)
      gpulock.lock();

    mvImagePyramid.clear();
    orb->detectAndComputeWithPyramid(image, mask, keypoints, descriptors, &mvImagePyramid);
    if (isIntel)
      gpulock.unlock();
}

void ORBextractorOCV::computeImagePyramid(cv::InputArray image)
{
    mvImagePyramid[0] = image.getMat();
    for (int i = 1; i < nlevels; i++) {
        cv::resize(mvImagePyramid[0], mvImagePyramid[i], Size(), 1/mvScaleFactor[i], 1/mvScaleFactor[i]);
    }
}


