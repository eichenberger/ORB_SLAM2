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

#ifndef ORBEXTRACTOROCV_H
#define ORBEXTRACTOROCV_H

#include <vector>
#include <list>
#include <opencv/cv.h>
#include <opencv2/features2d.hpp>
#include <opencv2/core.hpp>
#include "ORBextractor.h"


namespace ORB_SLAM2
{

class ORBextractorOCV : public ORBextractor
{
public:

    enum {HARRIS_SCORE=0, FAST_SCORE=1 };

    ORBextractorOCV(int _nfeatures, float _scaleFactor, int _nlevels,
             int _iniThFAST, int _minThFAST);

    ~ORBextractorOCV(){}

    // Compute the ORB features and descriptors on an image.
    // ORB are dispersed on the image using an octree.
    // Mask is ignored in the current implementation.
    void operator()( cv::InputArray image, cv::InputArray mask,
      std::vector<cv::KeyPoint>& keypoints,
      cv::OutputArray descriptors);


protected:
    cv::Ptr<cv::ORB> orb;

    void computeImagePyramid(cv::InputArray image);
};

} //namespace ORB_SLAM

#endif

