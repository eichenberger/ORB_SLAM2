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

#include "System.h"
#include "OrbSlam.h"

OrbSlam::OrbSlam(const string &strVocFile, const string &strSettingsFile, const eSensor sensor,
               const bool bUseViewer)
{
    this->system = new System(strVocFile, strSettingsFile, (const System::eSensor&)sensor, bUseViewer);
}

OrbSlam::~OrbSlam()
{
    delete this->system;
}

cv::Mat OrbSlam::TrackMonocular(cv::Mat const &im, const double &timestamp)
{
    return this->system->TrackMonocular(im, timestamp);
}

void OrbSlam::ActivateLocalizationMode()
{
    this->system->ActivateLocalizationMode();
}

void OrbSlam::DeactivateLocalizationMode()
{
    this->system->DeactivateLocalizationMode();
}

bool OrbSlam::MapChanged()
{
    this->system->MapChanged();
}

void OrbSlam::Reset()
{
    this->system->Reset();
}

void OrbSlam::Shutdown()
{
    this->system->Shutdown();
}

std::vector<OrbKeyPoint> OrbSlam::GetKeyPoints()
{
    static std::vector<OrbKeyPoint> keyPoints;
    std::vector<MapPoint*> mp = this->system->GetMap()->GetAllMapPoints();

    keyPoints.clear();
    for(auto const& value: mp) {
        cv::Mat pos = value->GetWorldPos();
        cv::Mat desc = value->GetDescriptor();
        keyPoints.push_back(OrbKeyPoint(pos, desc));
    }

    return keyPoints;
}

std::vector<OrbKeyFrame> OrbSlam::GetKeyFrames()
{
    static std::vector<OrbKeyFrame> keyFrames;
    std::vector<KeyFrame*> kf = this->system->GetMap()->GetAllKeyFrames();

    keyFrames.clear();
    for(auto const& value: kf) {
        cv::Mat pose = value->GetPose();
        cv::Mat cameraCenter = value->GetCameraCenter();
        const vector<cv::KeyPoint> kps = value->mvKeys;
        cv::Mat points2d = cv::Mat(kps.size(), 2, CV_64F);
        int i = 0;
        for (auto const& kp: kps) {
            points2d.at<double>(i, 0) = kp.pt.x;
            points2d.at<double>(i, 1) = kp.pt.y;
            i++;
        }
        cv::Mat descriptors = value->mDescriptors;
        keyFrames.push_back(OrbKeyFrame(pose, cameraCenter, points2d,
                    descriptors));
    }

    return keyFrames;
}

bool OrbSlam::NewKeyFrameSinceLastCall()
{
    static int lastNumberOfKeyframes = 0;
    std::vector<KeyFrame*> kf = this->system->GetMap()->GetAllKeyFrames();

    if (lastNumberOfKeyframes < kf.size()) {
        lastNumberOfKeyframes = kf.size();
        return true;
    }
    else
        return false;
}
