/* File : example.i */
%module orbslam

%include <opencv.i>
%cv_instantiate_all_defaults

%include <std_string.i>

%{
#include "../include/OrbSlam.h"
%}

%include <std_vector.i>
namespace std {
  %template(OrbKeyPointVector) vector<OrbKeyPoint>;
  %template(OrbKeyFrameVector) vector<OrbKeyFrame>;
};

/* Let's just grab the original header file here */
%include "../include/OrbSlam.h"

