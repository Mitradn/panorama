#define _CRT_SECURE_NO_DEPRECATE
#include <iostream>
#include <string>     // std::string, std::to_string
#include <vector>
#include <stdio.h>


#include <opencv2/highgui.hpp> // OpenCV window I/O
#include "opencv2/video/tracking.hpp"
#include "opencv2/stitching/detail/matchers.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"


using namespace cv;
using namespace std;

#define print(msg) std::cout << msg << std::endl

#ifndef _DEBUG_
#   define    _DEBUG_                      0
#endif
#ifndef _ENABLE_DISPLAY_
#   define    _ENABLE_DISPLAY_TRACKING     0
#   define    _ENABLE_DISPLAY_WARPING      0
#   define    _ENABLE_DISPLAY_MATCHING     0
#endif

//
const string videoPath = "C:/Users/Mana/Desktop/360/file_c10dc48f.mp4";
const string dataPath = "C:/Users/Mana/Desktop/360/file_8c51a216.txt";
const double xScale = 15;
const double yScale = 2;

double workScale = 1;

const bool rotateFrame = true;

const double videoFps =50.0062;

int warpWidth;

const float stitchedImgWidth = 4096;

// Parameters for Shi-Tomasi algorithm
const int maxCorners = 50;
const int minCorners = 10;
const double qualityLevel = 0.01; //[0.01 - 1500]
const double minDistance = 10;
const int blockSize = 3, gradientSize = 0;
const bool useHarrisDetector = false;
const double k = 0.04;

// Parameters for KLT tracking algorithm
const cv::Size subPixWinSize(10, 10), winSize(31, 31);
const cv::TermCriteria termcrit(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, 20, 0.03);
const float KLTerrorThrsh = 10.0;

// Parameters for compute translation algorithm
const float translationThrsh = 100.0;

// Parameters for Color Correction Coefficients algorithm
const bool colorCorr = false;
const float weightAlpha = 0.6;

// EqualizeIntensity
const bool histEqu = false;
