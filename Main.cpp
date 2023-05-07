#include "Main.h";


/*
Convert radian to degree
*/
double Rad2Deg(double radian) {
	return radian * (180.0 / CV_PI);
}

/*
Convert degree to radian
*/
double Deg2Rad(double degree) {
	return degree * (CV_PI / 180.0);
}

/***************************************************************************************************
* draw corners in image
*    Input: image and corners
*    Output: image with corners
***************************************************************************************************/
void DrawCorners(cv::Mat imgInput, vector<cv::Point2f> corners, cv::Mat& imgOutput)
{
	cv::RNG rng(12345);
	imgInput.copyTo(imgOutput);
	// Draw corners in first frame
	for (size_t i = 0; i < corners.size(); i++)
	{
		circle(imgOutput, corners[i], 4,
			cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 256), rng.uniform(0, 256)), FILLED);
	}
}

/***************************************************************************************************
* draw match corners
*    Input: left and right image, left and right image corners, match corners
*    Output: image with match corners
***************************************************************************************************/
void DrawMatchCorners(cv::Mat imgL, cv::Mat imgR, vector<cv::Point2f> cornersL,
	vector<cv::Point2f> cornersR, cv::Mat &imgOutput) {
	DrawCorners(imgL, cornersL, imgL);
	DrawCorners(imgR, cornersR, imgR);
	// Horizontally concat previous and current frames
	cv::hconcat(imgL, imgR, imgOutput);

	// Draw match between previous and current frame match corners
	cv::Point2f strat;
	cv::Point2f end;
	for (int i = 0; i < cornersL.size(); i++)
	{
		strat = cornersL[i];
		end.x = cornersR[i].x + imgL.cols;
		end.y = cornersR[i].y;
		cv::line(imgOutput, strat, end, cv::Scalar(0, 0, 255), 2, 8);
	}
}

/***************************************************************************************************
* Read and parse gyroscope data.
*    Input: txt file path and fps.
*    Output: yaw vector based on fps.
***************************************************************************************************/
void ReadSensorData(const string txtStringPath, double fps, vector<double> &yaw) {
	char *txtPath = new char[txtStringPath.length() + 1];
	strcpy(txtPath, txtStringPath.c_str());

	double framePeriod = 1 / fps;
	double frameTime = 0;

	vector<double> time;
	double timeData = 0;
	double timeOffset = 0;

	//vector<double> yaw;
	double yawData = 0;
	double yawOffset = 0;

	/*vector<double> pitch;
	double pitchData = 0;

	vector<double> roll;
	double rollData = 0;*/

	char buf[1000];

	// read file
	FILE *fp = fopen(txtPath, "r");
	bool validFlg = true;

	int counter = 0;

	// Read first row ...
	if (fgets(buf, 1000, fp) != NULL) {
		// Yaw
		yawData = Deg2Rad(atof(buf + 2));
		yawOffset = yawData;

		// Time
		fgets(buf, 1000, fp);
		timeData = atof(buf + 2) * 0.001;
		timeOffset = timeData;
		timeOffset = 0;

		// Pitch
		//fgets(buf, 1000, fp);
		//pitchData = atof(buf + 9);

		//// Roll
		//fgets(buf, 1000, fp);
		//rollData = atof(buf + 8);

		// Data storage in vectors
		time.reserve(counter + 1);
		time.push_back(timeData - timeOffset);

		yaw.reserve(counter + 1);
		yaw.push_back(yawData - yawOffset);

		/*pitch.reserve(counter + 1);
		pitch.push_back(pitchData);

		roll.reserve(counter + 1);
		roll.push_back(rollData);*/

		//
		validFlg = true;
		counter += 1;
	}


	while (validFlg) {
		if (fgets(buf, 1000, fp) != NULL) {
			frameTime = yaw.size() * framePeriod;

			// Yaw
			yawData = Deg2Rad(atof(buf + 2));

			// Time
			fgets(buf, 1000, fp);
			timeData = atof(buf + 2) * 0.001 - timeOffset;

			if (timeData < frameTime) {
				//fgets(buf, 1000, fp);
				/*fgets(buf, 1000, fp);
				fgets(buf, 1000, fp);*/
				continue;
			}

			//// Pitch
			//fgets(buf, 1000, fp);
			//pitchData = atof(buf + 9);

			//// Roll
			//fgets(buf, 1000, fp);
			//rollData = atof(buf + 8);

			// Data storage in vectors
			time.reserve(counter + 1);
			time.push_back(timeData);

			yaw.reserve(counter + 1);
			yaw.push_back(yawData - yawOffset);

			/*pitch.reserve(counter + 1);
			pitch.push_back(pitchData);

			roll.reserve(counter + 1);
			roll.push_back(rollData);*/

			//
			counter += 1;
		}
		else
			validFlg = false;
	}

}

/***************************************************************************************************
* Shi-Tomasi or Harris corner detector.
*    Input:  image matrix.
*    Output: corners.
***************************************************************************************************/
void CornersDetector(const cv::Mat grayImg, vector<cv::Point2f> &corners) {
	// Apply corner detection
	cv::goodFeaturesToTrack(grayImg,
		corners,
		maxCorners,
		qualityLevel,
		minDistance,
		noArray(),
		blockSize,
		gradientSize,
		useHarrisDetector,
		k);
	cv::cornerSubPix(grayImg, corners, subPixWinSize, cv::Size(-1, -1), termcrit);
}

/***************************************************************************************************
*  Compute mean intensity
***************************************************************************************************/
void MeanIntensity(cv::Mat inputImg, cv::Scalar &meanIntensity) {
	inputImg.convertTo(inputImg, CV_32F);
	meanIntensity = cv::mean(inputImg);
	meanIntensity[3] = 0;
}

/***************************************************************************************************
*  Color Correction Coefficients
***************************************************************************************************/
void ColorCorrectionCoefficients(cv::Scalar prvImgMeanIntensity, cv::Scalar curImgMeanIntensity,
	cv::Scalar globalMeanIntensity, float weight, cv::Scalar &alpha) {
	alpha[0] = (globalMeanIntensity[0] * weight + prvImgMeanIntensity[0] * (1 - weight)) /
		curImgMeanIntensity[0];
	alpha[1] = (globalMeanIntensity[1] * weight + prvImgMeanIntensity[1] * (1 - weight)) /
		curImgMeanIntensity[1];
	alpha[2] = (globalMeanIntensity[2] * weight + prvImgMeanIntensity[2] * (1 - weight)) /
		curImgMeanIntensity[2];
	alpha[3] = 0;
}

/***************************************************************************************************
*  Apply color correction
***************************************************************************************************/
void ApplyColorCorrection(cv::Mat &inputImg, cv::Scalar alpha) {
	inputImg.convertTo(inputImg, CV_32F);
	cv::multiply(inputImg, alpha, inputImg);
	inputImg.convertTo(inputImg, CV_8U);
}

/***************************************************************************************************
*  Histogram equalization
***************************************************************************************************/
Mat EqualizeIntensity(const Mat inputImage) {
	if (inputImage.channels() >= 3) {
		Mat ycrcb;

		cvtColor(inputImage, ycrcb, COLOR_BGR2YCrCb);

		vector<Mat> channels;
		split(ycrcb, channels);

		equalizeHist(channels[0], channels[0]);

		Mat result;
		merge(channels, ycrcb);

		cvtColor(ycrcb, result, COLOR_YCrCb2BGR);

		return result;
	}
	return Mat();
}


/***************************************************************************************************
* Ramp Blending
*    Input: left and right image.
*    Output: blended image.
***************************************************************************************************/

void RampBlending2(cv::Mat leftImg, cv::Mat rightImg, cv::Mat &blendedImg) {
	assert(leftImg.size() == rightImg.size());

	int h = leftImg.size().height;
	int w = leftImg.size().width;

	cv::Mat leftImg_, rightImg_;
	leftImg.convertTo(leftImg_, CV_32F);
	rightImg.convertTo(rightImg_, CV_32F);
	//
	cv::Mat bgrBlendedImg[3], bgrLeftImg[3], bgrRightImg[3];   //destination array
	split(leftImg_, bgrLeftImg);                           //split source
	split(rightImg_, bgrRightImg);                           //split source

	bgrBlendedImg[0] = cv::Mat::zeros(bgrLeftImg[1].size(), CV_32F);
	bgrBlendedImg[1] = cv::Mat::zeros(bgrLeftImg[1].size(), CV_32F);
	bgrBlendedImg[2] = cv::Mat::zeros(bgrLeftImg[1].size(), CV_32F);

	double alpha1, alpha2;
#pragma omp parallel for private(alpha1, alpha2)
	for (int r = 0; r < h; r++) {
		for (int c = 0; c < w; c++) {
			alpha1 = double(w - 1 - c) / double(w - 1);
			alpha2 = 1 - alpha1;
			if (bgrLeftImg[0].at<float>(r, c) == 0 && bgrLeftImg[1].at<float>(r, c) == 0 &&
				bgrLeftImg[2].at<float>(r, c) == 0) {
				alpha1 = 0;
				alpha2 = 1;
			}
			if (bgrRightImg[0].at<float>(r, c) == 0 && bgrRightImg[1].at<float>(r, c) == 0 &&
				bgrRightImg[2].at<float>(r, c) == 0) {
				alpha1 = 1;
				alpha2 = 0;
			}
			bgrBlendedImg[0].at<float>(r, c) = alpha1 * bgrLeftImg[0].at<float>(r, c) +
				alpha2 * bgrRightImg[0].at<float>(r, c);
			bgrBlendedImg[1].at<float>(r, c) = alpha1 * bgrLeftImg[1].at<float>(r, c) +
				alpha2 * bgrRightImg[1].at<float>(r, c);
			bgrBlendedImg[2].at<float>(r, c) = alpha1 * bgrLeftImg[2].at<float>(r, c) +
				alpha2 * bgrRightImg[2].at<float>(r, c);
		}
	}
	cv::merge(bgrBlendedImg, 3, blendedImg);
	blendedImg.convertTo(blendedImg, CV_8U);

}




/***************************************************************************************************
* warp pixel wise with blending base translation (delta x and delta y).
*    Input: image and total translation.
*    Output: warpped image and top right corner points.
***************************************************************************************************/
void WarpBaseTranslation(cv::Mat inputImg, float translation[], cv::Mat &stitchedImg,
	cv::Point2f &rightCornerPoint) {
	int w = (int)inputImg.size().width;
	int h = (int)inputImg.size().height;

	int deltaX = (int)translation[0];
	int deltaY = (int)translation[1] + h / 2;


	int wBlend = (int)rightCornerPoint.x - deltaX;
	if (wBlend > w)
		wBlend = w;
	else if (wBlend <= 0)
		wBlend = 0;

	int hBlend = h;

	if (wBlend > 0) {
		cv::Mat leftImg, rightImg, blendedImg;

		// Left image
		stitchedImg(cv::Rect(deltaX, deltaY, wBlend, hBlend)).copyTo(leftImg);

		// Right image
		inputImg(cv::Rect(0, 0, wBlend, hBlend)).copyTo(rightImg);

		// Blend
		RampBlending2(leftImg, rightImg, blendedImg);

		// Warp
		blendedImg.copyTo(stitchedImg(cv::Rect(deltaX, deltaY, wBlend, hBlend)));

		if (w - wBlend >= 1) {
			inputImg(cv::Rect(wBlend, 0, w - wBlend, h)).copyTo(
				stitchedImg(cv::Rect(deltaX + wBlend, deltaY, w - wBlend, h)));
		}
	}
	else {
		// Warp first frame
		inputImg.copyTo(stitchedImg(cv::Rect(deltaX, deltaY, w, h)));
	}
	rightCornerPoint.x = deltaX + w;
	rightCornerPoint.y = deltaY;
}

/***************************************************************************************************
* Compute translation (delta x and y) based on roll
*	 Input:	left and right image corners with image size and translation threshold.
*    Output: translation vetcor (delta x and y).
*		Notes: delta x and y calculated based on average distance between
*		two image corners and translation threshold.
***************************************************************************************************/
void ComputeTranslation(vector<cv::Point2f> pointL, vector<cv::Point2f> pointR, cv::Size imgSize,
	float trlThrsh, float(&translation)[2]) {
	if (pointL.size() > minCorners && pointR.size() > minCorners &&
		pointL.size() == pointR.size()) {

		// Calculate translation based on average distance between first and last image corners
		float avgTrl[2];
		for (size_t i = 0; i < pointL.size(); i++) {
			avgTrl[0] = (avgTrl[0] * i + pointL[i].x - pointR[i].x) / (i + 1);
			avgTrl[1] = (avgTrl[1] * i + pointL[i].y - pointR[i].y) / (i + 1);
		}
		translation[0] = avgTrl[0];
		translation[1] = avgTrl[1];

		cv::Point2f normPoint = cv::Point2f(-imgSize.width / 2, -imgSize.height / 2);

		int counter = 0;
		float roll = 0.0;

		for (int i = 0; i < pointL.size(); i++) {
			for (int j = i + 1; j < pointL.size(); j++) {
				cv::Point2f p1 = pointL[i] + normPoint;;
				cv::Point2f p2 = pointL[j] + normPoint;;
				cv::Point2f q1 = pointR[i] + normPoint;
				cv::Point2f q2 = pointR[j] + normPoint;

				float mp1 = p1.y / p1.x;
				float mp2 = p2.y / p2.x;
				float mp3 = (p1.y - p2.y) / (p1.x - p2.x);
				//float dp3 = -mp3*p1.x + p1.y;

				float mq3 = (q1.y - q2.y) / (q1.x - q2.x);
				float dq3 = -mq3 * q1.x + q1.y;

				float thetaP3Q3 = atan(((mp3 - mq3) / (1 + mp3 * mq3))); // roll

				if (isfinite(thetaP3Q3)) {
					roll += thetaP3Q3;
					counter++;
				}
			}
		}

		if (counter > 0) {
			roll /= counter;

			float cosRoll = cos(roll);
			float sinRoll = sin(roll);

			float deltaX = avgTrl[0] * cosRoll + avgTrl[1] * sinRoll;
			float deltaY = -avgTrl[0] * sinRoll + avgTrl[1] * cosRoll;

			if (abs(deltaX - avgTrl[0]) < trlThrsh && abs(deltaY - avgTrl[1]) < trlThrsh) {
				translation[0] = deltaX;
				translation[1] = deltaY;
			}
		}

	}
	else {
		translation[0] = INFINITY;
		translation[1] = 0;
	}
}

/***************************************************************************************************
* Calculate Optical Flow Pyramid using  Lucas Kanade for corner tracking
*	 Input:	Previous and current images with previous corners.
*    Output: Tracked corners.
***************************************************************************************************/
void CornerTracking(cv::Mat prvGrayImg, cv::Mat curGrayImg, vector<cv::Point2f> &prvCorners,
	vector<cv::Point2f> &trackedCorners) {
	// KLT var.
	vector<uchar> status;
	vector<float> err;
	size_t countAccTracked = 0;

	//trackedCorners.resize(0);

	// Cornaer tracking
	cv::calcOpticalFlowPyrLK(prvGrayImg, curGrayImg, prvCorners, trackedCorners, status,
		err, winSize, 3, termcrit, 0, 0.001);


	for (size_t i = countAccTracked = 0; i < trackedCorners.size(); i++) {
		if (!status[i] || err[i] > KLTerrorThrsh)
			continue;
		trackedCorners[countAccTracked] = trackedCorners[i];
		prvCorners[countAccTracked] = prvCorners[i];
		countAccTracked++;
	}

	trackedCorners.resize(countAccTracked);
	prvCorners.resize(countAccTracked);


#if _ENABLE_DISPLAY_TRACKING
	Mat outputImg, displayPrvImg, displayCurImg;
	prvGrayImg.copyTo(displayPrvImg);
	curGrayImg.copyTo(displayCurImg);
	DrawMatchCorners(displayPrvImg, displayCurImg, prvCorners, trackedCorners, outputImg);
	String winName = "Corner tracking";
	namedWindow(winName, WINDOW_NORMAL);
	cv::resizeWindow(winName, outputImg.size().width / 3, outputImg.size().height / 3);
	imshow(winName, outputImg);
	waitKey(0);
#endif


}

/***************************************************************************************************
* Matching keypoints between two images.
*    Input: Source and destination image with keypoints and descriptor.
*    Output: Source and destination matched corners.
***************************************************************************************************/
void KeyPointMatching(cv::Mat srcImg, cv::Mat dstImg, std::vector<KeyPoint> srcImgKeypoints,
	std::vector<KeyPoint> dstImgKeypoints,
	cv::Mat srcImgDescriptors, cv::Mat dstImgDescriptors,
	vector<Point2f> &srcImgCorners, vector<Point2f> &dstImgCorners) {
	if (srcImgKeypoints.size() < minCorners || dstImgKeypoints.size() < minCorners) {
		srcImgCorners.resize(0);
		dstImgCorners.resize(0);
		return;
	}

	cv::Size imgSize = srcImg.size();
	// Match
	Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create(DescriptorMatcher::BRUTEFORCE);
	std::vector<std::vector<DMatch> > knnMatches;
	matcher->knnMatch(srcImgDescriptors, dstImgDescriptors, knnMatches, 2);


	std::vector<DMatch> matches;

	for (size_t i = 0; i < knnMatches.size(); ++i) {
		for (int j = 0; j < knnMatches[i].size(); j++) {
			matches.push_back(knnMatches[i][j]);
		}
	}

	// Extract corners from keypoints
	srcImgCorners.resize(matches.size());
	dstImgCorners.resize(matches.size());

	for (size_t i = 0; i < matches.size(); i++) {
		srcImgCorners[i] = srcImgKeypoints[matches[i].queryIdx].pt;
		dstImgCorners[i] = dstImgKeypoints[matches[i].trainIdx].pt;
	}

	// Control number of corners for homography
	if (srcImgCorners.size() < 4 || dstImgCorners.size() < 4) {
		srcImgCorners.resize(0);
		dstImgCorners.resize(0);
		return;
	}

	// Compute homography
	cv::Mat maskRansac;
	findHomography(srcImgCorners, dstImgCorners, RANSAC, 3, maskRansac);

	// Filter corners with RANSAC
	vector<cv::Point2f> srcImgRansacCorners, drcImgRansacCorners;
	for (int j = 0; j < maskRansac.size().height; j++) {
		if (maskRansac.at<uchar>(j, 0) == 1) {
			srcImgRansacCorners.push_back(srcImgCorners[j]);
			drcImgRansacCorners.push_back(dstImgCorners[j]);
		}
	}

	// Control number of filterd corners with RANSAC
	if (srcImgRansacCorners.size() < 0 || drcImgRansacCorners.size() < 0) {
		srcImgCorners.resize(0);
		dstImgCorners.resize(0);
		return;
	}


	srcImgCorners = srcImgRansacCorners;
	dstImgCorners = drcImgRansacCorners;


#if _ENABLE_DISPLAY_MATCHING
	// Draw matches
	Mat imgMatches;
	/*drawMatches(srcImg, srcImgKeypoints, dstImg, dstImgKeypoints, goodMatches, imgMatches, Scalar::all(-1),
	Scalar::all(-1), std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);*/
	DrawMatchCorners(srcImg, dstImg, srcImgCorners, dstImgCorners, imgMatches);
	// Show detected matches
	namedWindow("Good Matches", WINDOW_NORMAL);
	imshow("Good Matches", imgMatches);
	cv::waitKey(0);
#endif
	return;

}

/***************************************************************************************************
* fill the holes based on the intensity level of the neighbors.
***************************************************************************************************/
void FillHoles(cv::Mat &inputImg) {
	int w = (int)inputImg.size().width;
	int h = (int)inputImg.size().height;

	// Fill the holes
	for (int i = 0; i < w; i++) {
		for (int j = 7 * h / 8; j < h; j++) {
			if (inputImg.at<cv::Vec3b>(j, i).val[0] == 0 && inputImg.at<cv::Vec3b>(j, i).val[1] == 0
				&& inputImg.at<cv::Vec3b>(j, i).val[2] == 0) {

				inputImg.at<cv::Vec3b>(j, i).val[0] = inputImg.at<cv::Vec3b>(j - 1, i).val[0];
				inputImg.at<cv::Vec3b>(j, i).val[1] = inputImg.at<cv::Vec3b>(j - 1, i).val[1];
				inputImg.at<cv::Vec3b>(j, i).val[2] = inputImg.at<cv::Vec3b>(j - 1, i).val[2];
			}
		}

		for (int j = h / 8; j >= 0; j--) {
			if (inputImg.at<cv::Vec3b>(j, i).val[0] == 0 && inputImg.at<cv::Vec3b>(j, i).val[1] == 0
				&& inputImg.at<cv::Vec3b>(j, i).val[2] == 0) {
				inputImg.at<cv::Vec3b>(j, i).val[0] = inputImg.at<cv::Vec3b>(j + 1, i).val[0];
				inputImg.at<cv::Vec3b>(j, i).val[1] = inputImg.at<cv::Vec3b>(j + 1, i).val[1];
				inputImg.at<cv::Vec3b>(j, i).val[2] = inputImg.at<cv::Vec3b>(j + 1, i).val[2];
			}
		}
	}
}

/***************************************************************************************************
* Sharpen the edges with Gaussian kernel.
***************************************************************************************************/
void SharpenEdges(cv::Mat &inputImg) {
	Mat imgTmp;
	cv::GaussianBlur(inputImg, imgTmp, cv::Size(0, 0), 3);
	cv::addWeighted(inputImg, 1.8, imgTmp, -0.8, 0, inputImg);
}

/***************************************************************************************************
* Cut the upper and lower boundaries in the stitched image. Then sharpens the image and
*	fill the holes based on the average intensity of the neighbors.
*	 Input:	stitched image and right corner point (x coordinate).
*    Output: adjusted image.
***************************************************************************************************/
void Adjustment(cv::Mat inputImg, float xRightCornerPoint, cv::Mat &outputImg) {
	inputImg = inputImg(cv::Rect(0, 0, xRightCornerPoint, inputImg.rows));
	int w = (int)inputImg.size().width;
	int h = (int)inputImg.size().height;

	int topBorder;

	for (int j = 0; j < h; j++) {
		int count = 0;

		for (int i = 0; i < w; i++) {
			if (inputImg.at<cv::Vec3b>(j, i).val[0] != 0 && inputImg.at<cv::Vec3b>(j, i).val[1] != 0
				&& inputImg.at<cv::Vec3b>(j, i).val[2] != 0)
				count++;
		}
		if (count >= 0.5 * w) {
			topBorder = j;
			break;
		}
	}

	int botBorder;
	for (int j = h - 1; j >= 0; j--) {
		int count = 0;

		for (int i = 0; i < w; i++) {
			if (inputImg.at<cv::Vec3b>(j, i).val[0] != 0 && inputImg.at<cv::Vec3b>(j, i).val[1] != 0
				&& inputImg.at<cv::Vec3b>(j, i).val[2] != 0)
				count++;
		}
		if (count >= 0.5 * w) {
			botBorder = j;
			break;
		}
	}

	inputImg = inputImg(cv::Rect(0, topBorder, w, botBorder - topBorder));


	// Fill Holes
	FillHoles(inputImg);

	// Sharp
	SharpenEdges(inputImg);

	inputImg.copyTo(outputImg);
}

/***************************************************************************************************
* Cuts the first and the end of the image and then blends them.
*    Input: adjusted image and warp width.
*    Output: 360 panorama image.
***************************************************************************************************/
void Create360Panorama(cv::Mat inputImg, cv::Mat &outputImg) {
	float w = inputImg.size().width;
	float h = inputImg.size().height;
	int wBlend = warpWidth / 2;

	cv::Mat tmpImg(cv::Size(w - wBlend, h), inputImg.type());
	tmpImg.setTo(cv::Scalar(0));

	cv::Mat leftImg, rightImg, blendedImg;

	// End --> left image - 1--> 0
	inputImg(cv::Rect(w - wBlend, 0, wBlend, h)).copyTo(leftImg);

	// First --> right image - 0 --> 1
	inputImg(cv::Rect(0, 0, wBlend, h)).copyTo(rightImg);

	// Blend
	RampBlending2(leftImg, rightImg,blendedImg);

	blendedImg.copyTo(tmpImg(cv::Rect(0, 0, wBlend, h)));

	inputImg(cv::Rect(wBlend, 0, w - 2 * wBlend, h)).
		copyTo(tmpImg(cv::Rect(wBlend, 0, w - 2 * wBlend, h)));

	tmpImg.copyTo(outputImg);
}

/***************************************************************************************************
* Change output image aspect ratio to 2:1
*    Input: Mat input image.
*    Output: Mat output image.
***************************************************************************************************/
void SetAspectRatio(cv::Mat inputImg, cv::Mat &outputImg) {
	int w = inputImg.size().width;

	if (w % 2 == 1)
		w = w - 1;

	int h = inputImg.size().height;

	cv::Mat tmpImg(cv::Size(w, w / 2), inputImg.type());
	tmpImg.setTo(cv::Scalar::all(158));

	if (w > 2 * h) {
		inputImg(cv::Rect(0, 0, w, h)).copyTo(tmpImg(cv::Rect(0, (w / 2 - h) / 2, w, h)));
	}
	else if (w < 2 * h) {
		inputImg(cv::Rect(0, (h - w / 2) / 2, w, w / 2)).copyTo(tmpImg);
	}
	else {
		inputImg(cv::Rect(0, 0, w, h)).copyTo(tmpImg);
	}
	tmpImg.copyTo(outputImg);
}



/***************************************************************************************************
------------------------------------------------Main------------------------------------------------
***************************************************************************************************/
int main(void) {

	// Start time
	int64 app_start_time = getTickCount();

	// Read video
	cv::VideoCapture video(videoPath);



	// Exit if video is not opened
	if (!video.isOpened()) {
		print("   [Error] Could not read video file");
		return 1;
	}

	

	// Read first frame
	cv::Mat inputFrame;

	video.read(inputFrame);

	// Remove two first frames
	//    video.set(CAP_PROP_POS_FRAMES, 2);

	// Calculate frame size
	cv::Size frameSize = cv::Size(inputFrame.size().width*workScale,
		inputFrame.size().height*workScale);
	if (rotateFrame)
		frameSize = cv::Size(inputFrame.size().height*workScale,
			inputFrame.size().width*workScale);

	// Warp width (gloabal var)
	warpWidth = frameSize.width / 10;


#if _DEBUG_
	print("   [*] Num Frames: " << video.get(CAP_PROP_FRAME_COUNT));
	print("   [*] Input size: " << frameSize);
	print("   [*] FPS: " << video.get(CAP_PROP_FPS));
#endif
	// Set video fps when video.get(CAP_PROP_FPS) not zero


	// Counters
	int fCounter = -1, countWarped = 0, countAccTrl = 0; // Acc == Acceptable




	// Matrices
	cv::Mat prvImg, curImg, prvGrayImg, curGrayImg;
	// Corners
	vector<cv::Point2f> prvCorners, curCorners, trackedCorners;

	// Parameters for compute translation between two frame  - Trl == Translation
	float curTrl[2] = { 0.0, 0.0 }; // {Delta X, Delta Y}
	float prvTrl[2] = { static_cast<float>(warpWidth / 3), 0.0 }; // {Delta X, Delta Y}
	float totalTrl[2] = { 0.0, 0.0 }; // {Delta X, Delta Y}
	float avgTrl[2] = { static_cast<float>(warpWidth / 3), 0.0 }; // {Delta X, Delta Y}

	
	// Last right corner points in warped image (End of stitched image)
	cv::Point2f rightCornerPoint = cv::Point2f(0.0, 0.0);

	// variable for save all image with info. for warp
	vector<cv::Mat> sourceImg; // width = warp width
	vector<float> allDeltaXTrl;
	vector<float> allDeltaYTrl;

	// Sensor (gyroscope) info.
	vector<double> yawSensor;

	ReadSensorData(dataPath, videoFps, yawSensor);


	// Variable for create 360 panorama
	Mat firstImg, firstGrayImg;


	bool arrive360Flag = false;
	bool arrive350Flag = false;

	float firstImgTrl[2] = { 0.0, 0.0 }; // {Delta X, Delta Y}


	// Feature detector and matcher var.
	Ptr<AKAZE> detector = AKAZE::create();
	std::vector<KeyPoint> firstImgKeypoints, lastImgKeypoints;
	cv::Mat firstImgDescriptors, lastImgDescriptors;
	std::vector<cv::Point2f> firstImgCorners, lastImgCorners;

	// Progress

	float progress = 0.0;

	// Set interval
	int maxCountFrames = min(400, int(yawSensor.size()));
	float interval = float(yawSensor.size()) / float(maxCountFrames);
	float intervalCounter = 0;

	// Exposure compensation
	vector<cv::Scalar> allMeanIntensity;
	cv::Scalar globalMeanIntensity = cv::Scalar(0, 0, 0, 0);


	// Main loop
	while (true) {

		// Read frame
		video.read(inputFrame);
		fCounter++;

		// Remove extra frames
		if (fCounter < intervalCounter && !arrive350Flag)
			continue;

		intervalCounter += interval;

		if (inputFrame.empty() || arrive360Flag || fCounter >= yawSensor.size())
			break;


	

		// Remove duplicate frames
		if (fCounter > 0 && fCounter < yawSensor.size() &&
			yawSensor[fCounter] - yawSensor[fCounter - 1] <= 0)
			continue;

		// Set 360 panorama flags
		if (yawSensor[fCounter] >= Deg2Rad(345))
			arrive350Flag = true;
		if (arrive350Flag && yawSensor[fCounter] >= Deg2Rad(360))
			arrive360Flag = true;

		// Rotate Frame
		if (rotateFrame)
			cv::rotate(inputFrame, inputFrame, cv::ROTATE_90_CLOCKWISE);

		// Resize Frame
		cv::resize(inputFrame, inputFrame, frameSize, cv::INTER_LINEAR_EXACT);


		inputFrame.copyTo(curImg);


#if _DEBUG_
		print(">> - Image Num:         " << countWarped);
		print("   - Yaw:               " << yawSensor[fCounter] * 180 / CV_PI);
#endif


		// Convert To gray image
		cv::cvtColor(curImg, curGrayImg, cv::COLOR_RGB2GRAY);

		if (countWarped > 0) {
			// Calculate Optical Flow Pyramid using Lucas Kanade for corner tracking
			CornerTracking(prvGrayImg, curGrayImg, prvCorners, trackedCorners);

			// Calculate translation based on roll
			ComputeTranslation(prvCorners, trackedCorners, frameSize, translationThrsh, curTrl);

			if (!isnan(curTrl[0]) && !isnan(curTrl[1]) && curTrl[0] >= 2 &&
				curTrl[0] < 0.9 * warpWidth) {
				// Calculate average of translations
				avgTrl[0] = (avgTrl[0] * countAccTrl + curTrl[0]) / (countAccTrl + 1);
				avgTrl[1] = (avgTrl[1] * countAccTrl + curTrl[1]) / (countAccTrl + 1);

				prvTrl[0] = curTrl[0];
				prvTrl[1] = curTrl[1];

				countAccTrl++;
			}
			else if (!isnan(curTrl[0]) && !isnan(curTrl[1]) && curTrl[0] <= 0) {
				continue;
			}
			else {
				curTrl[0] = avgTrl[0];
				curTrl[1] = avgTrl[1];
			}

		}
		else {
			// Detect and describe first image features
			detector->detectAndCompute(curGrayImg, noArray(), firstImgKeypoints,
				firstImgDescriptors);
			curImg.copyTo(firstImg);
			curGrayImg.copyTo(firstGrayImg);
		}
#if _DEBUG_
		print("   - Delta X=         " << curTrl[0] << " - Delta Y=         " << curTrl[1]);
		print("   - Average Delta X= " << avgTrl[0] << " - Average Delta Y= " << avgTrl[1]);
#endif
		// Cut images based on warp sidth
		cv::Mat tmpImg;
		curImg(cv::Rect(frameSize.width / 2 - warpWidth / 2, 0, warpWidth,
			frameSize.height)).copyTo(tmpImg);


		// Calc mean intensity for Exposure compensation
		if (colorCorr) {
			cv::Scalar meanIntensity;
			MeanIntensity(curImg, meanIntensity);
			globalMeanIntensity[0] =
				(globalMeanIntensity[0] * countWarped + meanIntensity[0]) / (countWarped + 1);
			globalMeanIntensity[1] =
				(globalMeanIntensity[1] * countWarped + meanIntensity[1]) / (countWarped + 1);
			globalMeanIntensity[2] =
				(globalMeanIntensity[2] * countWarped + meanIntensity[2]) / (countWarped + 1);
			allMeanIntensity.reserve(countWarped + 1);
			allMeanIntensity.push_back(meanIntensity);
		}


		// Data storage ...
		sourceImg.reserve(countWarped + 1);
		allDeltaXTrl.reserve(countWarped + 1);
		allDeltaYTrl.reserve(countWarped + 1);


		sourceImg.push_back(tmpImg);
		allDeltaXTrl.push_back(curTrl[0]);
		allDeltaYTrl.push_back(curTrl[1]);


		// Calculate translation between first frame and last frame to determine the final frame
		if (arrive350Flag) {
			// Detect and describe last image features for
			detector->detectAndCompute(curGrayImg, noArray(), lastImgKeypoints, lastImgDescriptors);

			// Compute translation between first and last images
			KeyPointMatching(firstImg, curImg, firstImgKeypoints, lastImgKeypoints,
				firstImgDescriptors, lastImgDescriptors, firstImgCorners,
				lastImgCorners);

			// Calculate translation based on roll
			ComputeTranslation(lastImgCorners, firstImgCorners, frameSize, translationThrsh * 5,
				firstImgTrl);

			// Detect last frame
			if (firstImgTrl[0] < warpWidth / 2 && firstImgTrl[0] > 0 &&
				abs(firstImgTrl[1]) < frameSize.height / 10)
				arrive360Flag = true;

#if _DEBUG_
			print("   - Last Img -> Delta X: " << firstImgTrl[0] << " -> Delta Y: " << firstImgTrl[1]);
#endif

		}

		// Corner Detector
		if (trackedCorners.size() < maxCorners / 2)
			CornersDetector(curGrayImg, curCorners);
		else
			std::swap(curCorners, trackedCorners);


#if _DEBUG_
		print("   - Count Corners: " << curCorners.size());
#endif
		curImg.copyTo(prvImg);
		curGrayImg.copyTo(prvGrayImg);
		std::swap(curCorners, prvCorners);

		countWarped++;
		// Progress bar
		if (countWarped % (int(yawSensor.size() / 85.0 + 0.5)) == 0 && progress < 85)
		{
			progress += 1;
			print("<-------------------------------------- Progress " << progress << " % -------------------------------------->");

			/*FileStorage fs([tempText3 UTF8String], FileStorage::WRITE);
			fs << "progress" << progress;
			fs.release();*/
		}
	}
	progress = 85;
	video.release();

	if (colorCorr) {
#if _DEBUG_
		print("   ************************************************ Exposure compensation ************************************************");
#endif
		cv::Scalar alpha;
		for (int i = 0; i < sourceImg.size(); i++) {

			if (i > 0)
				ColorCorrectionCoefficients(allMeanIntensity[i - 1], allMeanIntensity[i],
					globalMeanIntensity, weightAlpha, alpha);
			else
				ColorCorrectionCoefficients(allMeanIntensity[sourceImg.size() - 1],
					allMeanIntensity[i], globalMeanIntensity, weightAlpha,
					alpha);


			ApplyColorCorrection(sourceImg[i], alpha);

#if _DEBUG_
			print(">> - Image: " << i);
			print("     Alpha: " << alpha);
#endif
		}
	}




#if _DEBUG_
	print("   ************************************************ Warp ************************************************");
#endif
	// Change delta Y average
	avgTrl[1] = (avgTrl[1] * (sourceImg.size() - 1) + firstImgTrl[1]) / (sourceImg.size());




	// Warp Matrix
	cv::Mat stitchedImg(cv::Size(frameSize.width * xScale, frameSize.height * yScale), CV_8UC3);
	stitchedImg.setTo(cv::Scalar::all(0));

	for (int i = 0; i < sourceImg.size(); i++) {
		// Calc. total trl
		totalTrl[0] = totalTrl[0] + allDeltaXTrl[i];
		totalTrl[1] = totalTrl[1] + allDeltaYTrl[i] - avgTrl[1];


		// Warp with total translation
		WarpBaseTranslation(sourceImg[i], totalTrl, stitchedImg, rightCornerPoint);


#if _DEBUG_
		print(">> - Image: " << i);
		print("   - Total Delta X= " << totalTrl[0] << " Total Delta Y= " << totalTrl[1]);
#endif

#if _ENABLE_DISPLAY_WARPING
		String winName = "Stitched image";
		namedWindow(winName, WINDOW_NORMAL);
		cv::resizeWindow(winName, stitchedImg.size().width / 3, stitchedImg.size().height / 3);
		imshow(winName, stitchedImg);
		waitKey(0);
#endif
		// Progress bar
		if (i % (int(sourceImg.size() / 15.0) + 1) == 0 && i > 0 && progress < 100)
		{
			progress += 1;
			print("<-------------------------------------- Progress " << progress << " % -------------------------------------->");
			/*FileStorage fs([tempText3 UTF8String], FileStorage::WRITE);
			fs << "progress" << progress;
			fs.release();*/
		}

	}




	// Adjustment
	Adjustment(stitchedImg, rightCornerPoint.x, stitchedImg);

	// create 360 panorama
	Create360Panorama(stitchedImg, stitchedImg);

	cv::resize(stitchedImg, stitchedImg, cv::Size(stitchedImgWidth, (float(stitchedImg.rows) /
		float(stitchedImg.cols)) *
		stitchedImgWidth));


	// Histogram equalization
	if (histEqu)
		stitchedImg = EqualizeIntensity(stitchedImg);

	// Change output image aspect ratio to 2:1
	SetAspectRatio(stitchedImg, stitchedImg);



#if _DEBUG_
	print("   [*] ALL time: " << ((getTickCount() - app_start_time) / getTickFrequency()) << " sec");
	print("   [*] Count Acceptable Match: " << countAccTrl);
	print("   [*] Avg Delta X: " << avgTrl[0] << " Avg Delta Y: " << avgTrl[1]);
#endif


	// OutputFile
	cv::imwrite("res.jpg", stitchedImg);
	return 0;
}