#ifndef _FRAME_PACKAGE_h
#define _FRAME_PACKAGE_h

#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <opencv2/opencv_modules.hpp>
#include "Platform.h"

PACK(ImuData{
	uint32_t timestamp_ms;
	uint32_t timestamp_us;
	int16_t gx;
	int16_t gy;
	int16_t gz;
	int16_t ax;
	int16_t ay;
	int16_t az;
	int16_t temperature;
	bool cam;
};)

struct FramePackage
{
	cv::Mat image;
	ImuData imu;
};

#endif