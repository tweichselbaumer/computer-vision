#ifndef _OUTPUT_PACKAGE_h
#define _OUTPUT_PACKAGE_h

#include "FramePackage.h"

struct ImuData
{
	double timestamp;
	double gx;
	double gy;
	double gz;
	double ax;
	double ay;
	double az;
	double temperature;
};

struct OutputPackage
{
	FramePackage* pFramePackage;
	ImuData imuData;
};

#endif