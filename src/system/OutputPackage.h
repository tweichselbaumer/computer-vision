#ifndef _OUTPUT_PACKAGE_h
#define _OUTPUT_PACKAGE_h

#include "FramePackage.h"
#include "SlamPublishPackage.h"

struct ImuData
{
	uint64_t timestamp;
	double gx;
	double gy;
	double gz;
	double ax;
	double ay;
	double az;
	double temperature;
};

struct ImuDataDerived
{
	uint64_t timestamp;
	double gx;
	double gy;
	double gz;
	double ax;
	double ay;
	double az;
};

struct OutputPackage
{
	FramePackage* pFramePackage;
	ImuData imuData;
	ImuDataDerived imuDataDerived;
};

#endif