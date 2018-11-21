#ifndef _SLAM_PUBLISH_PACKAGE_h
#define _SLAM_PUBLISH_PACKAGE_h

#include "Platform.h"

enum SlamPublishType : uint8_t
{
	FRAME_ONLY = 1
};

PACK(SlamPublishFrame{
	unsigned long id;
	double tx;
	double ty;
	double tz;
	double q1;
	double q2;
	double q3;
	double q4;
	double s;
	};)

PACK(SlamPublishKeyFrame{
	unsigned long id;
	};)

class SlamPublishPackage
{
public:
	~SlamPublishPackage();
	uint8_t* getData(uint32_t* pSize);
	SlamPublishFrame frame;
};

#endif