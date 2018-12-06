#ifndef _SLAM_PUBLISH_PACKAGE_h
#define _SLAM_PUBLISH_PACKAGE_h

#include "Platform.h"

enum SlamOverallStatus : uint8_t
{
	SLAM_START = 1,
	SLAM_STOP = 2,
	SLAM_RESTART = 3,
};

enum SlamOperationStatus : uint8_t
{
	SLAM_RUNNING = 1,
	SLAM_STOPPED = 2,
	SLAM_INITIALIZING = 3,
	SLAM_WAITING_FOR_MOTION = 4,
};

enum SlamPublishType : uint8_t
{
	SLAM_PUBLISH_FRAME = 1,
	SLAM_PUBLISH_KEY_FRAME = 2,
	SLAM_RESET = 3,
};

PACK(SlamStatusUpdate{
	SlamOperationStatus operationStatus;
	});

PACK(SlamPublishFrame{
	uint32_t id;
	double timestamp;
	double tx;
	double ty;
	double tz;
	double q1;
	double q2;
	double q3;
	double q4;
	double s;
	});

PACK(SlamPublishKeyFrame{
	uint32_t id;
	int points;
	double fx;
	double fy;
	double cx;
	double cy;
	});

PACK(SlamPublishPoint{
	float u;
	float v;
	float inverseDepth;
	uint8_t color[8];
	});

class SlamPublishPackage
{
public:
	~SlamPublishPackage();
	uint8_t* getData(uint32_t* pSize);
	SlamPublishFrame frame;
	SlamPublishType publishType;
	SlamPublishKeyFrame* pKeyFrame = NULL;
	std::vector<SlamPublishPoint*> points;
};

#endif