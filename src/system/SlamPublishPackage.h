#ifndef _SLAM_PUBLISH_PACKAGE_h
#define _SLAM_PUBLISH_PACKAGE_h

#include "Platform.h"

enum SlamPublishType : uint8_t
{
	FRAME = 1,
	KEYFRAME_WITH_POINTS = 2,
	RESET = 3,
};

PACK(SlamPublishFrame{
	uint32_t id;
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
		uint32_t id;
		int points;
		double fx;
		double fy;
		double cx;
		double cy;
		};)

		PACK(SlamPublishPoint{
			float u;
			float v;
			float inverseDepth;
			uint8_t color[8];
			};)

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