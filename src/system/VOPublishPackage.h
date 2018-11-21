#ifndef _VO_PUBLISH_PACKAGE_h
#define _VO_PUBLISH_PACKAGE_h

#include "Platform.h"

PACK(PublishFrame{
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

	struct PublishPackage
	{
		bool hasFrame;
		PublishFrame* pFrame;
	};

#endif