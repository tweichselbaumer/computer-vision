#ifndef _LINKUP_LABEL_CONTAINER_h
#define _LINKUP_LABEL_CONTAINER_h

#include "Platform.h"

struct LinkUpLabelContainer
{
	LinkUpEventLabel* pCameraEvent;
	LinkUpEventLabel* pImuEvent;
	LinkUpEventLabel* pCameraImuEvent;

	LinkUpPropertyLabel_Int16* pExposureLabel;

	LinkUpFunctionLabel* pReceiveReplayDataLabel;
	LinkUpFunctionLabel* pGetChessboardCornerLabel;
};

#endif