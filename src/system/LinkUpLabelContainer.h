#ifndef _LINKUP_LABEL_CONTAINER_h
#define _LINKUP_LABEL_CONTAINER_h

#include "Platform.h"

struct LinkUpLabelContainer
{
	LinkUpEventLabel* pCameraEvent;
	LinkUpEventLabel* pImuEvent;
	LinkUpEventLabel* pCameraImuEvent;

	LinkUpPropertyLabel_Int16* pExposureLabel;
	LinkUpPropertyLabel_Boolean* pRecodRemoteLabel;

	LinkUpFunctionLabel* pReceiveReplayDataLabel;
	LinkUpFunctionLabel* pGetChessboardCornerLabel;
	LinkUpFunctionLabel* pUpdateSettingsLabel;

	LinkUpPropertyLabel_Double* pAccelerometerScaleLabel;
	LinkUpPropertyLabel_Double* pGyroscopeScaleLabel;
	LinkUpPropertyLabel_Double* pTemperatureScaleLabel;
	LinkUpPropertyLabel_Double* pTemperatureOffsetLabel;
};

#endif