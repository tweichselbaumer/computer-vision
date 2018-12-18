#ifndef _LINKUP_LABEL_CONTAINER_h
#define _LINKUP_LABEL_CONTAINER_h

#include "Platform.h"

struct LinkUpLabelContainer
{
	LinkUpEventLabel* pCameraEvent;
	LinkUpEventLabel* pImuEvent;
	LinkUpEventLabel* pCameraImuEvent;

	LinkUpEventLabel* pImuDerivedEvent;

	LinkUpEventLabel* pSlamMapEvent;

	LinkUpPropertyLabel_Int16* pExposureLabel;
	LinkUpPropertyLabel_Boolean* pRecodRemoteLabel;

	LinkUpFunctionLabel* pReceiveReplayDataLabel;
	LinkUpFunctionLabel* pGetChessboardCornerLabel;
	LinkUpFunctionLabel* pUpdateSettingsLabel;

	LinkUpPropertyLabel_Double* pAccelerometerScaleLabel;
	LinkUpPropertyLabel_Double* pGyroscopeScaleLabel;
	LinkUpPropertyLabel_Double* pTemperatureScaleLabel;
	LinkUpPropertyLabel_Double* pTemperatureOffsetLabel;

	LinkUpPropertyLabel_Binary* pImuFilterALabel;
	LinkUpPropertyLabel_Binary* pImuFilterBLabel;

	LinkUpPropertyLabel_Binary* pImuCamCalibrationTCILabel;
	LinkUpPropertyLabel_Binary* pImuCalibrationRAGLabel;
	LinkUpPropertyLabel_Binary* pImuCalibrationMinvALabel;
	LinkUpPropertyLabel_Binary* pImuCalibrationMinvGLabel;
	LinkUpPropertyLabel_Double* pAccelerometerNoiseDensityLabel;
	LinkUpPropertyLabel_Double* pAccelerometerRandomWalkLabel;
	LinkUpPropertyLabel_Double* pGyroscopeRandomWalkLabel;
	LinkUpPropertyLabel_Double* pGyroscopeNoiseDensityLabel;

	LinkUpFunctionLabel* pSlamChangeStatusLabel;
	LinkUpEventLabel* pSlamStatusEvent;
	LinkUpPropertyLabel_Boolean* pSlamReproducibleExecutionLabel;

};

#endif