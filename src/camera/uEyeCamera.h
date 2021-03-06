#ifdef WITH_CAMERA
#ifndef _UEYE_CAMERA_H
#define _UEYE_CAMERA_H

#include <uEye.h>
#include <stdint.h>
#include <cstring>
#include <iostream>
#include "Camera.h"
#include <glog/logging.h>

class uEyeCamera :public Camera
{
public:
	uint8_t open();
	uint8_t close();
	uint16_t getHeight();
	uint16_t getWidth();
	uint8_t capture(uint8_t* pData, int16_t exposureSetting, double* pNewExposure, bool wait, uint32_t* pMissed);
protected:
	HIDS hCam = 0;
	INT id;
	char* pImgMem;
	double minExposure;
	double maxExposure;
	double incExposure;
	int16_t lastExposureSetting = 1;
	void setExposure(int16_t exposure);
};
#endif
#endif //WITH_CAMERA