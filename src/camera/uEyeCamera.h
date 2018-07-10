#ifndef _UEYE_CAMERA_H
#define _UEYE_CAMERA_H

#include <uEye.h>
#include <stdint.h>
#include <cstring>
#include "Camera.h"

class uEyeCamera :public Camera
{
public:
	uint8_t open();
	uint8_t close();
	uint16_t getHeight();
	uint16_t getWidth();
	uint8_t capture(uint8_t* pData);
private:
	HIDS hCam = 0;
	INT id;
	char* pImgMem;
};
#endif