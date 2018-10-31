#ifndef _CAMERA_H
#define _CAMERA_H

#include <stdint.h>

class Camera
{
public:
	virtual uint8_t open() = 0;
	virtual uint8_t close() = 0;
	virtual uint8_t capture(uint8_t* pData, int16_t exposureSetting, double* pNewExposure, bool wait, uint32_t* pMissed) = 0;
	virtual uint16_t getHeight() = 0;
	virtual uint16_t getWidth() = 0;
};
#endif