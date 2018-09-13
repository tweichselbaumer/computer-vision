#include "uEyeCamera.h"

uint8_t uEyeCamera::open()
{
	INT nRet;

	nRet = is_InitCamera(&hCam, NULL);
	if (nRet != IS_SUCCESS)
	{
		return -1;
	}

	nRet = is_SetColorMode(hCam, IS_CM_MONO8);
	if (nRet != IS_SUCCESS)
	{
		return -1;
	}

	nRet = is_SetBinning(hCam, IS_BINNING_2X_VERTICAL | IS_BINNING_2X_HORIZONTAL);
	if (nRet != IS_SUCCESS)
	{
		return -1;
	}

	IS_RECT rectAOI;
	rectAOI.s32X = 72;
	rectAOI.s32Y = 0;
	rectAOI.s32Width = getWidth();
	rectAOI.s32Height = getHeight();

	nRet = is_AOI(hCam, IS_AOI_IMAGE_SET_AOI, (void*)&rectAOI, sizeof(rectAOI));
	if (nRet != IS_SUCCESS)
	{
		return -1;
	}

	nRet = is_AllocImageMem(hCam, getWidth(), getHeight(), 8, &pImgMem, &id);
	if (nRet != IS_SUCCESS)
	{
		return -1;
	}

	nRet = is_SetImageMem(hCam, pImgMem, id);
	if (nRet != IS_SUCCESS)
	{
		return -1;
	}

	UINT nRange[3];

	ZeroMemory(nRange, sizeof(nRange));

	nRet = is_PixelClock(hCam, IS_PIXELCLOCK_CMD_GET_RANGE, (void*)nRange, sizeof(nRange));

	nRet = is_PixelClock(hCam, IS_PIXELCLOCK_CMD_SET,
		(void*)&nRange[1], sizeof(nRange[1]));

	nRet = is_SetDisplayMode(hCam, IS_SET_DM_DIB);
	if (nRet != IS_SUCCESS)
	{
		return -1;
	}

	setExposure(0);

#ifdef EXTERN_CAMERA_TRIGGER
	is_SetExternalTrigger(hCam, IS_SET_TRIGGER_LO_HI);
#else
	double FPS, NEWFPS;
	FPS = 20;
	is_SetFrameRate(hCam, FPS, &NEWFPS);
#endif //EXTERN_CAMERA_TRIGGER

	nRet = is_Exposure(hCam, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MIN, &minExposure, 8);
	if (nRet != IS_SUCCESS)
	{
		return -1;
	}

	nRet = is_Exposure(hCam, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_MAX, &maxExposure, 8);
	if (nRet != IS_SUCCESS)
	{
		return -1;
	}

	nRet = is_Exposure(hCam, IS_EXPOSURE_CMD_GET_EXPOSURE_RANGE_INC, &incExposure, 8);
	if (nRet != IS_SUCCESS)
	{
		return -1;
	}

	return 0;
}


void uEyeCamera::setExposure(int16_t exposure)
{
	double enable = 1;
	double disable = 0;

	if (exposure != lastExposureSetting) {
		if (exposure == -1)
		{
			is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_GAIN, &enable, 0);
			is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_WHITEBALANCE, &enable, 0);
			is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_FRAMERATE, &disable, 0);
			is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SHUTTER, &enable, 0);
			is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_GAIN, &enable, 0);
			is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_WHITEBALANCE, &enable, 0);
			is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_SHUTTER, &enable, 0);
		}
		else
		{
			is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_GAIN_SHUTTER, &disable, 0);
			double newExposure = incExposure * (exposure + 1.5);

			if (newExposure < minExposure)
				newExposure = minExposure;
			else if (newExposure > maxExposure)
				newExposure = maxExposure;

			is_Exposure(hCam, IS_EXPOSURE_CMD_SET_EXPOSURE, &newExposure, 8);
		}
		lastExposureSetting = exposure;
	}
}


uint8_t uEyeCamera::close()
{
	INT nRet;
	nRet = is_FreeImageMem(hCam, pImgMem, id);
	if (nRet != IS_SUCCESS)
	{
		return -1;
	}
	nRet = is_ExitCamera(hCam);
	if (nRet != IS_SUCCESS)
	{
		return -1;
	}
	return 0;
}

uint8_t uEyeCamera::capture(uint8_t* pData, int16_t exposureSetting, double* pNewExposure)
{
	INT nRet;
	setExposure(exposureSetting);
	is_Exposure(hCam, IS_EXPOSURE_CMD_GET_EXPOSURE, pNewExposure, 8);
	nRet = is_FreezeVideo(hCam, IS_WAIT);
	//nRet = is_CaptureVideo(hCam, IS_WAIT);
	if (nRet != IS_SUCCESS)
	{
		return -1;
	}

	void* pMem;

	nRet = is_GetImageMem(hCam, &pMem);
	if (nRet != IS_SUCCESS)
	{
		return -1;
	}

	memcpy(pData, pMem, getHeight() * getWidth() * sizeof(uint8_t));
	return 0;
}

uint16_t uEyeCamera::getHeight()
{
	return 512;
}

uint16_t uEyeCamera::getWidth()
{
	return 512;
}