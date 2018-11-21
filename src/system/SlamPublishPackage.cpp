#include "SlamPublishPackage.h"

SlamPublishPackage::~SlamPublishPackage() 
{

}

uint8_t* SlamPublishPackage::getData(uint32_t* pSize)
{
	*pSize = sizeof(SlamPublishFrame) + sizeof(SlamPublishType);
	uint8_t* pData = (uint8_t*)calloc(*pSize, sizeof(uint8_t));
	pData[0] = SlamPublishType::FRAME_ONLY;
	memcpy(pData + 1, &frame, sizeof(SlamPublishFrame));
	return pData;
}