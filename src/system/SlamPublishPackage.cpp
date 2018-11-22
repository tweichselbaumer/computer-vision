#include "SlamPublishPackage.h"

SlamPublishPackage::~SlamPublishPackage()
{
	if (pKeyFrame) {
		for (auto p : points) {
			free(p);
		}
		points.clear();
		free(pKeyFrame);
	}
}

uint8_t* SlamPublishPackage::getData(uint32_t* pSize)
{
	if (pKeyFrame == NULL) 
	{
		*pSize = sizeof(SlamPublishFrame) + sizeof(SlamPublishType);
		uint8_t* pData = (uint8_t*)calloc(*pSize, sizeof(uint8_t));
		pData[0] = SlamPublishType::FRAME;
		memcpy(pData + 1, &frame, sizeof(SlamPublishFrame));
		return pData;
	}
	else
	{
		*pSize = sizeof(SlamPublishFrame) + sizeof(SlamPublishType) + sizeof(SlamPublishPoint)* points.size() + sizeof(SlamPublishKeyFrame);
		uint8_t* pData = (uint8_t*)calloc(*pSize, sizeof(uint8_t));
		pData[0] = SlamPublishType::KEYFRAME_WITH_POINTS;
		memcpy(pData + 1, &frame, sizeof(SlamPublishFrame));
		pKeyFrame->points = points.size();
		memcpy(pData + 1 + sizeof(SlamPublishFrame), pKeyFrame, sizeof(SlamPublishKeyFrame));
		int i = 0;
		for (auto p : points) 
		{
			memcpy(pData + 1 + sizeof(SlamPublishFrame) + sizeof(SlamPublishKeyFrame) + i * sizeof(SlamPublishPoint), pKeyFrame, sizeof(SlamPublishKeyFrame));
		}
		return pData;
	}
}