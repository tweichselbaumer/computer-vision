#include "OutputModule.h"

OutputModule::OutputModule(InputModule* pInputModule, LinkUpLabelContainer* pLinkUpLabelContainer)
{
	pFreeQueue_ = new boost::lockfree::queue<OutputPackage*>(queueSize);
	pInQueue_ = new boost::lockfree::queue<OutputPackage*>(queueSize);
	pInputModule_ = pInputModule;
	pLinkUpLabelContainer_ = pLinkUpLabelContainer;
}

void OutputModule::start()
{
	for (int i = 0; i < queueSize; i++)
	{
		OutputPackage* pOutputPackage = (OutputPackage*)calloc(1, sizeof(OutputPackage));
		pFreeQueue_->push(pOutputPackage);
	}

	bIsRunning_ = true;

	thread_ = boost::thread(boost::bind(&OutputModule::doWork, this));
}

void OutputModule::stop()
{
	bIsRunning_ = false;
	thread_.join();
}

void OutputModule::writeOut(OutputPackage* pResult)
{
	pInQueue_->push(pResult);
}

OutputPackage* OutputModule::nextFreeOutputPackage()
{
	OutputPackage* pOutputPackage;
	while (pFreeQueue_->empty())
	{
		boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
	}
	pFreeQueue_->pop(pOutputPackage);
	return pOutputPackage;
}

void OutputModule::doWork()
{
	while (bIsRunning_)
	{
		std::vector<int> compression_params;

		OutputPackage* pOutputPackage;
		while (pInQueue_->empty())
		{
			boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
		}
		pInQueue_->pop(pOutputPackage);

		if (pOutputPackage->pFramePackage->imu.cam)
		{
			std::vector<uchar> buf;
			if (pLinkUpLabelContainer_->pCameraEvent->isSubscribed)
			{
				imencode(".bmp", pOutputPackage->pFramePackage->image, buf, compression_params);
				uint8_t pTemp[buf.size() + sizeof(double)];
				memcpy(pTemp, &pOutputPackage->pFramePackage->exposureTime, sizeof(double));
				memcpy(pTemp + sizeof(double), (uint8_t*)&buf[0], buf.size());
				pLinkUpLabelContainer_->pCameraEvent->fireEvent(pTemp, buf.size() + sizeof(double));
			}
		}
		if (pLinkUpLabelContainer_->pImuEvent->isSubscribed)
		{
			pLinkUpLabelContainer_->pImuEvent->fireEvent((uint8_t*)&(pOutputPackage->pFramePackage->imu), sizeof(ImuData));
		}
		if (pLinkUpLabelContainer_->pCameraImuEvent->isSubscribed)
		{
			if (pOutputPackage->pFramePackage->imu.cam)
			{
				std::vector<uchar> buf;

				imencode(".bmp", pOutputPackage->pFramePackage->image, buf, compression_params);
				uint8_t pTemp[buf.size() + sizeof(ImuData) + sizeof(double)];
				memcpy(pTemp + sizeof(ImuData), &pOutputPackage->pFramePackage->exposureTime, sizeof(double));
				memcpy(pTemp + sizeof(ImuData) + sizeof(double), (uint8_t*)&buf[0], buf.size());
				*(ImuData*)pTemp = pOutputPackage->pFramePackage->imu;
				pLinkUpLabelContainer_->pCameraImuEvent->fireEvent(pTemp, buf.size() + sizeof(ImuData) + sizeof(double));
			}
			else
			{
				pLinkUpLabelContainer_->pCameraImuEvent->fireEvent((uint8_t*)&(pOutputPackage->pFramePackage->imu), sizeof(ImuData));
			}
		}
		pInputModule_->release(pOutputPackage->pFramePackage);
		pFreeQueue_->push(pOutputPackage);
	}
}