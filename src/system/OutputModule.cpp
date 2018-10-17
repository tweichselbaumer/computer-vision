#include "OutputModule.h"

OutputModule::OutputModule(InputModule* pInputModule, LinkUpLabelContainer* pLinkUpLabelContainer)
{
	pFreeQueue_ = new boost::lockfree::queue<OutputPackage*>(queueSize);
	pInQueue_ = new boost::lockfree::queue<OutputPackage*>(queueSize);
	pInputModule_ = pInputModule;
	pLinkUpLabelContainer_ = pLinkUpLabelContainer;
}

void OutputModule::startRecording()
{
	time_t now;
	time(&now);

	string filename = str((boost::format("/home/up/data/imu_%d.csv") % (int)now));

	pCSVWritter_ = new CSVWritter({ "time", "gx", "gy", "gz", "ax", "ay", "az", "temperature" }, filename);
	pCSVWritter_->open();
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
	pCSVWritter_->close();
}

void OutputModule::writeOut(OutputPackage* pResult)
{
	pInQueue_->push(pResult);
}

OutputPackage* OutputModule::nextFreeOutputPackage()
{
	OutputPackage* pOutputPackage;
	while (!pFreeQueue_->pop(pOutputPackage))
	{
		boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
	}
	return pOutputPackage;
}

void OutputModule::doWork()
{
	while (bIsRunning_)
	{
		std::vector<int> compression_params;

		OutputPackage* pOutputPackage;
		while (!pInQueue_->pop(pOutputPackage))
		{
			boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
		}

		if (pOutputPackage->pFramePackage->imu.cam)
		{
			std::vector<uchar> buf;
			if (pLinkUpLabelContainer_->pCameraEvent->isSubscribed)
			{
				imencode(".bmp", pOutputPackage->pFramePackage->image, buf, compression_params);
				uint8_t* pTemp = (uint8_t*)calloc(buf.size() + sizeof(double), sizeof(uint8_t));
				memcpy(pTemp, &pOutputPackage->pFramePackage->exposureTime, sizeof(double));
				memcpy(pTemp + sizeof(double), (uint8_t*)&buf[0], buf.size());
				pLinkUpLabelContainer_->pCameraEvent->fireEvent(pTemp, buf.size() + sizeof(double));
				free(pTemp);
			}
		}
		if (pLinkUpLabelContainer_->pImuEvent->isSubscribed)
		{
			pLinkUpLabelContainer_->pImuEvent->fireEvent((uint8_t*)&(pOutputPackage->pFramePackage->imu), sizeof(RawImuData));
		}
		if (pLinkUpLabelContainer_->pCameraImuEvent->isSubscribed)
		{
			if (pOutputPackage->pFramePackage->imu.cam)
			{
				std::vector<uchar> buf;

				imencode(".bmp", pOutputPackage->pFramePackage->image, buf, compression_params);
				uint8_t* pTemp = (uint8_t*)calloc(buf.size() + sizeof(RawImuData) + sizeof(double), sizeof(uint8_t));
				memcpy(pTemp + sizeof(RawImuData), &pOutputPackage->pFramePackage->exposureTime, sizeof(double));
				memcpy(pTemp + sizeof(RawImuData) + sizeof(double), (uint8_t*)&buf[0], buf.size());
				*((RawImuData*)pTemp) = pOutputPackage->pFramePackage->imu;
				pLinkUpLabelContainer_->pCameraImuEvent->fireEvent(pTemp, buf.size() + sizeof(RawImuData) + sizeof(double));
				free(pTemp);
			}
			else
			{
				pLinkUpLabelContainer_->pCameraImuEvent->fireEvent((uint8_t*)&(pOutputPackage->pFramePackage->imu), sizeof(RawImuData));
			}
		}

		bool temp = pLinkUpLabelContainer_->pRecodRemoteLabel->getValue();

		if (temp && !bIsRecording_)
		{
			bIsRecording_ = true;
			startRecording();
		}
		else if (!temp && bIsRecording_)
		{
			pCSVWritter_->close();
			bIsRecording_ = false;
		}

		if (bIsRecording_)
			pCSVWritter_->writeValues(pOutputPackage->imuData.timestamp, (double*)&(pOutputPackage->imuData) + 1, 7);

		pInputModule_->release(pOutputPackage->pFramePackage);
		pFreeQueue_->push(pOutputPackage);
	}
}