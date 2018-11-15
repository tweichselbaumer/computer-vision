#include "InputModule.h"

InputModule::InputModule(boost::asio::io_service& io_service, LinkUpLabelContainer* pLinkUpLabelContainer)
{
	pFreeQueue_ = new boost::lockfree::queue<FramePackage*>(queueSize);
	pOutQueue_ = new boost::lockfree::queue<FramePackage*>(queueSize);
	pCameraQueue_ = new boost::lockfree::queue<FramePackage*>(queueSize);
	pReplayQueue_ = new boost::lockfree::queue<uint8_t*>(queueSize);
	pPort_ = new boost::asio::serial_port(io_service);
	pLinkUpLabelContainer_ = pLinkUpLabelContainer;
}

uint8_t * InputModule::onReplayData(uint8_t* pDataIn, uint32_t nSizeIn, uint32_t* pSizeOut)
{
	*pSizeOut = 0;
	uint8_t* pCopy = (uint8_t*)calloc(nSizeIn + sizeof(uint32_t), sizeof(uint8_t));
	memcpy(pCopy + sizeof(uint32_t), pDataIn, nSizeIn);
	*((uint32_t*)pCopy) = nSizeIn;
	pReplayQueue_->push(pCopy);
	return NULL;
}

void InputModule::doWorkReplay()
{
	while (bIsRunning_)
	{
		uint8_t* pDataIn;

		while (!pReplayQueue_->pop(pDataIn))
		{
			boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
		}

		uint32_t nSizeIn = *((uint32_t*)pDataIn);

		liveTimeout_ = liveTimeout;

		FramePackage* pFramePackage = NULL;

		while (!pFreeQueue_->pop(pFramePackage))
		{
			boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
		}

		pFramePackage->imu = *((RawImuData*)(pDataIn + sizeof(uint32_t)));

		if (pFramePackage->imu.cam)
		{
			memcpy(&pFramePackage->exposureTime, pDataIn + sizeof(RawImuData) + sizeof(uint32_t), sizeof(double));
			cv::Mat matImg;
			matImg = cv::imdecode(cv::Mat(1, nSizeIn - (sizeof(RawImuData) + sizeof(double)), CV_8UC1, pDataIn + sizeof(RawImuData) + sizeof(double) + sizeof(uint32_t)), CV_LOAD_IMAGE_UNCHANGED);
			memcpy(pFramePackage->image.data, matImg.data, matImg.rows*matImg.cols);
			/*imshow("replay", matImg);
			cv::waitKey(1);*/
		}

		free(pDataIn);

		pOutQueue_->push(pFramePackage);
	}
}

void InputModule::start()
{
	bIsRunning_ = true;
	for (int i = 0; i < queueSize; i++)
	{
		FramePackage* pFramePackage = (FramePackage*)calloc(1, sizeof(FramePackage));
		pFramePackage->image.create(pCamera_->getWidth(), pCamera_->getHeight(), CV_8UC1);
		pFreeQueue_->push(pFramePackage);
	}

	replayThread_ = boost::thread(boost::bind(&InputModule::doWorkReplay, this));

#ifdef EXTERN_CAMERA_TRIGGER
	pPort_->open("/dev/ttyS4");
	pPort_->set_option(boost::asio::serial_port_base::baud_rate(115200));
	thread_ = boost::thread(boost::bind(&InputModule::doWork, this));
#endif //EXTERN_CAMERA_TRIGGER


#ifdef WITH_CAMERA
	pCamera_->open();

#ifdef EXTERN_CAMERA_TRIGGER
	//doCamInitSequence();
#endif //EXTERN_CAMERA_TRIGGER

	cameraThread_ = boost::thread(boost::bind(&InputModule::doWorkCamera, this));
#endif //WITH_CAMERA

#ifdef EXTERN_CAMERA_TRIGGER
	threadPing_ = boost::thread(boost::bind(&InputModule::doWorkPing, this));
#endif //EXTERN_CAMERA_TRIGGER


}

void InputModule::doCamInitSequence()
{
	uint32_t nMissed;
	cv::Mat pTemp(pCamera_->getWidth(), pCamera_->getHeight(), CV_8UC1);
	double exposure;
	for (int i = 0; i < 20; i++)
	{
		pCamera_->capture(pTemp.data, pLinkUpLabelContainer_->pExposureLabel->getValue(), &exposure, false, &nMissed);
		boost::this_thread::sleep_for(boost::chrono::milliseconds(50));
	}
}

void InputModule::doWorkPing()
{
	uint8_t pTemp[64];

	while (bIsRunning_)
	{
		LinkUpPacket packet;
		packet.nLength = 1;
		packet.pData = (uint8_t*)calloc(1, sizeof(uint8_t));
		packet.pData[0] = 1;
		raw_.send(packet);
		uint16_t nBytesToWrite = raw_.getRaw(pTemp, 64);
		boost::asio::write(*pPort_, boost::asio::buffer(pTemp, nBytesToWrite));
		boost::this_thread::sleep_for(boost::chrono::milliseconds(400));
	}
}

void InputModule::release(FramePackage* pframePackage)
{
	pFreeQueue_->push(pframePackage);
}

void InputModule::stop()
{
	bIsRunning_ = false;
	thread_.join();
	pPort_->close();
	pCamera_->close();
}


FramePackage* InputModule::next()
{
	FramePackage* pFramePackage;
	while (!pOutQueue_->pop(pFramePackage))
	{
		boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
	}
	return pFramePackage;
}

void InputModule::doWorkCamera()
{
	uint32_t nMissedFrames = 0;
	while (bIsRunning_)
	{
		FramePackage* pFramePackage;
		while (!pFreeQueue_->pop(pFramePackage))
		{
			//boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
		}
		if (pCamera_->capture(pFramePackage->image.data, pLinkUpLabelContainer_->pExposureLabel->getValue(), &(pFramePackage->exposureTime), true, &nMissedFrames) == 0)
		{
			if (liveTimeout_ > 0)
			{
				pFreeQueue_->push(pFramePackage);
				liveTimeout_--;
			}
			else
			{
#ifdef EXTERN_CAMERA_TRIGGER
				pCameraQueue_->push(pFramePackage);
				pFramePackage->missedTrigger = false;
				if (nMissedFrames > 0)
				{
					for (uint32_t i = 0; i < nMissedFrames; i++)
					{
						FramePackage* pFramePackageMissed;
						while (!pFreeQueue_->pop(pFramePackageMissed))
						{
							//boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
						}
						pFramePackageMissed->missedTrigger = true;
						pFramePackageMissed->exposureTime = 0;
						pCameraQueue_->push(pFramePackageMissed);
					}
				}

#else

				pFramePackage->imu.timestamp_us = (uint32_t)1000 * 1000 / CLOCKS_PER_SEC * clock();
				pFramePackage->imu.timestamp_ms = pFramePackage->imu.timestamp_us / 1000;
				pFramePackage->imu.cam = TRUE;

				pFramePackage->imu.ax = 0;
				pFramePackage->imu.ay = 0;
				pFramePackage->imu.ax = 0;
				pFramePackage->imu.az = 0;
				pFramePackage->imu.gx = 0;
				pFramePackage->imu.gy = 0;
				pFramePackage->imu.gz = 0;
				pFramePackage->imu.temperature = 0;

				pOutQueue_->push(pFramePackage);
#endif //EXTERN_CAMERA_TRIGGER
			}
		}
		else
		{
			int debug = 1;
		}

		//boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
	}
}

void InputModule::doWork()
{
	FramePackage *pFrameToDelay = NULL;
	int toDelay = -1;
	bool hasCam = false;

	while (bIsRunning_)
	{
		uint32_t count = boost::asio::read(*pPort_, boost::asio::buffer(pBuffer_, 64));
		raw_.progress(pBuffer_, count);

		while (raw_.hasNext())
		{
			LinkUpPacket packet = raw_.next();
			FramePackage* pFramePackage = NULL;

			if (liveTimeout_ <= 0)
			{
				if (((RawImuData*)packet.pData)->cam)
				{
					while (!pCameraQueue_->pop(pFramePackage))
					{
						boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
					}
					toDelay = ((int)(round(pFramePackage->exposureTime / (1000 / 200)))) - 1;
					if (toDelay >= 0)
					{
						hasCam = false;
						pFrameToDelay = pFramePackage;
						while (!pFreeQueue_->pop(pFramePackage))
						{
							boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
						}
					}
					else
					{
						hasCam = !pFramePackage->missedTrigger;
					}
				}
				else
				{
					if (toDelay == 0)
					{
						toDelay = -1;
						pFramePackage = pFrameToDelay;
						hasCam = !pFramePackage->missedTrigger;
					}
					else
					{
						hasCam = false;
						if (toDelay > 0)
							toDelay--;

						while (!pFreeQueue_->pop(pFramePackage))
						{
							boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
						}
					}
				}
				pFramePackage->imu = *((RawImuData*)packet.pData);
				pFramePackage->imu.cam = hasCam;

				while (!pOutQueue_->push(pFramePackage)) {}
			}

			free(packet.pData);
		}
		boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
	}
}