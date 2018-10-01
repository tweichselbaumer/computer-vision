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
			imshow("replay", matImg);
			cv::waitKey(1);
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
		FramePackage* pframePackage = (FramePackage*)calloc(1, sizeof(FramePackage));
		pframePackage->image.create(pCamera_->getWidth(), pCamera_->getHeight(), CV_8UC1);
		pFreeQueue_->push(pframePackage);
	}

	replayThread_ = boost::thread(boost::bind(&InputModule::doWorkReplay, this));

#ifdef EXTERN_CAMERA_TRIGGER
	pPort_->open("/dev/ttyS4");
	pPort_->set_option(boost::asio::serial_port_base::baud_rate(115200));
	threadPing_ = boost::thread(boost::bind(&InputModule::doWorkPing, this));
	thread_ = boost::thread(boost::bind(&InputModule::doWork, this));
#endif //EXTERN_CAMERA_TRIGGER

#ifdef WITH_CAMERA
	pCamera_->open();
	cameraThread_ = boost::thread(boost::bind(&InputModule::doWorkCamera, this));
#endif //WITH_CAMERA
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
		boost::this_thread::sleep_for(boost::chrono::milliseconds(200));
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
	while (bIsRunning_)
	{
		FramePackage* pFramePackage;
		while (!pFreeQueue_->pop(pFramePackage))
		{
			boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
		}
		pCamera_->capture(pFramePackage->image.data, pLinkUpLabelContainer_->pExposureLabel->getValue(), &(pFramePackage->exposureTime));

		if (liveTimeout_ > 0)
		{
			pFreeQueue_->push(pFramePackage);
			liveTimeout_--;
		}
		else
		{
#ifdef EXTERN_CAMERA_TRIGGER
			pCameraQueue_->push(pFramePackage);
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

		boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
}
}

void InputModule::doWork()
{
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
					if (pCameraQueue_->empty())
					{
						while (!pFreeQueue_->pop(pFramePackage))
						{
							boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
						}
					}
					while (pCameraQueue_->pop(pFramePackage))
					{
						if (pFramePackage != NULL)
						{
							release(pFramePackage);
						}
					}
				}
				else
				{
					while (!pFreeQueue_->pop(pFramePackage))
					{
						boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
					}
				}
				pFramePackage->imu = *((RawImuData*)packet.pData);
				pOutQueue_->push(pFramePackage);
			}

			free(packet.pData);
		}
		boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
	}
}