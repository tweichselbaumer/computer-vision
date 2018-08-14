#include "InputModule.h"

InputModule::InputModule(boost::asio::io_service& io_service, LinkUpPropertyLabel_Int16* pExposerLabel)
{
	pFreeQueue_ = new boost::lockfree::queue<FramePackage*>(queueSize);
	pOutQueue_ = new boost::lockfree::queue<FramePackage*>(queueSize);
	pCameraQueue_ = new boost::lockfree::queue<FramePackage*>(queueSize);
	pPort_ = new boost::asio::serial_port(io_service);
	pExposerLabel_ = pExposerLabel;
}

void InputModule::start()
{
	for (int i = 0; i < queueSize; i++)
	{
		FramePackage* pframePackage = (FramePackage*)calloc(1, sizeof(FramePackage));
		pframePackage->image.create(pCamera_->getWidth(), pCamera_->getHeight(), CV_8UC1);
		pFreeQueue_->push(pframePackage);
	}

	pPort_->open("/dev/ttyS4");
	pPort_->set_option(boost::asio::serial_port_base::baud_rate(115200));

	pCamera_->open();
	bIsRunning_ = true;
	thread_ = boost::thread(boost::bind(&InputModule::doWork, this));
	threadPing_ = boost::thread(boost::bind(&InputModule::doWorkPing, this));
	cameraThread_ = boost::thread(boost::bind(&InputModule::doWorkCamera, this));
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
	while (pOutQueue_->empty())
	{
		boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
	}
	pOutQueue_->pop(pFramePackage);
	return pFramePackage;
}


void InputModule::doWorkCamera()
{
	while (bIsRunning_)
	{
		FramePackage* pFramePackage;
		while (pFreeQueue_->empty())
		{
			boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
		}
		pFreeQueue_->pop(pFramePackage);
		pCamera_->capture(pFramePackage->image.data, pExposerLabel_->getValue(), &(pFramePackage->exposureTime));
		pCameraQueue_->push(pFramePackage);
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
			if (((ImuData*)packet.pData)->cam)
			{
				if (pCameraQueue_->empty())
				{
					while (pFreeQueue_->empty())
					{
						boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
					}
					pFreeQueue_->pop(pFramePackage);
				}
				while (!pCameraQueue_->empty())
				{
					if (pFramePackage != NULL)
					{
						release(pFramePackage);
					}
					pCameraQueue_->pop(pFramePackage);
				}
			}
			else
			{
				while (pFreeQueue_->empty())
				{
					boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
				}
				pFreeQueue_->pop(pFramePackage);
			}
			pFramePackage->imu = *((ImuData*)packet.pData);
			pOutQueue_->push(pFramePackage);
			free(packet.pData);
		}
		boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
	}
}