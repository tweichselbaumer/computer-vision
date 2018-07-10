#ifndef _INPUT_MODULE
#define _INPUT_MODULE

#include "../camera/Camera.h"
#include "FramePackage.h"
#include <boost/lockfree/queue.hpp>
#include <boost/thread.hpp>
#include "../camera/uEyeCamera.h"
#include <boost/asio/serial_port.hpp>
#include "LinkUpRaw.h"
#include <boost/asio.hpp>

class InputModule
{
public:
	InputModule(boost::asio::io_service& io_service);
	void start();
	void stop();
	FramePackage* next();
	void release(FramePackage* pframePackage);
private:
	enum { queueSize = 1000 };
	void doWork();
	boost::lockfree::queue<FramePackage*>* pOutQueue_;
	boost::lockfree::queue<FramePackage*>* pFreeQueue_;
	boost::thread thread_;
	bool bIsRunning_ = false;
	Camera* pCamera_ = new uEyeCamera();

	uint8_t pBuffer_[1024];
	LinkUpRaw raw_ = { };
	boost::asio::serial_port* pPort_;
};
#endif