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
#include "LinkUpPropertyLabel.h"

class InputModule
{
public:
	InputModule(boost::asio::io_service& io_service, LinkUpPropertyLabel_Int16* pExposerLabel);
	void start();
	void stop();
	FramePackage* next();
	void release(FramePackage* pframePackage);
private:
	enum { queueSize = 1000 };
	void doWork();
	void doWorkPing();
	void doWorkCamera();
	boost::lockfree::queue<FramePackage*>* pOutQueue_;
	boost::lockfree::queue<FramePackage*>* pFreeQueue_;
	boost::lockfree::queue<FramePackage*>* pCameraQueue_;
	boost::thread thread_;
	boost::thread threadPing_;
	boost::thread cameraThread_;
	bool bIsRunning_ = false;
	Camera* pCamera_ = new uEyeCamera();
	LinkUpPropertyLabel_Int16* pExposerLabel_;

	uint8_t pBuffer_[64];
	LinkUpRaw raw_ = { };
	boost::asio::serial_port* pPort_;
};
#endif