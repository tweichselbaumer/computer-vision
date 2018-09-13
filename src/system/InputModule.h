#ifndef _INPUT_MODULE
#define _INPUT_MODULE

#include "FramePackage.h"

#include "../camera/Camera.h"
#include "../camera/uEyeCamera.h"

#include <boost/lockfree/queue.hpp>
#include <boost/thread.hpp>
#include <boost/asio/serial_port.hpp>
#include <boost/asio.hpp>

#include "LinkUpRaw.h"
#include "LinkUpPropertyLabel.h"
#include "LinkUpFunctionLabel.h"

#include "LinkUpLabelContainer.h"

#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core/ocl.hpp>

class InputModule
{
public:
	InputModule(boost::asio::io_service& io_service, LinkUpLabelContainer* pLinkUpLabelContainer);
	void start();
	void stop();
	FramePackage* next();
	void release(FramePackage* pframePackage);
	uint8_t * onReplayData(uint8_t* pDataIn, uint32_t nSizeIn, uint32_t* pSizeOut);
private:
	enum { queueSize = 200 };
	enum { liveTimeout = 4 * 20 };

	void doWork();
	void doWorkPing();
	void doWorkCamera();

	boost::lockfree::queue<FramePackage*>* pOutQueue_;
	boost::lockfree::queue<FramePackage*>* pFreeQueue_;
	boost::lockfree::queue<FramePackage*>* pCameraQueue_;
	boost::thread thread_;
	boost::thread threadPing_;
	boost::thread cameraThread_;
	boost::asio::serial_port* pPort_;

	bool bIsRunning_ = false;
	int liveTimeout_ = 0;

	Camera* pCamera_ = new uEyeCamera();

	LinkUpLabelContainer* pLinkUpLabelContainer_;
	LinkUpRaw raw_ = {};

	uint8_t pBuffer_[64];
};
#endif