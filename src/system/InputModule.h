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
#include "LinkUpFunctionLabel.h"

#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core/ocl.hpp>

using namespace cv;

class InputModule
{
public:
	InputModule(boost::asio::io_service& io_service, LinkUpPropertyLabel_Int16* pExposerLabel);
	void start();
	void stop();
	FramePackage* next();
	void release(FramePackage* pframePackage);
	uint8_t * onReplayData(uint8_t* pDataIn, uint32_t nSizeIn, uint32_t* pSizeOut);
private:
	enum { queueSize = 1000 };
	enum { liveTimeout = 4 * 200 };
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
	int liveTimeout_ = 0;
};
#endif