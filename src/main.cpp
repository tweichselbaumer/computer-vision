#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core/ocl.hpp>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>

#include <boost/asio.hpp>
#include <boost/timer/timer.hpp>
#include <boost/thread.hpp>
#include "socket/tcp_server.h"

#include <uEye.h>

#include "AvlTree.h"
#include "Platform.h"

using boost::asio::ip::tcp;

using namespace boost::timer;
using namespace std;
using namespace cv;

boost::asio::io_service io_service;

LinkUpEventLabel* pEvent;
LinkUpPropertyLabel_Int8* pQualityLabel;
LinkUpNode* pLinkUpNode;

bool running = true;

void doWork()
{
	io_service.run();
}

void doWork22()
{
	INT nRet;

	HIDS hCam = 0;
	nRet = is_InitCamera(&hCam, NULL);
	if (nRet != IS_SUCCESS)
	{
		//TODO: error
		return;
	}

	nRet = is_SetColorMode(hCam, IS_CM_MONO8);
	if (nRet != IS_SUCCESS)
	{
		//TODO: error
	}

	nRet = is_SetBinning(hCam, IS_BINNING_2X_VERTICAL | IS_BINNING_2X_HORIZONTAL);
	if (nRet != IS_SUCCESS)
	{
		//TODO: error
	}

	IS_RECT rectAOI;
	rectAOI.s32X = 64;
	rectAOI.s32Y = 0;
	rectAOI.s32Width = 512;
	rectAOI.s32Height = 512;

	nRet = is_AOI(hCam, IS_AOI_IMAGE_SET_AOI, (void*)&rectAOI, sizeof(rectAOI));
	if (nRet != IS_SUCCESS)
	{
		//TODO: error
	}

	INT id;
	char* pImgMem;

	nRet = is_AllocImageMem(hCam, 512, 512, 8, &pImgMem, &id);
	if (nRet != IS_SUCCESS)
	{
		//TODO: error
	}

	Mat image(512, 512, CV_8UC1);

	nRet = is_SetImageMem(hCam, pImgMem, id);
	if (nRet != IS_SUCCESS)
	{
		//TODO: error
	}

	UINT nRange[3];

	ZeroMemory(nRange, sizeof(nRange));

	nRet = is_PixelClock(hCam, IS_PIXELCLOCK_CMD_GET_RANGE, (void*)nRange, sizeof(nRange));

	nRet = is_PixelClock(hCam, IS_PIXELCLOCK_CMD_SET,
		(void*)&nRange[1], sizeof(nRange[1]));

	nRet = is_SetDisplayMode(hCam, IS_SET_DM_DIB);
	if (nRet != IS_SUCCESS)
	{
		//TODO: error
	}

	double enable = 1;
	is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_GAIN, &enable, 0);
	is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_WHITEBALANCE, &enable, 0);
	is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_FRAMERATE, &enable, 0);
	is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SHUTTER, &enable, 0);
	is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_GAIN, &enable, 0);
	is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_WHITEBALANCE, &enable, 0);
	is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_SHUTTER, &enable, 0);

	is_SetExternalTrigger(hCam, IS_SET_TRIGGER_SOFTWARE);

	bool webp = false;

	double FPS, NEWFPS;
	FPS = 40;
	is_SetFrameRate(hCam, FPS, &NEWFPS);

	while (running)
	{
		std::vector<int> compression_params;

		if (webp)
		{
			/*compression_params.push_back(IMWRITE_WEBP_QUALITY);
			compression_params.push_back(pQualityLabel->getValue());*/
			/*compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
			compression_params.push_back(9);*/
		}
		else
		{
			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
			compression_params.push_back(pQualityLabel->getValue());
		}

		nRet = is_FreezeVideo(hCam, IS_WAIT);
		if (nRet != IS_SUCCESS)
		{
			//TODO: error
		}

		void* pMem;

		nRet = is_GetImageMem(hCam, &pMem);
		if (nRet != IS_SUCCESS)
		{
			//TODO: error
		}

		memcpy(image.data, pMem, 512 * 512 * sizeof(uint8_t));

		std::vector<uchar> buf;
		if (pEvent->isSubscribed)
		{
			if (webp)
			{
				imencode(".bmp", image, buf, compression_params);

			}
			else
			{
				imencode(".jpg", image, buf, compression_params);
			}


			pEvent->fireEvent((uint8_t*)&buf[0], buf.size());
		}
	}
}

void doWork2()
{
	//cv::ocl::setUseOpenCL(true);
	VideoCapture capture = VideoCapture(0);

	capture.set(CAP_PROP_FPS, 30);
	capture.set(CAP_PROP_FRAME_WIDTH, 640);
	capture.set(CAP_PROP_FRAME_HEIGHT, 480);
	capture.set(CAP_PROP_MONOCHROME, true);


	std::vector<std::vector<Point3f>> object_points;
	std::vector<std::vector<Point2f>> image_points;

	Mat image;
	Mat gray_image;
	//capture >> image;
	//capture >> gray_image;

	bool webp = false;


	while (running)
	{
		std::vector<int> compression_params;

		if (webp)
		{
			/*compression_params.push_back(IMWRITE_WEBP_QUALITY);
			compression_params.push_back(pQualityLabel->getValue());*/
			/*	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
				compression_params.push_back(9);*/
		}
		else
		{
			compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
			compression_params.push_back(pQualityLabel->getValue());
		}

		capture >> image;

		//if (image.channels() == 3 || image.channels() == 4) {
		cvtColor(image, gray_image, CV_BGR2GRAY);
		/*}
		else {
			gray_image = image.clone();
		}*/

		std::vector<uchar> buf;
		if (pEvent->isSubscribed)
		{
			if (webp)
			{
				imencode(".bmp", gray_image, buf, compression_params);

			}
			else
			{
				imencode(".jpg", gray_image, buf, compression_params);
			}


			pEvent->fireEvent((uint8_t*)&buf[0], buf.size());
		}
	}

	capture.release();
}

void doWork3()
{
	while (running) {
		pLinkUpNode->progress(0, 0, 1000, false);
		boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
	}
}

void testIDS()
{
	int version = is_GetDLLVersion();
	INT nRet;

	HIDS hCam = 0;
	nRet = is_InitCamera(&hCam, NULL);
	if (nRet != IS_SUCCESS)
	{
		//TODO: error
	}

	nRet = is_SetColorMode(hCam, IS_CM_MONO8);
	if (nRet != IS_SUCCESS)
	{
		//TODO: error
	}

	nRet = is_SetBinning(hCam, IS_BINNING_2X_VERTICAL | IS_BINNING_2X_HORIZONTAL);
	if (nRet != IS_SUCCESS)
	{
		//TODO: error
	}

	IS_RECT rectAOI;
	rectAOI.s32X = 64;
	rectAOI.s32Y = 0;
	rectAOI.s32Width = 512;
	rectAOI.s32Height = 512;

	nRet = is_AOI(hCam, IS_AOI_IMAGE_SET_AOI, (void*)&rectAOI, sizeof(rectAOI));
	if (nRet != IS_SUCCESS)
	{
		//TODO: error
	}

	INT id;
	char* pImgMem;

	nRet = is_AllocImageMem(hCam, 512, 512, 8, &pImgMem, &id);
	if (nRet != IS_SUCCESS)
	{
		//TODO: error
	}

	nRet = is_SetImageMem(hCam, pImgMem, id);
	if (nRet != IS_SUCCESS)
	{
		//TODO: error
	}

	nRet = is_SetDisplayMode(hCam, IS_SET_DM_DIB);
	if (nRet != IS_SUCCESS)
	{
		//TODO: error
	}

	double enable = 1;
	is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_GAIN, &enable, 0);
	is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_WHITEBALANCE, &enable, 0);
	is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_FRAMERATE, &enable, 0);
	is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SHUTTER, &enable, 0);
	is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_GAIN, &enable, 0);
	is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_WHITEBALANCE, &enable, 0);
	is_SetAutoParameter(hCam, IS_SET_ENABLE_AUTO_SENSOR_SHUTTER, &enable, 0);

	double FPS, NEWFPS;
	FPS = 30;
	is_SetFrameRate(hCam, FPS, &NEWFPS);

	while (1) {
		nRet = is_FreezeVideo(hCam, IS_WAIT);
		if (nRet != IS_SUCCESS)
		{
			//TODO: error
		}

		void* pMem;

		nRet = is_GetImageMem(hCam, &pMem);
		if (nRet != IS_SUCCESS)
		{
			//TODO: error
		}

		Mat frame(512, 512, CV_8UC1);
		memcpy(frame.data, pMem, 512 * 512 * sizeof(uint8_t));
		imshow("A", frame);
		waitKey(1);
		std::vector<int> compression_params;
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(9);
		std::vector<uchar> buf;
		imencode(".png", frame, buf, compression_params);
	}
}

int main(int argc, char* argv[])
{
	try
	{
		pLinkUpNode = new LinkUpNode("test");

		//testIDS();

		/*for (int i = 1; i <= 5; i++) {
		char str[25] = { 0 };
		sprintf(str, "label_int_%d", i);
		LinkUpPropertyLabel_Int32* pLabel = new  LinkUpPropertyLabel_Int32(str, pLinkUpNode);
		pLabel->setValue(12);
		}

		for (int i = 1; i <= 5; i++) {
		char str[25] = { 0 };
		sprintf(str, "label_bin_%d", i);
		LinkUpPropertyLabel_Binary* pLabel = new  LinkUpPropertyLabel_Binary(str, 25, pLinkUpNode);
		pLabel->setValue((uint8_t*)str);
		}*/

		pEvent = new  LinkUpEventLabel("label_event", pLinkUpNode);
		pQualityLabel = new LinkUpPropertyLabel_Int8("jpeg_quality", pLinkUpNode);

		pQualityLabel->setValue(30);

		boost::shared_ptr< boost::asio::io_service::work > work(
			new boost::asio::io_service::work(io_service)
		);

		tcp_server server(io_service, 3000, pLinkUpNode, 1);

		std::cout << "Press [return] to exit." << std::endl;

		boost::thread_group worker_threads;
		worker_threads.create_thread(doWork);
		worker_threads.create_thread(doWork22);
		worker_threads.create_thread(doWork3);

		std::cin.get();

		running = false;
		io_service.stop();

		worker_threads.join_all();

		return 0;
	}
	catch (std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
	}

	return 0;
}