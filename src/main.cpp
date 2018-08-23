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

#include "system/InputModule.h"

#include "AvlTree.h"
#include "Platform.h"
#include "socket/TcpServer.h"

#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

using boost::asio::ip::tcp;
using namespace boost::timer;
using namespace std;
using namespace cv;

boost::asio::io_service io_service;

LinkUpEventLabel* pCameraEvent;
LinkUpEventLabel* pImuEvent;
LinkUpEventLabel* pCameraImuEvent;

LinkUpPropertyLabel_Int16* pExposureLabel;

LinkUpFunctionLabel* pReceiveReplayDataLabel;
LinkUpFunctionLabel* pGetChessboardCornerLabel;

TcpServer* pTcpServer;
LinkUpNode* pLinkUpNode;

InputModule* pInputModule;

bool running = true;

void doWork()
{
	io_service.run();
}

void doWork2()
{
	std::vector<int> compression_params;
	/*compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(0);*/

	while (running)
	{
		FramePackage* pFramePackage = pInputModule->next();
		if (pFramePackage->imu.cam)
		{
			std::vector<uchar> buf;
			if (pCameraEvent->isSubscribed)
			{
				imencode(".bmp", pFramePackage->image, buf, compression_params);
				uint8_t pTemp[buf.size() + sizeof(double)];
				memcpy(pTemp, &pFramePackage->exposureTime, sizeof(double));
				memcpy(pTemp + sizeof(double), (uint8_t*)&buf[0], buf.size());
				pCameraEvent->fireEvent(pTemp, buf.size() + sizeof(double));
			}
		}
		if (pImuEvent->isSubscribed)
		{
			pImuEvent->fireEvent((uint8_t*)&(pFramePackage->imu), sizeof(ImuData));
		}
		if (pCameraImuEvent->isSubscribed)
		{
			if (pFramePackage->imu.cam)
			{
				std::vector<uchar> buf;

				imencode(".bmp", pFramePackage->image, buf, compression_params);
				uint8_t pTemp[buf.size() + sizeof(ImuData) + sizeof(double)];
				memcpy(pTemp + sizeof(ImuData), &pFramePackage->exposureTime, sizeof(double));
				memcpy(pTemp + sizeof(ImuData) + sizeof(double), (uint8_t*)&buf[0], buf.size());
				*(ImuData*)pTemp = pFramePackage->imu;
				pCameraImuEvent->fireEvent(pTemp, buf.size() + sizeof(ImuData) + sizeof(double));
			}
			else
			{
				pCameraImuEvent->fireEvent((uint8_t*)&(pFramePackage->imu), sizeof(ImuData));
			}
		}
		pInputModule->release(pFramePackage);
	}
}

void linkUpWorker()
{
	while (running) {
		pLinkUpNode->progress(0, 0, 100, false);
		pLinkUpNode->progress(0, 0, 100, true);
		boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
	}
}

uint8_t* onReplayData(uint8_t* pDataIn, uint32_t nSizeIn, uint32_t* pSizeOut)
{
	return pInputModule->onReplayData(pDataIn, nSizeIn, pSizeOut);
}

uint8_t* onChessboardCorner(uint8_t* pDataIn, uint32_t nSizeIn, uint32_t* pSizeOut)
{
	uint8_t* pOut = (uint8_t*)calloc(1, sizeof(Point3f));

	*pSizeOut = sizeof(Point3f);

	int32_t squaresX = *((int32_t*)pDataIn);
	int32_t squaresY = *((int32_t*)(pDataIn + 4));
	float squareLength = *((float*)(pDataIn + 8));
	float markerLength = *((float*)(pDataIn + 12));
	int32_t markerId = *((int32_t*)(pDataIn + 16));

cv:Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250));

	*(Point3f*)pOut = board->chessboardCorners[markerId];

	return pOut;
}

int main(int argc, char* argv[])
{
	try
	{
		pLinkUpNode = new LinkUpNode("computer_vision");

		pCameraEvent = new  LinkUpEventLabel("camera_event", pLinkUpNode);
		pImuEvent = new  LinkUpEventLabel("imu_event", pLinkUpNode);
		pCameraImuEvent = new  LinkUpEventLabel("camera_imu_event", pLinkUpNode);

		pExposureLabel = new LinkUpPropertyLabel_Int16("camera_exposure", pLinkUpNode);
		pExposureLabel->setValue(-1);

		pReceiveReplayDataLabel = new LinkUpFunctionLabel("replay_data", pLinkUpNode);
		pGetChessboardCornerLabel = new LinkUpFunctionLabel("get_chessboard_corner", pLinkUpNode);

		boost::shared_ptr<boost::asio::io_service::work> work(
			new boost::asio::io_service::work(io_service)
		);

		pTcpServer = new TcpServer(io_service, 3000, pLinkUpNode, 1);

		std::cout << "Press [return] to exit." << std::endl;

		pInputModule = new InputModule(io_service, pExposureLabel);
		pReceiveReplayDataLabel->setFunction(&onReplayData);
		pGetChessboardCornerLabel->setFunction(&onChessboardCorner);

		boost::thread_group worker_threads;
		worker_threads.create_thread(doWork);
		worker_threads.create_thread(doWork2);
		worker_threads.create_thread(linkUpWorker);

		pInputModule->start();

		std::cin.get();

		running = false;
		io_service.stop();

		worker_threads.join_all();
		pInputModule->stop();

		return 0;
	}
	catch (std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
	}

	return 0;
}