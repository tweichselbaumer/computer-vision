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

using boost::asio::ip::tcp;
using namespace boost::timer;
using namespace std;
using namespace cv;

boost::asio::io_service io_service;

LinkUpEventLabel* pCameraEvent;
LinkUpEventLabel* pImuEvent;
LinkUpEventLabel* pCameraImuEvent;
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
				pCameraEvent->fireEvent((uint8_t*)&buf[0], buf.size());
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
				uint8_t pTemp[buf.size() + sizeof(ImuData)];
				memcpy(pTemp + sizeof(ImuData), (uint8_t*)&buf[0], buf.size());
				*(ImuData*)pTemp = pFramePackage->imu;
				pCameraImuEvent->fireEvent(pTemp, buf.size() + sizeof(ImuData));
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

int main(int argc, char* argv[])
{
	try
	{
		pLinkUpNode = new LinkUpNode("computer_vision");

		pCameraEvent = new  LinkUpEventLabel("camera_event", pLinkUpNode);
		pImuEvent = new  LinkUpEventLabel("imu_event", pLinkUpNode);
		pCameraImuEvent = new  LinkUpEventLabel("camera_imu_event", pLinkUpNode);

		boost::shared_ptr<boost::asio::io_service::work> work(
			new boost::asio::io_service::work(io_service)
		);

		pTcpServer = new TcpServer(io_service, 3000, pLinkUpNode, 1);

		std::cout << "Press [return] to exit." << std::endl;

		pInputModule = new InputModule(io_service);

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