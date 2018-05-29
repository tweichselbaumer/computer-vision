#include <opencv2\opencv.hpp>
#include <opencv\highgui.h>
#include <opencv2\opencv_modules.hpp>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>

#include <boost/asio.hpp>
#include <boost/timer/timer.hpp>
#include <boost/thread.hpp>
#include "socket/tcp_server.h"

#include "AVLTree.h"
#include "Platform.h"

using boost::asio::ip::tcp;

using namespace boost::timer;
using namespace std;
using namespace cv;

boost::asio::io_service io_service;

LinkUpEventLabel* pEvent;
LinkUpNode* pLinkUpNode;

bool running = true;

void doWork()
{
	io_service.run();
}

void doWork2()
{
	VideoCapture capture = VideoCapture(1);

	capture.set(CAP_PROP_FPS, 30);
	capture.set(CAP_PROP_FRAME_WIDTH, 640);
	capture.set(CAP_PROP_FRAME_HEIGHT, 480);

	std::vector<std::vector<Point3f>> object_points;
	std::vector<std::vector<Point2f>> image_points;

	Mat image;
	Mat gray_image;
	capture >> image;

	bool png = false;

	std::vector<int> compression_params;

	if (png)
	{
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(3);
		compression_params.push_back(IMWRITE_PNG_STRATEGY);
		compression_params.push_back(IMWRITE_PNG_STRATEGY_FILTERED);
	}
	else
	{
		compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
		compression_params.push_back(30);
	}

	while (running) 
	{
		capture >> image;
		cvtColor(image, gray_image, CV_BGR2GRAY);

		std::vector<uchar> buf;

		if (png)
		{
			imencode(".png", gray_image, buf, compression_params);
		}
		else
		{
			imencode(".jpg", gray_image, buf, compression_params);
		}

		pEvent->fireEvent((uint8_t*)&buf[0], buf.size());
		//pEvent->fireEvent((uint8_t*)gray_image.data, gray_image.rows*gray_image.cols);
		boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
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

int main(int argc, char* argv[])
{
	try
	{
		pLinkUpNode = new LinkUpNode("test");

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

		boost::shared_ptr< boost::asio::io_service::work > work(
			new boost::asio::io_service::work(io_service)
		);

		tcp_server server(io_service, 3000, pLinkUpNode, 1);

		std::cout << "Press [return] to exit." << std::endl;

		boost::thread_group worker_threads;
		worker_threads.create_thread(doWork);
		worker_threads.create_thread(doWork2);
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