//#include <opencv2\opencv.hpp>
//#include <opencv\highgui.h>
//#include <opencv2\opencv_modules.hpp>
//
//#include <boost/asio.hpp>
//
//using namespace cv;
//
//int main()
//{
//	int numBoards = 0;
//	int numCornersHor;
//	int numCornersVer;
//
//	try
//	{
//		if (argc != 2)
//		{
//			std::cerr << "Usage: async_tcp_echo_server <port>\n";
//			return 1;
//		}
//
//		boost::asio::io_service io_service;
//
//		server s(io_service, std::atoi(argv[1]));
//
//		io_service.run();
//	}
//	catch (std::exception& e)
//	{
//		std::cerr << "Exception: " << e.what() << "\n";
//	}
//
//	printf("Enter number of corners along width: ");
//	scanf("%d", &numCornersHor);
//
//	printf("Enter number of corners along height: ");
//	scanf("%d", &numCornersVer);
//
//	printf("Enter number of boards: ");
//	scanf("%d", &numBoards);
//
//	int numSquares = numCornersHor * numCornersVer;
//	Size board_sz = Size(numCornersHor, numCornersVer);
//
//	VideoCapture capture = VideoCapture(0);
//
//	std::vector<std::vector<Point3f>> object_points;
//	std::vector<std::vector<Point2f>> image_points;
//
//	std::vector<Point2f> corners;
//	int successes = 0;
//
//	Mat image;
//	Mat gray_image;
//	capture >> image;
//
//
//	std::vector<Point3f> obj;
//	for (int j = 0; j<numSquares; j++)
//		obj.push_back(Point3f(j / numCornersHor, j%numCornersHor, 0.0f));
//
//	while (successes<numBoards)
//	{
//		cvtColor(image, gray_image, CV_BGR2GRAY);
//
//		bool found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);
//
//		if (found)
//		{
//			cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
//			drawChessboardCorners(gray_image, board_sz, corners, found);
//		}
//
//		imshow("win1", image);
//		imshow("win2",gray_image);
//
//		capture >> image;
//		int key = waitKey(1);
//
//		if (key == 27)
//
//			return 0;
//
//		if (key == ' ' && found != 0)
//		{
//			image_points.push_back(corners);
//			object_points.push_back(obj);
//
//			printf("Snap stored!");
//
//			successes++;
//
//			if (successes >= numBoards)
//				break;
//		}
//	}
//
//	Mat intrinsic = Mat(3, 3, CV_32FC1);
//	Mat distCoeffs;
//	std::vector<Mat> rvecs;
//	std::vector<Mat> tvecs;
//
//	intrinsic.ptr<float>(0)[0] = 1;
//	intrinsic.ptr<float>(1)[1] = 1;
//
//	calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);
//
//	Mat imageUndistorted;
//	while (1)
//	{
//		capture >> image;
//		undistort(image, imageUndistorted, intrinsic, distCoeffs);
//
//		imshow("win1", image);
//		imshow("win2", imageUndistorted);
//		waitKey(1);
//	}
//
//	capture.release();
//
//	return 0;
//}

//
// async_tcp_echo_server.cpp
// ~~~~~~~~~~~~~~~~~~~~~~~~~
//
// Copyright (c) 2003-2017 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>
#include <boost/asio.hpp>

//#include <nlohmann/json.hpp>

using boost::asio::ip::tcp;
using namespace std;

class session
	: public std::enable_shared_from_this<session>
{
public:
	session(tcp::socket socket)
		: socket_(std::move(socket))
	{
	}

	void start()
	{
		do_read();
	}

private:
	void do_read()
	{
		auto self(shared_from_this());
		socket_.async_read_some(boost::asio::buffer(data_, max_length),
			[this, self](boost::system::error_code ec, std::size_t length)
		{
			if (!ec)
			{
				do_write(length);
				do_write(length);
				do_write(length);
			}
		});
	}

	void do_write(std::size_t length)
	{
		auto self(shared_from_this());
		boost::asio::async_write(socket_, boost::asio::buffer(data_, length),
			[this, self](boost::system::error_code ec, std::size_t /*length*/)
		{
			if (!ec)
			{
				do_read();
			}
		});
	}

	tcp::socket socket_;
	enum { max_length = 1024 };
	char data_[max_length];
};

class server
{
public:
	server(boost::asio::io_service& io_service, short port)
		: acceptor_(io_service, tcp::endpoint(tcp::v4(), port)),
		socket_(io_service)
	{
		do_accept();
	}

private:
	void do_accept()
	{
		acceptor_.async_accept(socket_,
			[this](boost::system::error_code ec)
		{
			if (!ec)
			{
				std::make_shared<session>(std::move(socket_))->start();
			}

			do_accept();
		});
	}

	tcp::acceptor acceptor_;
	tcp::socket socket_;
};

int main(int argc, char* argv[])
{
	//using json = nlohmann::json;
	try
	{
		if (argc != 2)
		{
			std::cerr << "Usage: async_tcp_echo_server <port>\n";
			return 1;
		}

		boost::asio::io_service io_service;

		server s(io_service, std::atoi(argv[1]));

		io_service.run();

	}
	catch (std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
	}

	return 0;
}