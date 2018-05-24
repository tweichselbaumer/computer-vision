#include <opencv2\opencv.hpp>
#include <opencv\highgui.h>
#include <opencv2\opencv_modules.hpp>

using namespace cv;

int main()
{
	int numBoards = 10;
	int numCornersHor = 10;
	int numCornersVer = 10;

	/*printf("Enter number of corners along width: ");
	scanf("%d", &numCornersHor);

	printf("Enter number of corners along height: ");
	scanf("%d", &numCornersVer);

	printf("Enter number of boards: ");
	scanf("%d", &numBoards);*/

	int numSquares = numCornersHor * numCornersVer;
	Size board_sz = Size(numCornersHor, numCornersVer);

	VideoCapture capture = VideoCapture(0);

	std::vector<std::vector<Point3f>> object_points;
	std::vector<std::vector<Point2f>> image_points;

	std::vector<Point2f> corners;
	int successes = 0;

	Mat image;
	Mat gray_image;
	capture >> image;

	bool png = false;


	std::vector<int> compression_params;

	if (png) 
	{
		compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
		compression_params.push_back(2);
	}
	else
	{
		compression_params.push_back(CV_IMWRITE_JPEG_QUALITY);
		compression_params.push_back(90);
	}

	//imencode(".png", image, buf, compression_params);

	while (true) {
		cvtColor(image, gray_image, CV_BGR2GRAY);
		std::vector<uchar> buf;
		int64 start = cv::getTickCount();
		imshow("win1", image);
		if (png)
		{
			imencode(".png", gray_image, buf, compression_params);
			//imwrite("test.png", gray_image, compression_params);
		}
		else 
		{
			imencode(".jpg", gray_image, buf, compression_params);
			//imwrite("test.jpg", gray_image, compression_params);
		}
		imshow("win2", imdecode(buf,0));
		capture >> image;
		int key = waitKey(1);

		if (key == 27)
			return 0;

		double fps = cv::getTickFrequency() / (cv::getTickCount() - start);
		std::cout << "FPS : " << fps << "  Compress Size(KB): " << (double)buf.size() / 1024 << "  Orginal Size(KB): " << (double)(image.size().width* image.size().height) / 1024 << "  MBit/s:" << (double)buf.size() / 1024 /1024*fps << std::endl;
	}

	std::vector<Point3f> obj;
	for (int j = 0; j < numSquares; j++)
		obj.push_back(Point3f(j / numCornersHor, j%numCornersHor, 0.0f));

	while (successes < numBoards)
	{
		cvtColor(image, gray_image, CV_BGR2GRAY);

		bool found = findChessboardCorners(image, board_sz, corners, CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FILTER_QUADS);

		if (found)
		{
			cornerSubPix(gray_image, corners, Size(11, 11), Size(-1, -1), TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
			drawChessboardCorners(gray_image, board_sz, corners, found);
		}

		imshow("win1", image);
		imshow("win2", gray_image);

		capture >> image;
		int key = waitKey(1);

		if (key == 27)

			return 0;

		if (key == ' ' && found != 0)
		{
			image_points.push_back(corners);
			object_points.push_back(obj);

			printf("Snap stored!");

			successes++;

			if (successes >= numBoards)
				break;
		}
	}

	Mat intrinsic = Mat(3, 3, CV_32FC1);
	Mat distCoeffs;
	std::vector<Mat> rvecs;
	std::vector<Mat> tvecs;

	intrinsic.ptr<float>(0)[0] = 1;
	intrinsic.ptr<float>(1)[1] = 1;

	calibrateCamera(object_points, image_points, image.size(), intrinsic, distCoeffs, rvecs, tvecs);

	Mat imageUndistorted;
	while (1)
	{
		capture >> image;
		undistort(image, imageUndistorted, intrinsic, distCoeffs);

		imshow("win1", image);
		imshow("win2", imageUndistorted);
		waitKey(1);
	}

	capture.release();

	return 0;
}