#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>
#include <unistd.h>

#include <boost/asio.hpp>
#include <boost/timer/timer.hpp>
#include <boost/thread.hpp>

#include "system/InputModule.h"
#include "system/ProgressingModule.h"
#include "system/OutputModule.h"
#include "system/LinkUpLabelContainer.h"

#include "AvlTree.h"
#include "Platform.h"
#include "socket/TcpServer.h"

#include "io/Settings.h"


using boost::asio::ip::tcp;
using namespace boost::timer;
using namespace std;
using namespace cv;

boost::asio::io_service io_service;

TcpServer* pTcpServer;
LinkUpNode* pLinkUpNode;

LinkUpLabelContainer linkUpLabelContainer = {};

InputModule* pInputModule;
ProgressingModule* pProgressingModule;
OutputModule* pOutputModule;

Settings* pSettings;

bool running = true;

void doWork()
{
	io_service.run();
}

void linkUpWorker()
{
	while (running) {
		pLinkUpNode->progress(0, 0, 100, false);
		pLinkUpNode->progress(0, 0, 100, true);
		boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
	}
}


void loadSettings()
{
	pSettings->load();
	linkUpLabelContainer.pExposureLabel->setValue(-1);

	linkUpLabelContainer.pRecodRemoteLabel->setValue(pSettings->recordRemote);

	linkUpLabelContainer.pAccelerometerScaleLabel->setValue(pSettings->imu_parameter.accelerometer_scale);
	linkUpLabelContainer.pGyroscopeScaleLabel->setValue(pSettings->imu_parameter.gyroscope_scale);
	linkUpLabelContainer.pTemperatureOffsetLabel->setValue(pSettings->imu_parameter.temperature_offset);
	linkUpLabelContainer.pTemperatureScaleLabel->setValue(pSettings->imu_parameter.temperature_scale);
}

void updateSettings()
{
	pSettings->recordRemote = linkUpLabelContainer.pRecodRemoteLabel->getValue();

	pSettings->imu_parameter.accelerometer_scale = linkUpLabelContainer.pAccelerometerScaleLabel->getValue();
	pSettings->imu_parameter.gyroscope_scale = linkUpLabelContainer.pGyroscopeScaleLabel->getValue();
	pSettings->imu_parameter.temperature_scale = linkUpLabelContainer.pTemperatureScaleLabel->getValue();
	pSettings->imu_parameter.temperature_offset = linkUpLabelContainer.pTemperatureOffsetLabel->getValue();

	pSettings->save();
}

uint8_t* onReplayData(uint8_t* pDataIn, uint32_t nSizeIn, uint32_t* pSizeOut)
{
	return pInputModule->onReplayData(pDataIn, nSizeIn, pSizeOut);
}

uint8_t* onUpdateSettings(uint8_t* pDataIn, uint32_t nSizeIn, uint32_t* pSizeOut)
{
	uint8_t* pOut = 0;

	updateSettings();

	return NULL;
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
	int32_t dictionary = *((int32_t*)(pDataIn + 20));

cv:Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, cv::aruco::getPredefinedDictionary(dictionary));

	*(Point3f*)pOut = board->chessboardCorners[markerId];

	return pOut;
}

int main(int argc, char* argv[])
{
	try
	{
		pSettings = new Settings("/opt/firefly/config.json");
		pLinkUpNode = new LinkUpNode("computer_vision");

		linkUpLabelContainer.pCameraEvent = new  LinkUpEventLabel("camera_event", pLinkUpNode);
		linkUpLabelContainer.pImuEvent = new  LinkUpEventLabel("imu_event", pLinkUpNode);
		linkUpLabelContainer.pCameraImuEvent = new  LinkUpEventLabel("camera_imu_event", pLinkUpNode);

		linkUpLabelContainer.pExposureLabel = new LinkUpPropertyLabel_Int16("camera_exposure", pLinkUpNode);
		linkUpLabelContainer.pRecodRemoteLabel = new LinkUpPropertyLabel_Boolean("record_remote", pLinkUpNode);

		linkUpLabelContainer.pAccelerometerScaleLabel = new LinkUpPropertyLabel_Double("acc_scale", pLinkUpNode);
		linkUpLabelContainer.pGyroscopeScaleLabel = new LinkUpPropertyLabel_Double("gyro_scale", pLinkUpNode);
		linkUpLabelContainer.pTemperatureScaleLabel = new LinkUpPropertyLabel_Double("temp_scale", pLinkUpNode);
		linkUpLabelContainer.pTemperatureOffsetLabel = new LinkUpPropertyLabel_Double("temp_offset", pLinkUpNode);

		linkUpLabelContainer.pReceiveReplayDataLabel = new LinkUpFunctionLabel("replay_data", pLinkUpNode);
		linkUpLabelContainer.pGetChessboardCornerLabel = new LinkUpFunctionLabel("get_chessboard_corner", pLinkUpNode);
		linkUpLabelContainer.pUpdateSettingsLabel = new LinkUpFunctionLabel("update_settings", pLinkUpNode);

		loadSettings();

		boost::shared_ptr<boost::asio::io_service::work> work(
			new boost::asio::io_service::work(io_service)
		);

		pTcpServer = new TcpServer(io_service, 3000, pLinkUpNode, 1);

		std::cout << "Press [return] to exit." << std::endl;

		pInputModule = new InputModule(io_service, &linkUpLabelContainer);
		pOutputModule = new OutputModule(pInputModule, &linkUpLabelContainer);
		pProgressingModule = new ProgressingModule(pInputModule, pOutputModule, &linkUpLabelContainer);

		linkUpLabelContainer.pReceiveReplayDataLabel->setFunction(&onReplayData);
		linkUpLabelContainer.pGetChessboardCornerLabel->setFunction(&onChessboardCorner);
		linkUpLabelContainer.pUpdateSettingsLabel->setFunction(&onUpdateSettings);

		boost::thread_group worker_threads;
		worker_threads.create_thread(doWork);
		worker_threads.create_thread(linkUpWorker);

		pInputModule->start();
		pOutputModule->start();
		pProgressingModule->start();

		if (!isatty(fileno(stdin)))
		{
			while (true)
				boost::this_thread::sleep_for(boost::chrono::seconds(1));
		}
		std::cin.get();

		updateSettings();

		running = false;
		io_service.stop();

		worker_threads.join_all();

		pInputModule->stop();
		pOutputModule->stop();
		pProgressingModule->stop();


		return 0;
	}
	catch (std::exception& e)
	{
		std::cerr << "Exception: " << e.what() << "\n";
	}

	return 0;
}