#include <opencv2/opencv.hpp>
#include <opencv/highgui.h>
#include <opencv2/opencv_modules.hpp>
#include <opencv2/core/ocl.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/aruco/charuco.hpp>

#ifdef WIN32
#include <winsock2.h>
#endif// WIN32

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

#include <cstdlib>
#include <iostream>
#include <memory>
#include <utility>

#include <glog/logging.h>

#ifdef __linux
#include <unistd.h>
#endif


using boost::asio::ip::tcp;
using namespace boost::timer;
using namespace std;

boost::asio::io_service io_service;

TcpServer* pTcpServer;
LinkUpNode* pLinkUpNode;

LinkUpLabelContainer linkUpLabelContainer = {};

InputModule* pInputModule;
shared_ptr<ProgressingModule> pProgressingModule;
OutputModule* pOutputModule;

Settings* pSettings;

bool running = true;

void doWork()
{
	io_service.run();
}

void linkUpWorkerNormal()
{
	while (running) {
		pLinkUpNode->progress(0, 0, LinkUpProgressType::Normal);
		boost::this_thread::sleep_for(boost::chrono::milliseconds(1));
	}
}

void linkUpWorkerAdvanced()
{
	while (running) {
		pLinkUpNode->progress(0, 0, LinkUpProgressType::Advanced);
		boost::this_thread::sleep_for(boost::chrono::milliseconds(5));
	}
}

void loadSettings()
{
	pSettings->load();
	linkUpLabelContainer.pExposureLabel->setValue(-1);

	linkUpLabelContainer.pRecodRemoteLabel->setValue(pSettings->recordRemote);

	linkUpLabelContainer.pSlamReproducibleExecutionLabel->setValue(pSettings->reproducibleExecution);

	linkUpLabelContainer.pAccelerometerScaleLabel->setValue(pSettings->imu_parameter.accelerometer_scale);
	linkUpLabelContainer.pGyroscopeScaleLabel->setValue(pSettings->imu_parameter.gyroscope_scale);
	linkUpLabelContainer.pTemperatureOffsetLabel->setValue(pSettings->imu_parameter.temperature_offset);
	linkUpLabelContainer.pTemperatureScaleLabel->setValue(pSettings->imu_parameter.temperature_scale);

	linkUpLabelContainer.pAccelerometerNoiseDensityLabel->setValue(pSettings->imu_parameter.accelerometer_noise);
	linkUpLabelContainer.pAccelerometerRandomWalkLabel->setValue(pSettings->imu_parameter.accelerometer_walk);
	linkUpLabelContainer.pGyroscopeNoiseDensityLabel->setValue(pSettings->imu_parameter.gyroscope_noise);
	linkUpLabelContainer.pGyroscopeRandomWalkLabel->setValue(pSettings->imu_parameter.gyroscope_walk);

	linkUpLabelContainer.pImuFilterALabel->setValue((uint8_t*)pSettings->imu_filter_paramerter.pA, pSettings->imu_filter_paramerter.nA * sizeof(double));
	linkUpLabelContainer.pImuFilterBLabel->setValue((uint8_t*)pSettings->imu_filter_paramerter.pB, pSettings->imu_filter_paramerter.nB * sizeof(double));

	linkUpLabelContainer.pImuCamCalibrationTCILabel->setValue((uint8_t*)pSettings->imu_calibration.T_cam_imu, 4 * 4 * sizeof(double));
	linkUpLabelContainer.pImuCalibrationRAGLabel->setValue((uint8_t*)pSettings->imu_calibration.R_acc_imu, 3 * 3 * sizeof(double));
	linkUpLabelContainer.pImuCalibrationMinvGLabel->setValue((uint8_t*)pSettings->imu_calibration.M_inv_gyro, 3 * 3 * sizeof(double));
	linkUpLabelContainer.pImuCalibrationMinvALabel->setValue((uint8_t*)pSettings->imu_calibration.M_inv_acc, 3 * 3 * sizeof(double));
}

void updateSettings()
{
	pSettings->recordRemote = linkUpLabelContainer.pRecodRemoteLabel->getValue();
	pSettings->reproducibleExecution = linkUpLabelContainer.pSlamReproducibleExecutionLabel->getValue();

	pSettings->imu_parameter.accelerometer_scale = linkUpLabelContainer.pAccelerometerScaleLabel->getValue();
	pSettings->imu_parameter.gyroscope_scale = linkUpLabelContainer.pGyroscopeScaleLabel->getValue();
	pSettings->imu_parameter.temperature_scale = linkUpLabelContainer.pTemperatureScaleLabel->getValue();
	pSettings->imu_parameter.temperature_offset = linkUpLabelContainer.pTemperatureOffsetLabel->getValue();

	pSettings->imu_parameter.accelerometer_noise = linkUpLabelContainer.pAccelerometerNoiseDensityLabel->getValue();
	pSettings->imu_parameter.accelerometer_walk = linkUpLabelContainer.pAccelerometerRandomWalkLabel->getValue();
	pSettings->imu_parameter.gyroscope_noise = linkUpLabelContainer.pGyroscopeNoiseDensityLabel->getValue();
	pSettings->imu_parameter.gyroscope_walk = linkUpLabelContainer.pGyroscopeRandomWalkLabel->getValue();

	uint16_t nSizeA;
	uint16_t nSizeB;
	pSettings->imu_filter_paramerter.pA = (double*)linkUpLabelContainer.pImuFilterALabel->getValue(&nSizeA);
	pSettings->imu_filter_paramerter.pB = (double*)linkUpLabelContainer.pImuFilterBLabel->getValue(&nSizeB);
	pSettings->imu_filter_paramerter.nA = nSizeA / sizeof(double);
	pSettings->imu_filter_paramerter.nB = nSizeB / sizeof(double);


	uint16_t nSize;
	double* temp;

	temp = (double*)linkUpLabelContainer.pImuCamCalibrationTCILabel->getValue(&nSize);
	if (nSize != sizeof(double) * 4 * 4)
	{
		free(temp);
		temp = (double*)calloc(4 * 4, sizeof(double));
	}
	pSettings->imu_calibration.T_cam_imu = temp;

	temp = (double*)linkUpLabelContainer.pImuCalibrationRAGLabel->getValue(&nSize);
	if (nSize != sizeof(double) * 3 * 3)
	{
		free(temp);
		temp = (double*)calloc(3 * 3, sizeof(double));
	}
	pSettings->imu_calibration.R_acc_imu = temp;

	temp = (double*)linkUpLabelContainer.pImuCalibrationMinvALabel->getValue(&nSize);
	if (nSize != sizeof(double) * 3 * 3)
	{
		free(temp);
		temp = (double*)calloc(3 * 3, sizeof(double));
	}
	pSettings->imu_calibration.M_inv_acc = temp;

	temp = (double*)linkUpLabelContainer.pImuCalibrationMinvGLabel->getValue(&nSize);
	if (nSize != sizeof(double) * 3 * 3)
	{
		free(temp);
		temp = (double*)calloc(3 * 3, sizeof(double));
}
	pSettings->imu_calibration.M_inv_gyro = temp;

	pSettings->save();
}

uint8_t* onReplayData(uint8_t* pDataIn, uint32_t nSizeIn, uint32_t* pSizeOut)
{
	return pInputModule->onReplayData(pDataIn, nSizeIn, pSizeOut);
}

uint8_t* onSetSlamStatusData(uint8_t* pDataIn, uint32_t nSizeIn, uint32_t* pSizeOut)
{
	return pProgressingModule->onSetSlamStatusData(pDataIn, nSizeIn, pSizeOut);
}

uint8_t* onUpdateSettings(uint8_t* pDataIn, uint32_t nSizeIn, uint32_t* pSizeOut)
{
	*pSizeOut = 0;
	updateSettings();
	return NULL;
}

uint8_t* onChessboardCorner(uint8_t* pDataIn, uint32_t nSizeIn, uint32_t* pSizeOut)
{
	uint8_t* pOut = (uint8_t*)calloc(1, sizeof(cv::Point3f));

	*pSizeOut = sizeof(cv::Point3f);

	int32_t squaresX = *((int32_t*)pDataIn);
	int32_t squaresY = *((int32_t*)(pDataIn + 4));
	float squareLength = *((float*)(pDataIn + 8));
	float markerLength = *((float*)(pDataIn + 12));
	int32_t markerId = *((int32_t*)(pDataIn + 16));
	int32_t dictionary = *((int32_t*)(pDataIn + 20));

	cv::Ptr<cv::aruco::CharucoBoard> board = cv::aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, cv::aruco::getPredefinedDictionary(dictionary));

	*(cv::Point3f*)pOut = board->chessboardCorners[markerId];

	return pOut;
}

int main(int argc, char* argv[])
{
	FLAGS_logtostderr = 0;
	FLAGS_minloglevel = 0;

#ifdef __linux
	FLAGS_log_dir = "/opt/firefly/log";
#else
	FLAGS_log_dir = "./log";
#endif

	google::InitGoogleLogging(argv[0]);

	std::cout << "starting computer-vision...";
#ifdef __linux
	pSettings = new Settings("/opt/firefly/config.json");
#else
	pSettings = new Settings("config.json");
#endif
	pLinkUpNode = new LinkUpNode("computer_vision");

	linkUpLabelContainer.pCameraEvent = new  LinkUpEventLabel("camera_event", pLinkUpNode);
	linkUpLabelContainer.pImuEvent = new  LinkUpEventLabel("imu_event", pLinkUpNode);
	linkUpLabelContainer.pCameraImuEvent = new  LinkUpEventLabel("camera_imu_event", pLinkUpNode);

	linkUpLabelContainer.pImuDerivedEvent = new LinkUpEventLabel("imu_derived_event", pLinkUpNode);

	linkUpLabelContainer.pSlamMapEvent = new  LinkUpEventLabel("slam_map_event", pLinkUpNode);

	linkUpLabelContainer.pExposureLabel = new LinkUpPropertyLabel_Int16("camera_exposure", pLinkUpNode);
	linkUpLabelContainer.pRecodRemoteLabel = new LinkUpPropertyLabel_Boolean("record_remote", pLinkUpNode);

	linkUpLabelContainer.pAccelerometerScaleLabel = new LinkUpPropertyLabel_Double("acc_scale", pLinkUpNode);
	linkUpLabelContainer.pGyroscopeScaleLabel = new LinkUpPropertyLabel_Double("gyro_scale", pLinkUpNode);
	linkUpLabelContainer.pTemperatureScaleLabel = new LinkUpPropertyLabel_Double("temp_scale", pLinkUpNode);
	linkUpLabelContainer.pTemperatureOffsetLabel = new LinkUpPropertyLabel_Double("temp_offset", pLinkUpNode);

	linkUpLabelContainer.pReceiveReplayDataLabel = new LinkUpFunctionLabel("replay_data", pLinkUpNode);
	linkUpLabelContainer.pGetChessboardCornerLabel = new LinkUpFunctionLabel("get_chessboard_corner", pLinkUpNode);
	linkUpLabelContainer.pUpdateSettingsLabel = new LinkUpFunctionLabel("update_settings", pLinkUpNode);

	linkUpLabelContainer.pImuFilterALabel = new LinkUpPropertyLabel_Binary("imu_filter_a", 0, pLinkUpNode);
	linkUpLabelContainer.pImuFilterBLabel = new LinkUpPropertyLabel_Binary("imu_filter_b", 0, pLinkUpNode);

	linkUpLabelContainer.pImuCamCalibrationTCILabel = new LinkUpPropertyLabel_Binary("calibration_T_cam_imu", 0, pLinkUpNode);
	linkUpLabelContainer.pImuCalibrationRAGLabel = new LinkUpPropertyLabel_Binary("calibration_R_acc_gyro", 0, pLinkUpNode);
	linkUpLabelContainer.pImuCalibrationMinvALabel = new LinkUpPropertyLabel_Binary("calibration_M_inv_acc", 0, pLinkUpNode);
	linkUpLabelContainer.pImuCalibrationMinvGLabel = new LinkUpPropertyLabel_Binary("calibration_M_inv_gyro", 0, pLinkUpNode);
	linkUpLabelContainer.pAccelerometerNoiseDensityLabel = new LinkUpPropertyLabel_Double("calibration_acc_noise", pLinkUpNode);
	linkUpLabelContainer.pAccelerometerRandomWalkLabel = new LinkUpPropertyLabel_Double("calibration_acc_walk", pLinkUpNode);
	linkUpLabelContainer.pGyroscopeRandomWalkLabel = new LinkUpPropertyLabel_Double("calibration_gyro_walk", pLinkUpNode);
	linkUpLabelContainer.pGyroscopeNoiseDensityLabel = new LinkUpPropertyLabel_Double("calibration_gyro_noise", pLinkUpNode);

	linkUpLabelContainer.pSlamChangeStatusLabel = new LinkUpFunctionLabel("slam_change_status", pLinkUpNode);
	linkUpLabelContainer.pSlamStatusEvent = new LinkUpEventLabel("slam_status_event", pLinkUpNode);

	linkUpLabelContainer.pSlamReproducibleExecutionLabel = new LinkUpPropertyLabel_Boolean("slam_reproducible_execution", pLinkUpNode);

	loadSettings();

	boost::shared_ptr<boost::asio::io_service::work> work(
		new boost::asio::io_service::work(io_service)
	);

	pTcpServer = new TcpServer(io_service, 3000, pLinkUpNode, 1);

	std::cout << "Press [return] to exit." << std::endl;

	pInputModule = new InputModule(io_service, &linkUpLabelContainer);
	pOutputModule = new OutputModule(pInputModule, &linkUpLabelContainer);
	pProgressingModule = shared_ptr<ProgressingModule>(new ProgressingModule(pInputModule, pOutputModule, &linkUpLabelContainer, pSettings));

	linkUpLabelContainer.pReceiveReplayDataLabel->setFunction(&onReplayData);
	linkUpLabelContainer.pGetChessboardCornerLabel->setFunction(&onChessboardCorner);
	linkUpLabelContainer.pUpdateSettingsLabel->setFunction(&onUpdateSettings);
	linkUpLabelContainer.pSlamChangeStatusLabel->setFunction(&onSetSlamStatusData);

	boost::thread_group worker_threads;
	worker_threads.create_thread(doWork);
	worker_threads.create_thread(linkUpWorkerNormal);
	worker_threads.create_thread(linkUpWorkerAdvanced);

	pInputModule->start();
	pOutputModule->start();
	pProgressingModule->start();

#ifdef __linux
	if (!isatty(fileno(stdin)))
	{
		while (true)
			boost::this_thread::sleep_for(boost::chrono::seconds(1));
	}
#endif
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