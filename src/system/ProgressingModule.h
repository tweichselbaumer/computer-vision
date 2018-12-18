#ifndef _PROGRESSING_MODULE_h
#define _PROGRESSING_MODULE_h

#include "InputModule.h"
#include "OutputModule.h"
#include "SlamPublishPackage.h"

#include <boost/lockfree/queue.hpp>

#ifdef WITH_DSO
#include "Settings.h"
#include "frontend/FullSystem.h"
#include "frontend/Undistort.h"
#include "frontend/OutputWrapper.h"
#include "frontend/DSOViewer.h"
#include "NumTypes.h"

#include "../dsp/IIR.h"
#include "../io/Settings.h"

#include <Eigen/Dense>
#include "sophus/se3.hpp"

//#ifdef __linux

//#endif //__linux
#endif //WITH_DSO

#include <boost/thread.hpp>

#ifdef WITH_DSO
class ProgressingModule : public ldso::OutputWrapper
#else
class ProgressingModule
#endif //WITH_DSO
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

	~ProgressingModule();
	ProgressingModule(InputModule* pInputModule, OutputModule* pOutputModule, LinkUpLabelContainer* pLinkUpLabelContainer, Settings* pSettings);
	void start();
	void stop();
	void reinitialize();
	uint8_t* onSetSlamStatusData(uint8_t* pDataIn, uint32_t nSizeIn, uint32_t* pSizeOut);

#ifdef WITH_DSO
	void publishKeyframes(std::vector<shared_ptr<Frame>> &frames, bool final, shared_ptr<CalibHessian> HCalib);
	void publishCamPose(shared_ptr<Frame> frame, shared_ptr<CalibHessian> HCalib);
	void setMap(shared_ptr<Map> m);
	void join();
	void reset();
	void refreshAll();
#endif //WITH_DSO

private:
	boost::thread thread_;

	void doWork();
	ImuData convertImu(RawImuData imuData);
	ImuDataDerived derivedImu(ImuData imuData);

	bool bIsRunning_ = false;

	InputModule* pInputModule_ = 0;
	OutputModule* pOutputModule_ = 0;
	LinkUpLabelContainer* pLinkUpLabelContainer_ = 0;
	Settings* pSettings_ = 0;

	shared_ptr<IIR> _pImuFilterGx = 0;
	shared_ptr<IIR> _pImuFilterGy = 0;
	shared_ptr<IIR> _pImuFilterGz = 0;
	shared_ptr<IIR> _pImuFilterAx = 0;
	shared_ptr<IIR> _pImuFilterAy = 0;
	shared_ptr<IIR> _pImuFilterAz = 0;

	SlamOperationStatus currentOperationStatus_ = SlamOperationStatus::SLAM_UNKNOWN;

	SlamOverallStatus currentStatus_ = SlamOverallStatus::SLAM_STOP;
	boost::lockfree::queue<SlamOverallStatus>* pNewSlamStatusQueue_;

	Eigen::Matrix3d R_acc_gyro = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d M_acc_inv = Eigen::Matrix3d::Identity();
	Eigen::Matrix3d M_gyro_inv = Eigen::Matrix3d::Identity();
	Sophus::SE3d T_cam_imu;

#ifdef WITH_DSO
	shared_ptr<ldso::FullSystem> fullSystem = 0;
	shared_ptr<ldso::Undistort> undistorter = 0;
	shared_ptr<OutputWrapper> viewer = 0;
	int frameID = 0;
#ifdef __linux
	std::string calib = "/opt/firefly/camera.txt";
	std::string vignetteFile = "/opt/firefly/vignette.png";
	std::string gammaFile = "/opt/firefly/pcalib.txt";
#else
	std::string calib = "camera.txt";
	std::string vignetteFile = "vignette.png";
	std::string gammaFile = "pcalib.txt";
#endif //__linux

#endif //WITH_DSO
};

#endif //_PROGRESSING_MODULE_h