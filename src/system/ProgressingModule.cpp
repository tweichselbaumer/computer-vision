#include "ProgressingModule.h"

ProgressingModule::~ProgressingModule()
{

}

uint8_t* ProgressingModule::onSetSlamStatusData(uint8_t* pDataIn, uint32_t nSizeIn, uint32_t* pSizeOut)
{
	*pSizeOut = 0;
	if (nSizeIn == 1)
		pNewSlamStatusQueue_->push(*((SlamOverallStatus*)pDataIn));
	return NULL;
}

ProgressingModule::ProgressingModule(InputModule* pInputModule, OutputModule* pOutputModule, LinkUpLabelContainer* pLinkUpLabelContainer, Settings* pSettings)
{
	pInputModule_ = pInputModule;
	pOutputModule_ = pOutputModule;
	pLinkUpLabelContainer_ = pLinkUpLabelContainer;
	pSettings_ = pSettings;
	_pImuFilterGx = shared_ptr<IIR>(new IIR(pSettings->imu_filter_paramerter.pA, pSettings->imu_filter_paramerter.pB, pSettings->imu_filter_paramerter.nA, pSettings->imu_filter_paramerter.nB));
	_pImuFilterGy = shared_ptr<IIR>(new IIR(pSettings->imu_filter_paramerter.pA, pSettings->imu_filter_paramerter.pB, pSettings->imu_filter_paramerter.nA, pSettings->imu_filter_paramerter.nB));
	_pImuFilterGz = shared_ptr<IIR>(new IIR(pSettings->imu_filter_paramerter.pA, pSettings->imu_filter_paramerter.pB, pSettings->imu_filter_paramerter.nA, pSettings->imu_filter_paramerter.nB));
	_pImuFilterAx = shared_ptr<IIR>(new IIR(pSettings->imu_filter_paramerter.pA, pSettings->imu_filter_paramerter.pB, pSettings->imu_filter_paramerter.nA, pSettings->imu_filter_paramerter.nB));
	_pImuFilterAy = shared_ptr<IIR>(new IIR(pSettings->imu_filter_paramerter.pA, pSettings->imu_filter_paramerter.pB, pSettings->imu_filter_paramerter.nA, pSettings->imu_filter_paramerter.nB));
	_pImuFilterAz = shared_ptr<IIR>(new IIR(pSettings->imu_filter_paramerter.pA, pSettings->imu_filter_paramerter.pB, pSettings->imu_filter_paramerter.nA, pSettings->imu_filter_paramerter.nB));

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			R_acc_gyro(i, j) = pSettings->imu_calibration.R_acc_imu[i * 3 + j];
			M_acc_inv(i, j) = pSettings->imu_calibration.M_inv_gyro[i * 3 + j];
			M_gyro_inv(i, j) = pSettings->imu_calibration.M_inv_acc[i * 3 + j];
		}
	}
	Eigen::Matrix4d m;
	m.setZero();
	for (int i = 0; i < 4; i++) {
		for (int j = 0; j < 4; j++) {
			m(i, j) = pSettings->imu_calibration.T_cam_imu[i * 4 + j];
		}
	}

	T_cam_imu = Sophus::SE3d(m);

	pNewSlamStatusQueue_ = new boost::lockfree::queue<SlamOverallStatus>(5);

	//runViTests();
}

void ProgressingModule::reinitialize()
{
#ifdef WITH_DSO
	ldso::setting_desiredImmatureDensity = 1500;
	ldso::setting_desiredPointDensity = 2000;
	ldso::setting_minFrames = 5;
	ldso::setting_maxFrames = 7;
	ldso::setting_maxOptIterations = 6;
	ldso::setting_minOptIterations = 1;
	ldso::setting_logStuff = false;
	ldso::setting_kfGlobalWeight = 1.3;

	ldso::setting_pointSelection = 0;

	/*ldso::benchmarkSetting_width = 256;
	ldso::benchmarkSetting_height = 256;*/

	setting_reTrackThreshold = 2;

	ldso::setting_photometricCalibration = 2;
	/*ldso::setting_affineOptModeA = 0;
	ldso::setting_affineOptModeB = 0;*/
	ldso::setting_enableLoopClosing = false;
	ldso::setting_debugout_runquiet = true;

	undistorter = shared_ptr<ldso::Undistort>(ldso::Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile));

	ldso::internal::setGlobalCalib(
		(int)undistorter->getSize()[0],
		(int)undistorter->getSize()[1],
		undistorter->getK().cast<float>());

	shared_ptr<ORBVocabulary> voc(new ORBVocabulary());
	//voc->load(vocFile);

	fullSystem = shared_ptr<ldso::FullSystem>(new ldso::FullSystem(voc));
	fullSystem->linearizeOperation = pSettings_->reproducibleExecution;

	fullSystem->setImuToCamTransformation(T_cam_imu);

	ldso::multiThreading = pSettings_->reproducibleExecution;

	viewer->reset();
	fullSystem->setViewer(viewer);

	if (undistorter->photometricUndist != 0)
		fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

	currentOperationStatus_ = SlamOperationStatus::SLAM_INITIALIZING;
#endif //WITH_DSO
}

void  ProgressingModule::start()
{
	/*viewer = shared_ptr<PangolinDSOViewer>(new PangolinDSOViewer(wG[0], hG[0], true));*/
	viewer = std::shared_ptr<ldso::OutputWrapper>(this);
	reinitialize();
	bIsRunning_ = true;
	thread_ = boost::thread(boost::bind(&ProgressingModule::doWork, this));
}

void  ProgressingModule::stop()
{
	bIsRunning_ = false;
	thread_.join();
}

ImuData ProgressingModule::convertImu(RawImuData raw)
{
	ImuData result = {};

	int multi = (int)(raw.timestamp_ms / ((pow(2, 32)) / 1000));

	result.timestamp = (raw.timestamp_us + multi * pow(2, 32)) * 1000;

	Eigen::Vector3d gyro(raw.gx, raw.gy, raw.gz);
	gyro /= pLinkUpLabelContainer_->pGyroscopeScaleLabel->getValue();
	gyro = M_gyro_inv * R_acc_gyro * gyro;

	Eigen::Vector3d acc(raw.ax, raw.ay, raw.az);
	acc = M_acc_inv * acc;
	acc /= pLinkUpLabelContainer_->pAccelerometerScaleLabel->getValue();

	result.gx = gyro.x();
	result.gy = gyro.y();
	result.gz = gyro.z();

	result.ax = acc.x();
	result.ay = acc.y();
	result.az = acc.z();

	result.temperature = raw.temperature / pLinkUpLabelContainer_->pTemperatureScaleLabel->getValue() + pLinkUpLabelContainer_->pTemperatureOffsetLabel->getValue();

	return result;
}

ldso::inertial::ImuData ProgressingModule::convertImu(ImuData imuData)
{
	ldso::inertial::ImuData result = {};

	result.time = imuData.timestamp / (1000.0 * 1000 * 1000);

	result.gx = imuData.gx / 180.0 * M_PI;
	result.gy = imuData.gy / 180.0 * M_PI;
	result.gz = imuData.gz / 180.0 * M_PI;

	result.ax = imuData.ax;
	result.ay = imuData.ay;
	result.az = imuData.az;

	return result;
}

ImuDataDerived ProgressingModule::derivedImu(ImuData data)
{
	ImuDataDerived result = {};

	result.timestamp = data.timestamp;

	result.gx = _pImuFilterGx->next(data.gx);
	result.gy = _pImuFilterGy->next(data.gy);
	result.gz = _pImuFilterGz->next(data.gz);

	result.ax = _pImuFilterAx->next(data.ax);
	result.ay = _pImuFilterAy->next(data.ay);
	result.az = _pImuFilterAz->next(data.az);

	return result;
}

void  ProgressingModule::doWork()
{
	bool isFirst = true;
	Vec7 movement;
	movement.setZero();
	SlamStatusUpdate slamStatus;

	while (bIsRunning_)
	{
		SlamOverallStatus newStatus;
		bool hasNewStatus = pNewSlamStatusQueue_->pop(newStatus);

#ifdef WITH_DSO

		if (hasNewStatus || fullSystem->isLost || ldso::setting_fullResetRequested)
		{
			if (hasNewStatus && currentStatus_ != newStatus)
				currentStatus_ = newStatus;

			if (currentStatus_ == SlamOverallStatus::SLAM_STOP)
				currentOperationStatus_ = SlamOperationStatus::SLAM_STOPPED;

			if (currentStatus_ == SlamOverallStatus::SLAM_START || currentStatus_ == SlamOverallStatus::SLAM_RESTART || fullSystem->isLost || ldso::setting_fullResetRequested)
			{
				currentStatus_ = SlamOverallStatus::SLAM_START;

				reinitialize();

				if (pSettings_->reproducibleExecution)
				{
					movement.setZero();
					isFirst = true;

					_pImuFilterGx->reset();
					_pImuFilterGy->reset();
					_pImuFilterGz->reset();
					_pImuFilterAx->reset();
					_pImuFilterAy->reset();
					_pImuFilterAz->reset();
				}
			}
		}

		if (hasNewStatus)
			continue;

#endif //WITH_DSO

		FramePackage* pFramePackage = pInputModule_->next();
		if (pFramePackage != NULL)
		{
			OutputPackage* pOutputPackage = pOutputModule_->nextFreeOutputPackage();
			pOutputPackage->pFramePackage = pFramePackage;

			pOutputPackage->imuData = convertImu(pOutputPackage->pFramePackage->imu);

			if (isFirst)
			{
				Vec6 move;
				int i = 0;

				do
				{
					i++;
					pOutputPackage->imuDataDerived = derivedImu(pOutputPackage->imuData);
					move[0] = pOutputPackage->imuDataDerived.gx;
					move[1] = pOutputPackage->imuDataDerived.gy;
					move[2] = pOutputPackage->imuDataDerived.gz;
					move[3] = pOutputPackage->imuDataDerived.ax;
					move[4] = pOutputPackage->imuDataDerived.ay;
					move[5] = pOutputPackage->imuDataDerived.az;
				} while (i < setting_vi_nMaxIterationsIIRInitialization && move.norm() > setting_vi_epsilonIIRInitialization);
			}
			else
			{
				pOutputPackage->imuDataDerived = derivedImu(pOutputPackage->imuData);
			}

			movement[0] += pOutputPackage->imuDataDerived.gx;
			movement[1] += pOutputPackage->imuDataDerived.gy;
			movement[2] += pOutputPackage->imuDataDerived.gz;
			movement[3] += pOutputPackage->imuDataDerived.ax;
			movement[4] += pOutputPackage->imuDataDerived.ay;
			movement[5] += pOutputPackage->imuDataDerived.az;
			movement[6] += 1.0;

#ifdef WITH_DSO

			bool hasMotion = false;

			if (pFramePackage->imu.cam && currentStatus_ == SlamOverallStatus::SLAM_START)
			{
				Vec6 mov = (movement / movement[6]).block(0, 0, 6, 1);

				hasMotion = mov.norm() > setting_vi_hasMovementThreshold;

				if (fullSystem->initialized || hasMotion)
				{
					vector<ldso::inertial::ImuData> imuDataHistory;
					int queueSize = imuQueue.size();
					for (int i = 0; i < queueSize; i++)
					{
						imuDataHistory.push_back(imuQueue.front());
						imuQueue.pop_front();
					}
					ldso::MinimalImageB minImg((int)pFramePackage->image.cols, (int)pFramePackage->image.rows, (unsigned char*)pFramePackage->image.data);
					ldso::ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, pFramePackage->exposureTime, pOutputPackage->imuData.timestamp / (1000.0 * 1000 * 1000), 1.0f);
					fullSystem->addActiveFrame(undistImg, imuDataHistory);

					delete undistImg;
				}

				if (movement[6] > setting_vi_hasMovementResetPeriod)
					movement.setZero();
			}

			if (pFramePackage->imu.cam)
			{
				if (fullSystem->initialized && currentStatus_ == SlamOverallStatus::SLAM_START)
					currentOperationStatus_ = SlamOperationStatus::SLAM_RUNNING;
				else if (currentStatus_ == SlamOverallStatus::SLAM_STOP)
					currentOperationStatus_ = SlamOperationStatus::SLAM_STOPPED;
				else if (!fullSystem->initialized && hasMotion && currentStatus_ == SlamOverallStatus::SLAM_START)
					currentOperationStatus_ = SlamOperationStatus::SLAM_INITIALIZING;
				else if (!fullSystem->initialized && !hasMotion && currentStatus_ == SlamOverallStatus::SLAM_START)
					currentOperationStatus_ = SlamOperationStatus::SLAM_WAITING_FOR_MOTION;
			}

			imuQueue.push_back(convertImu(pOutputPackage->imuData));

#endif //WITH_DSO

			pOutputModule_->writeOut(pOutputPackage);
		}

		slamStatus.operationStatus = currentOperationStatus_;
		pOutputModule_->writeOut(slamStatus);
	}
}

#ifdef WITH_DSO

void ProgressingModule::runViTests()
{
	srand(1217893891703);
	vector<inertial::ImuData> data;

	shared_ptr<inertial::PreIntegration> exactPreIntegration = shared_ptr<inertial::PreIntegration>(new inertial::PreIntegration());
	shared_ptr<inertial::PreIntegration> approxPreIntegration = shared_ptr<inertial::PreIntegration>(new inertial::PreIntegration());

	double scale = 100.0;

	for (int i = 0; i < 20; i++)
	{
		inertial::ImuData d;
		d.ax = (rand() % 100) / scale;
		d.ay = (rand() % 100) / scale;
		d.az = (rand() % 100) / scale;
		d.gx = (rand() % 100) / scale;
		d.gy = (rand() % 100) / scale;
		d.gz = (rand() % 100) / scale;

		data.push_back(d);
	}

	scale = 10000;
	Vec3 db_g((rand() % 100) / scale, (rand() % 100) / scale, (rand() % 100) / scale);
	Vec3 db_a((rand() % 100) / scale, (rand() % 100) / scale, (rand() % 100) / scale);

	scale = 1000.0;
	Vec3 b_g((rand() % 100) / scale, (rand() % 10) / scale, (rand() % 100) / scale);
	Vec3 b_a((rand() % 100) / scale, (rand() % 10) / scale, (rand() % 100) / scale);

	exactPreIntegration->lin_bias_g = b_g + db_g;
	exactPreIntegration->lin_bias_a = b_a + db_a;

	approxPreIntegration->lin_bias_g = b_g;
	approxPreIntegration->lin_bias_a = b_a;

	approxPreIntegration->addImuData(data);
	exactPreIntegration->addImuData(data);

	std::cout << "Error R: " << std::setprecision(15) << (exactPreIntegration->delta_R_ij * ((approxPreIntegration->delta_R_ij * SO3::exp(approxPreIntegration->d_delta_R_ij_dg * db_g)).inverse())).log().norm() << std::endl;
	std::cout << "Error v: " << std::setprecision(15) << (exactPreIntegration->delta_v_ij - (approxPreIntegration->delta_v_ij + approxPreIntegration->d_delta_v_ij_dg*db_g + approxPreIntegration->d_delta_v_ij_da*db_a)).norm() << std::endl;
	std::cout << "Error p: " << std::setprecision(15) << (exactPreIntegration->delta_p_ij - (approxPreIntegration->delta_p_ij + approxPreIntegration->d_delta_p_ij_dg*db_g + approxPreIntegration->d_delta_p_ij_da*db_a)).norm() << std::endl;


	shared_ptr<inertial::InertialFrameFrameHessian> exactFFH = shared_ptr<inertial::InertialFrameFrameHessian>(new inertial::InertialFrameFrameHessian(approxPreIntegration));
	shared_ptr<inertial::InertialFrameFrameHessian> approxFFH = shared_ptr<inertial::InertialFrameFrameHessian>(new inertial::InertialFrameFrameHessian(approxPreIntegration));

	shared_ptr<inertial::InertialFrameHessian> exactFH_from = shared_ptr<inertial::InertialFrameHessian>(new inertial::InertialFrameHessian());
	shared_ptr<inertial::InertialFrameHessian> exactFH_to = shared_ptr<inertial::InertialFrameHessian>(new inertial::InertialFrameHessian());
	shared_ptr<inertial::InertialFrameHessian> approxFH_from = shared_ptr<inertial::InertialFrameHessian>(new inertial::InertialFrameHessian());
	shared_ptr<inertial::InertialFrameHessian> approxFH_to = shared_ptr<inertial::InertialFrameHessian>(new inertial::InertialFrameHessian());

	exactFFH->from = exactFH_from;
	exactFH_from->from = exactFFH;
	exactFFH->to = exactFH_to;
	exactFH_to->to = exactFFH;

	approxFFH->from = approxFH_from;
	approxFH_from->from = approxFFH;
	approxFFH->to = approxFH_to;
	approxFH_to->to = approxFFH;

	scale = 1000;
	exactFH_from->W_v_B_EvalPT = Vec3((rand() % 100) / scale, (rand() % 100) / scale, (rand() % 100) / scale);
	exactFH_to->W_v_B_EvalPT = Vec3((rand() % 100) / scale, (rand() % 100) / scale, (rand() % 100) / scale);

	exactFH_from->db_a_EvalPT = Vec3((rand() % 100) / scale, (rand() % 100) / scale, (rand() % 100) / scale);
	exactFH_to->db_a_EvalPT = Vec3((rand() % 100) / scale, (rand() % 100) / scale, (rand() % 100) / scale);

	exactFH_from->db_g_EvalPT = Vec3((rand() % 100) / scale, (rand() % 100) / scale, (rand() % 100) / scale);
	exactFH_to->db_g_EvalPT = Vec3((rand() % 100) / scale, (rand() % 100) / scale, (rand() % 100) / scale);

	Vec6 uw_from;
	Vec6 uw_to;

	for (int i = 0; i < 6; i++)
	{
		uw_from[i] = (rand() % 100) / scale;
		uw_to[i] = (rand() % 100) / scale;
	}

	exactFH_from->T_WB_EvalPT = SE3::exp(uw_from);
	exactFH_to->T_WB_EvalPT = SE3::exp(uw_to);

	exactFH_from->T_BW_EvalPT = exactFH_from->T_WB_EvalPT.inverse();
	exactFH_to->T_BW_EvalPT = exactFH_to->T_WB_EvalPT.inverse();

	//--------------
	approxFH_from->W_v_B_EvalPT = exactFH_from->W_v_B_EvalPT;
	approxFH_to->W_v_B_EvalPT = exactFH_to->W_v_B_EvalPT;

	approxFH_from->db_a_EvalPT = exactFH_from->db_a_EvalPT;
	approxFH_to->db_a_EvalPT = exactFH_to->db_a_EvalPT;

	approxFH_from->db_g_EvalPT = exactFH_from->db_g_EvalPT;
	approxFH_to->db_g_EvalPT = exactFH_to->db_g_EvalPT;

	approxFH_from->T_WB_EvalPT = exactFH_from->T_WB_EvalPT;
	approxFH_to->T_WB_EvalPT = exactFH_to->T_WB_EvalPT;

	approxFH_from->T_BW_EvalPT = exactFH_from->T_BW_EvalPT;
	approxFH_to->T_BW_EvalPT = exactFH_to->T_BW_EvalPT;

	Vec15 x_from;
	Vec15 dx_from;

	Vec15 x_to;
	Vec15 dx_to;

	vector<Mat1515> s;

	for (int i = 0; i < 15; i++)
	{
		scale = 1000;
		x_from[i] = (rand() % 100) / scale;
		x_to[i] = (rand() % 100) / scale;
		scale = 100000;
		dx_from[i] = (rand() % 100) / scale;
		dx_to[i] = (rand() % 100) / scale;
		s.push_back(Mat1515());
		s[i].setZero();
		s[i].block<1, 1>(i, i) = Mat11::Identity();
	}

	Mat1515 select;

	approxFH_from->setState(x_from);
	approxFH_to->setState(x_to);
	approxFFH->linearize();

	//========= dr/du:
	select = s[0] + s[1] + s[2];

	exactFH_from->setState(x_from + select * dx_from);
	exactFH_to->setState(x_to);
	exactFFH->linearize();

	std::cout << "Error dr/du_i: " << std::setprecision(15) << (exactFFH->r - (approxFFH->r + approxFFH->J_from * select * dx_from)).norm() / exactFFH->r.norm() << std::endl;

	exactFH_from->setState(x_from);
	exactFH_to->setState(x_to + select * dx_to);
	exactFFH->linearize();
	std::cout << "Error dr/du_j: " << std::setprecision(15) << (exactFFH->r - (approxFFH->r + approxFFH->J_to * select * dx_to)).norm() / exactFFH->r.norm() << std::endl;

	//========= dr/dw:
	select = s[3] + s[4] + s[5];

	exactFH_from->setState(x_from + select * dx_from);
	exactFH_to->setState(x_to);
	exactFFH->linearize();

	std::cout << "Error dr/dw_i: " << std::setprecision(15) << (exactFFH->r - (approxFFH->r + approxFFH->J_from * select * dx_from)).norm()/ exactFFH->r.norm() << std::endl;

	exactFH_from->setState(x_from);
	exactFH_to->setState(x_to + select * dx_to);
	exactFFH->linearize();
	std::cout << "Error dr/dw_j: " << std::setprecision(15) << (exactFFH->r - (approxFFH->r + approxFFH->J_to * select * dx_to)).norm()/ exactFFH->r.norm() << std::endl;

	//========= dr/dv:
	select = s[6] + s[7] + s[8];

	exactFH_from->setState(x_from + select * dx_from);
	exactFH_to->setState(x_to);
	exactFFH->linearize();

	std::cout << "Error dr/dv_i: " << std::setprecision(15) << (exactFFH->r - (approxFFH->r + approxFFH->J_from * select * dx_from)).norm() / exactFFH->r.norm() << std::endl;

	exactFH_from->setState(x_from);
	exactFH_to->setState(x_to + select * dx_to);
	exactFFH->linearize();
	std::cout << "Error dr/dv_j: " << std::setprecision(15) << (exactFFH->r - (approxFFH->r + approxFFH->J_to * select * dx_to)).norm() / exactFFH->r.norm() << std::endl;

	//========= dr/dbg:
	select = s[9] + s[10] + s[11];

	exactFH_from->setState(x_from + select * dx_from);
	exactFH_to->setState(x_to);
	exactFFH->linearize();

	std::cout << "Error dr/dbg_i: " << std::setprecision(15) << (exactFFH->r - (approxFFH->r + approxFFH->J_from * select * dx_from)).norm() / exactFFH->r.norm() << std::endl;

	exactFH_from->setState(x_from);
	exactFH_to->setState(x_to + select * dx_to);
	exactFFH->linearize();
	std::cout << "Error dr/dbg_j: " << std::setprecision(15) << (exactFFH->r - (approxFFH->r + approxFFH->J_to * select * dx_to)).norm() / exactFFH->r.norm() << std::endl;

	//========= dr/dbg:
	select = s[12] + s[13] + s[14];

	exactFH_from->setState(x_from + select * dx_from);
	exactFH_to->setState(x_to);
	exactFFH->linearize();

	std::cout << "Error dr/dba_i: " << std::setprecision(15) << (exactFFH->r - (approxFFH->r + approxFFH->J_from * select * dx_from)).norm() / exactFFH->r.norm() << std::endl;

	exactFH_from->setState(x_from);
	exactFH_to->setState(x_to + select * dx_to);
	exactFFH->linearize();
	std::cout << "Error dr/dba_j: " << std::setprecision(15) << (exactFFH->r - (approxFFH->r + approxFFH->J_to * select * dx_to)).norm() / exactFFH->r.norm() << std::endl;
}

void ProgressingModule::publishKeyframes(std::vector<shared_ptr<Frame>> &frames, bool final, shared_ptr<CalibHessian> HCalib, shared_ptr<inertial::InertialHessian> HInertial)
{
	SE3 T_wd_w_temp;
	{
		unique_lock<mutex> lck(inertialMutex);
		T_wd_w = SE3(HInertial->R_DW_PRE, Vec3(0, 0, 0));
		T_wd_w_temp = T_wd_w;
	}

	for (shared_ptr<Frame> frame : frames)
	{
		if (frame->frameHessian && frame->frameHessian->flaggedForMarginalization)
		{
			Sim3 T_c_wd_ = frame->getPoseOpti();
			SE3 T_c_wd = SE3(T_c_wd_.quaternion(), T_c_wd_.translation());
			SE3 T_c_w = T_c_wd * T_wd_w_temp;

			Eigen::Vector3d translation = T_c_w.translation();
			Eigen::Quaterniond rotation = T_c_w.unit_quaternion();

			SlamPublishPackage* pSlamPublishPackage = new SlamPublishPackage();
			pSlamPublishPackage->publishType = SlamPublishType::SLAM_PUBLISH_KEY_FRAME;

			pSlamPublishPackage->frame.id = frame->id;
			pSlamPublishPackage->frame.timestamp = frame->timeStamp;
			pSlamPublishPackage->frame.tx = translation.x();
			pSlamPublishPackage->frame.ty = translation.y();
			pSlamPublishPackage->frame.tz = translation.z();

			pSlamPublishPackage->frame.q1 = rotation.w();
			pSlamPublishPackage->frame.q2 = rotation.x();
			pSlamPublishPackage->frame.q3 = rotation.y();
			pSlamPublishPackage->frame.q4 = rotation.z();

			pSlamPublishPackage->frame.s = 1;

			pSlamPublishPackage->pKeyFrame = (SlamPublishKeyFrame*)calloc(1, sizeof(SlamPublishKeyFrame));
			pSlamPublishPackage->pKeyFrame->id = frame->kfId;
			pSlamPublishPackage->pKeyFrame->cx = HCalib->cxl();
			pSlamPublishPackage->pKeyFrame->cy = HCalib->cyl();
			pSlamPublishPackage->pKeyFrame->fx = HCalib->fxl();
			pSlamPublishPackage->pKeyFrame->fy = HCalib->fyl();

			for (auto feat : frame->features)
			{
				if (feat->point && feat->point->mpPH)
				{
					auto p = feat->point->mpPH;

					float depth = 1.0f / (p->idepth_scaled);
					float depth4 = depth * depth;
					depth4 *= depth4;
					float var = (1.0f / (p->idepth_hessian + 0.01));
					float my_minRelBS = 0;
					float my_scaledTH = 1e10, my_absTH = 1e10, my_scale = 0;

					if (var * depth4 > my_scaledTH)
						continue;

					if (var > my_absTH)
						continue;

					if (p->maxRelBaseline < my_minRelBS)
						continue;

					SlamPublishPoint* pPoint = (SlamPublishPoint*)calloc(1, sizeof(SlamPublishPoint));
					for (int i = 0; i < 8; i++)
					{
						pPoint->color[i] = p->color[i];
					}
					pPoint->u = p->u;
					pPoint->v = p->v;
					pPoint->inverseDepth = p->idepth_scaled;
					pSlamPublishPackage->points.push_back(pPoint);
				}
			}

			pOutputModule_->writeOut(pSlamPublishPackage);
		}
	}
}

void ProgressingModule::publishCamPose(shared_ptr<Frame> frame, shared_ptr<CalibHessian> HCalib, shared_ptr<inertial::InertialHessian> HInertial)
{
	SE3 T_c_wd = frame->getPose();
	SE3 T_c_w;

	{
		unique_lock<mutex> lck(inertialMutex);
		T_c_w = T_c_wd * T_wd_w;
	}

	Eigen::Vector3d translation = T_c_w.translation();
	Eigen::Quaterniond rotation = T_c_w.unit_quaternion();

	SlamPublishPackage* pSlamPublishPackage = new SlamPublishPackage();
	pSlamPublishPackage->publishType = SlamPublishType::SLAM_PUBLISH_FRAME;

	pSlamPublishPackage->frame.id = frame->id;
	pSlamPublishPackage->frame.timestamp = frame->timeStamp;
	pSlamPublishPackage->frame.tx = translation.x();
	pSlamPublishPackage->frame.ty = translation.y();
	pSlamPublishPackage->frame.tz = translation.z();

	pSlamPublishPackage->frame.q1 = rotation.w();
	pSlamPublishPackage->frame.q2 = rotation.x();
	pSlamPublishPackage->frame.q3 = rotation.y();
	pSlamPublishPackage->frame.q4 = rotation.z();

	pSlamPublishPackage->frame.s = 1;

	pOutputModule_->writeOut(pSlamPublishPackage);
}

void ProgressingModule::setMap(shared_ptr<Map> m)
{

}

void ProgressingModule::join()
{

}

void ProgressingModule::reset()
{
	SlamPublishPackage* pSlamPublishPackage = new SlamPublishPackage();
	pSlamPublishPackage->publishType = SlamPublishType::SLAM_RESET;

	pOutputModule_->writeOut(pSlamPublishPackage);
}

void ProgressingModule::refreshAll()
{

}
#endif //WITH_DSO
