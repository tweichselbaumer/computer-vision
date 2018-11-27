#include "ProgressingModule.h"

ProgressingModule::ProgressingModule(InputModule* pInputModule, OutputModule* pOutputModule, LinkUpLabelContainer* pLinkUpLabelContainer)
{
	pInputModule_ = pInputModule;
	pOutputModule_ = pOutputModule;
	pLinkUpLabelContainer_ = pLinkUpLabelContainer;
}

void  ProgressingModule::start()
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

	undistorter = ldso::Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

	ldso::internal::setGlobalCalib(
		(int)undistorter->getSize()[0],
		(int)undistorter->getSize()[1],
		undistorter->getK().cast<float>());

	shared_ptr<ORBVocabulary> voc(new ORBVocabulary());
	//voc->load(vocFile);

	fullSystem = new ldso::FullSystem(voc);
	fullSystem->linearizeOperation = singleThread;
	this->reset();
	/*viewer = shared_ptr<PangolinDSOViewer>(new PangolinDSOViewer(wG[0], hG[0], true));
	fullSystem->setViewer(viewer);*/
	fullSystem->setViewer(std::shared_ptr<ldso::OutputWrapper>(this));

	if (undistorter->photometricUndist != 0)
		fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

#endif //WITH_DSO

	bIsRunning_ = true;
	thread_ = boost::thread(boost::bind(&ProgressingModule::doWork, this));
}

void  ProgressingModule::stop()
{
	bIsRunning_ = false;
	thread_.join();

#ifdef WITH_DSO
	/*delete undistorter;
	delete fullSystem;*/
#endif //WITH_DSO
}

ImuData ProgressingModule::convertImu(RawImuData raw)
{
	ImuData result = {};

	int multi = (int)(raw.timestamp_ms / ((pow(2, 32)) / 1000));

	result.timestamp = (raw.timestamp_us + multi * pow(2, 32)) * 1000;

	result.gx = raw.gx / pLinkUpLabelContainer_->pGyroscopeScaleLabel->getValue();
	result.gy = raw.gy / pLinkUpLabelContainer_->pGyroscopeScaleLabel->getValue();
	result.gz = raw.gz / pLinkUpLabelContainer_->pGyroscopeScaleLabel->getValue();

	result.ax = raw.ax / pLinkUpLabelContainer_->pAccelerometerScaleLabel->getValue();
	result.ay = raw.ay / pLinkUpLabelContainer_->pAccelerometerScaleLabel->getValue();
	result.az = raw.az / pLinkUpLabelContainer_->pAccelerometerScaleLabel->getValue();

	result.temperature = raw.temperature / pLinkUpLabelContainer_->pTemperatureScaleLabel->getValue() + pLinkUpLabelContainer_->pTemperatureOffsetLabel->getValue();

	return result;
}

void  ProgressingModule::doWork()
{
	int i = 0;
	while (bIsRunning_)
	{

		FramePackage* pFramePackage = pInputModule_->next();
		OutputPackage* pOutputPackage = pOutputModule_->nextFreeOutputPackage();
		pOutputPackage->pFramePackage = pFramePackage;

		pOutputPackage->imuData = convertImu(pOutputPackage->pFramePackage->imu);

#ifdef WITH_DSO

		if (pFramePackage->imu.cam)
		{
			ldso::MinimalImageB minImg((int)pFramePackage->image.cols, (int)pFramePackage->image.rows, (unsigned char*)pFramePackage->image.data);
			ldso::ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, pFramePackage->exposureTime, pOutputPackage->imuData.timestamp, 1.0f);
			fullSystem->addActiveFrame(undistImg, frameID++);

			delete undistImg;
		}

		if (fullSystem->isLost || ldso::setting_fullResetRequested)
		{
			ldso::setting_fullResetRequested = false;

			shared_ptr<ORBVocabulary> voc(new ORBVocabulary());
			//voc->load(vocFile);
			//delete fullSystem;

			fullSystem = new ldso::FullSystem(voc);
			fullSystem->linearizeOperation = singleThread;

			//fullSystem->setViewer(viewer);
			fullSystem->setViewer(std::shared_ptr<ldso::OutputWrapper>(this));
			//viewer->reset();
			this->reset();
			if (undistorter->photometricUndist != 0)
				fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
		}


#endif //WITH_DSO

		pOutputModule_->writeOut(pOutputPackage);

	}
}

#ifdef WITH_DSO
void ProgressingModule::publishKeyframes(std::vector<shared_ptr<Frame>> &frames, bool final, shared_ptr<CalibHessian> HCalib)
{
	for (shared_ptr<Frame> frame : frames)
	{
		if (frame->frameHessian && frame->frameHessian->flaggedForMarginalization)
		{
			Eigen::Vector3d translation = frame->getPoseOpti().translation();
			Eigen::Quaterniond rotation = frame->getPoseOpti().quaternion();

			SlamPublishPackage* pSlamPublishPackage = new SlamPublishPackage();
			pSlamPublishPackage->publishType = SlamPublishType::KEYFRAME_WITH_POINTS;

			pSlamPublishPackage->frame.id = frame->id;
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

void ProgressingModule::publishCamPose(shared_ptr<Frame> frame, shared_ptr<CalibHessian> HCalib)
{
	Eigen::Vector3d translation = frame->getPose().translation();
	Eigen::Quaterniond rotation = frame->getPose().unit_quaternion();

	SlamPublishPackage* pSlamPublishPackage = new SlamPublishPackage();
	pSlamPublishPackage->publishType = SlamPublishType::FRAME;

	pSlamPublishPackage->frame.id = frame->id;
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
	pSlamPublishPackage->publishType = SlamPublishType::RESET;

	pOutputModule_->writeOut(pSlamPublishPackage);
}

void ProgressingModule::refreshAll()
{

}
#endif //WITH_DSO
