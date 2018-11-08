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

	setting_desiredImmatureDensity = 600;
	setting_desiredPointDensity = 800;
	setting_minFrames = 4;
	setting_maxFrames = 6;
	setting_maxOptIterations = 4;
	setting_minOptIterations = 1;

	ldso::setting_pointSelection = 0;

	ldso::benchmarkSetting_width = 256;
	ldso::benchmarkSetting_height = 256;

	ldso::setting_photometricCalibration = 2;
	ldso::setting_affineOptModeA = 1;
	ldso::setting_affineOptModeB = 1;
	ldso::setting_enableLoopClosing = false;

	undistorter = ldso::Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

	ldso::internal::setGlobalCalib(
		(int)undistorter->getSize()[0],
		(int)undistorter->getSize()[1],
		undistorter->getK().cast<float>());

	shared_ptr<ORBVocabulary> voc(new ORBVocabulary());
	//voc->load(vocFile);

	fullSystem = new ldso::FullSystem(voc);
	fullSystem->linearizeOperation = false;

	viewer = shared_ptr<PangolinDSOViewer>(new PangolinDSOViewer(wG[0], hG[0], true));
	fullSystem->setViewer(viewer);

	if (undistorter->photometricUndist != 0)
		fullSystem->setGammaFunction(undistorter->photometricUndist->getG());

#endif WITH_DSO

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

			fullSystem = new ldso::FullSystem(voc);
			fullSystem->linearizeOperation = false;

			fullSystem->setViewer(viewer);
			viewer->reset();
			if (undistorter->photometricUndist != 0)
				fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
		}


#endif //WITH_DSO

		pOutputModule_->writeOut(pOutputPackage);
	}
}
