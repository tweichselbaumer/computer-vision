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
	dso::setting_desiredImmatureDensity = 2000;
	dso::setting_desiredPointDensity = 2000;
	dso::setting_minFrames = 5;
	dso::setting_maxFrames = 7;
	dso::setting_maxOptIterations = 9;
	dso::setting_minOptIterations = 1;
	dso::setting_logStuff = false;
	dso::setting_kfGlobalWeight = 1.3;

	dso::benchmarkSetting_width = 256;
	dso::benchmarkSetting_height = 256;

	dso::setting_photometricCalibration = 0;
	dso::setting_affineOptModeA = 1;
	dso::setting_affineOptModeB = 1;

	undistorter = dso::Undistort::getUndistorterForFile(calib, gammaFile, vignetteFile);

	dso::setGlobalCalib(
		(int)undistorter->getSize()[0],
		(int)undistorter->getSize()[1],
		undistorter->getK().cast<float>());


	fullSystem = new dso::FullSystem();
	fullSystem->linearizeOperation = false;

	//#ifdef __linux
	fullSystem->outputWrapper.push_back(new dso::IOWrap::PangolinDSOViewer(
		(int)undistorter->getSize()[0],
		(int)undistorter->getSize()[1]));
	//#endif //__linux

	//fullSystem->outputWrapper.push_back(new dso::IOWrap::SampleOutputWrapper());

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
	delete undistorter;
	delete fullSystem;
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
	bool toggle = false;
	int i = 0;

	while (bIsRunning_)
	{
		FramePackage* pFramePackage = pInputModule_->next();
		OutputPackage* pOutputPackage = pOutputModule_->nextFreeOutputPackage();
		pOutputPackage->pFramePackage = pFramePackage;

		pOutputPackage->imuData = convertImu(pOutputPackage->pFramePackage->imu);

#ifdef WITH_DSO
		toggle = !toggle;
		if (dso::setting_fullResetRequested)
		{
			frameID = 0;
			std::vector<dso::IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
			delete fullSystem;
			for (dso::IOWrap::Output3DWrapper* ow : wraps) ow->reset();
			fullSystem = new dso::FullSystem();
			fullSystem->linearizeOperation = false;
			fullSystem->outputWrapper = wraps;
			if (undistorter->photometricUndist != 0)
				fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
			dso::setting_fullResetRequested = false;
		}

		if (fullSystem->isLost) {
			frameID = 0;
			std::vector<dso::IOWrap::Output3DWrapper*> wraps = fullSystem->outputWrapper;
			delete fullSystem;

			for (dso::IOWrap::Output3DWrapper* ow : wraps) ow->reset();

			fullSystem = new dso::FullSystem();
			if (undistorter->photometricUndist != 0)
				fullSystem->setGammaFunction(undistorter->photometricUndist->getG());
			fullSystem->linearizeOperation = false;


			fullSystem->outputWrapper = wraps;

			dso::setting_fullResetRequested = false;

		}
		if (pFramePackage->imu.cam )
		{
			dso::MinimalImageB minImg((int)pFramePackage->image.cols, (int)pFramePackage->image.rows, (unsigned char*)pFramePackage->image.data);
			dso::ImageAndExposure* undistImg = undistorter->undistort<unsigned char>(&minImg, 1, 0, 1.0f);
			fullSystem->addActiveFrame(undistImg, frameID++);

			delete undistImg;
		}
	

#endif //WITH_DSO

		pOutputModule_->writeOut(pOutputPackage);
	}
}
