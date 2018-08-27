#include "ProgressingModule.h"

ProgressingModule::ProgressingModule(InputModule* pInputModule, OutputModule* pOutputModule, LinkUpLabelContainer* pLinkUpLabelContainer)
{
	pInputModule_ = pInputModule;
	pOutputModule_ = pOutputModule;
	pLinkUpLabelContainer_ = pLinkUpLabelContainer;
}

void  ProgressingModule::start()
{
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

	result.timestamp = (raw.timestamp_us + multi * pow(2, 32)) / (1000 * 1000);

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
	while (bIsRunning_)
	{
		FramePackage* pFramePackage = pInputModule_->next();
		OutputPackage* pOutputPackage = pOutputModule_->nextFreeOutputPackage();
		pOutputPackage->pFramePackage = pFramePackage;

		pOutputPackage->imuData = convertImu(pOutputPackage->pFramePackage->imu);

		//TODO: PROGRESS

		pOutputModule_->writeOut(pOutputPackage);
	}
}
