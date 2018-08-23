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

void  ProgressingModule::doWork()
{
	while (bIsRunning_)
	{
		FramePackage* pFramePackage = pInputModule_->next();
		OutputPackage* pOutputPackage = pOutputModule_->nextFreeOutputPackage();
		pOutputPackage->pFramePackage = pFramePackage;

		//TODO: PROGRESS

		pOutputModule_->writeOut(pOutputPackage);
	}
}
