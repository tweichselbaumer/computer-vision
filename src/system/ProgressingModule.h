#ifndef _PROGRESSING_MODULE_h
#define _PROGRESSING_MODULE_h

#include "InputModule.h"
#include "OutputModule.h"

#ifdef WITH_DSO
#include "Settings.h"
#include "frontend/FullSystem.h"
#include "frontend/Undistort.h"

//#ifdef __linux

//#endif //__linux
#endif //WITH_DSO

#include <boost/thread.hpp>

class ProgressingModule
{
public:
	ProgressingModule(InputModule* pInputModule, OutputModule* pOutputModule, LinkUpLabelContainer* pLinkUpLabelContainer);
	void start();
	void stop();
private:
	boost::thread thread_;

	void doWork();
	ImuData convertImu(RawImuData imuData);

	bool bIsRunning_ = false;

	InputModule* pInputModule_ = 0;
	OutputModule* pOutputModule_ = 0;
	LinkUpLabelContainer* pLinkUpLabelContainer_;

#ifdef WITH_DSO
	ldso::FullSystem* fullSystem = 0;
	ldso::Undistort* undistorter = 0;
	int frameID = 0;

	std::string calib = "camera.txt";
	std::string vignetteFile = "vignette.png";
	std::string gammaFile = "pcalib.txt";
#endif //WITH_DSO
};

#endif //_PROGRESSING_MODULE_h