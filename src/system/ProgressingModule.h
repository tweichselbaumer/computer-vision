#ifndef _PROGRESSING_MODULE_h
#define _PROGRESSING_MODULE_h

#include "InputModule.h"
#include "OutputModule.h"

#ifdef WITH_DSO
	#include "util/settings.h"
	#include "FullSystem/FullSystem.h"
	#include "util/Undistort.h"
	#include "IOWrapper/OutputWrapper/SampleOutputWrapper.h"
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
};

#endif //_PROGRESSING_MODULE_h