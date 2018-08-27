#ifndef _OUTPUT_MODULE_h
#define _OUTPUT_MODULE_h

#include "OutputPackage.h"
#include "InputModule.h"

#include <boost/lockfree/queue.hpp>
#include <boost/thread.hpp>

#include "../io/CSVWritter.h"

class OutputModule
{
public:
	OutputModule(InputModule* pInputModule, LinkUpLabelContainer* pLinkUpLabelContainer);
	void start();
	void stop();
	void writeOut(OutputPackage* pResult);
	OutputPackage* nextFreeOutputPackage();
private:
	enum { queueSize = 1000 };

	boost::thread thread_;
	boost::lockfree::queue<OutputPackage*>* pInQueue_;
	boost::lockfree::queue<OutputPackage*>* pFreeQueue_;

	void doWork();

	bool bIsRunning_ = false;

	InputModule* pInputModule_ = 0;
	LinkUpLabelContainer* pLinkUpLabelContainer_;
	CSVWritter* pCSVWritter_;
};

#endif