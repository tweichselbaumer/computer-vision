#ifndef _OUTPUT_MODULE_h
#define _OUTPUT_MODULE_h

#include "OutputPackage.h"
#include "InputModule.h"
#include "SlamPublishPackage.h"

#include <boost/lockfree/queue.hpp>
#include <boost/thread.hpp>
#include <boost/format.hpp>

#include <sstream>

#include "../io/CSVWritter.h"
#include <time.h> 

class OutputModule
{
public:
	OutputModule(InputModule* pInputModule, LinkUpLabelContainer* pLinkUpLabelContainer);
	void start();
	void stop();
	void writeOut(OutputPackage* pResult);
	void writeOut(SlamPublishPackage* pResult);
	void writeOut(SlamStatusUpdate pResult);
	OutputPackage* nextFreeOutputPackage();
private:
	enum { queueSize = 200 };

	boost::thread thread_;
	boost::thread threadPublish_;
	boost::lockfree::queue<OutputPackage*>* pInQueue_;
	boost::lockfree::queue<OutputPackage*>* pFreeQueue_;

	boost::lockfree::queue<SlamPublishPackage*>* pInSlamPublishQueue_;
	boost::lockfree::queue<SlamStatusUpdate>* pInSlamStatusQueue_;


	void doWork();
	void doWorkPublish();

	bool bIsRunning_ = false;
	bool bIsRecording_ = false;

	void startRecording();

	InputModule* pInputModule_ = 0;
	LinkUpLabelContainer* pLinkUpLabelContainer_;
	CSVWritter* pCSVWritter_;
};

#endif