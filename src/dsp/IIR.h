#ifndef _IIR_h
#define _IIR_h

#include "Platform.h"
#include "Convolution.h"

class IIR {
private:
	Convolution* pConvW_;
	Convolution* pConvY_;
	double yLast_ = 0;
	bool isValid_ = false;
public:
	IIR(double *pA, double *pB, int nA, int nB);
	~IIR();
	double next(double x);
};

#endif