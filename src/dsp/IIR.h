#ifndef _IIR_h
#define _IIR_h

#include "Platform.h"
#include "Convolution.h"

class IIR {
private:
	Convolution* pConvW_;
	Convolution* pConvY_;
	double yLast_ = 0;
	int nSize_ = 0;
public:
	IIR(double *pA, double *pB, int nSize);
	~IIR();
	double next(double x);
};

#endif