#ifndef _CONVOLUTION_h
#define _CONVOLUTION_h

#include "Platform.h"

class Convolution {
private:
	int nSize_;
	double* pX_;
	double* pH_;
	int nPtr_;
public:
	Convolution(double *pH, int nSize);
	~Convolution();
	double next(double x);
};

#endif