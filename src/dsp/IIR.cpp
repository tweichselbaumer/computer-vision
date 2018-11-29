#include "IIR.h"

IIR::IIR(double *pA, double *pB, int nA, int nB)
{
	if (nA >= 1 && nB >= 1 && nA + 1 >= nB)
	{
		isValid_ = true;
		pConvW_ = new Convolution(pB, nB);
		pConvY_ = new Convolution(pA, nA);
		yLast_ = 0;
	}
}

IIR::~IIR()
{
	delete pConvW_;
	delete pConvY_;
}

double IIR::next(double x)
{
	if (isValid_)
	{
		double w = pConvW_->next(x);
		yLast_ = pConvY_->next(yLast_) + w;
		return yLast_;
	}
	else
	{
		return x;
	}
}