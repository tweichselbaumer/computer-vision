#include "IIR.h"

IIR::IIR(double *pA, double *pB, int nSize)
{
	nSize_ = nSize;
	if (nSize > 1)
	{
		pConvW_ = new Convolution(pB, nSize);
		pConvY_ = new Convolution(pA, nSize - 1);
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
	if (nSize_ > 1)
	{
		double w = pConvW_->next(x);
		yLast_ = pConvW_->next(yLast_ + w);
		return yLast_;
	}
	else
	{
		return x;
	}
}