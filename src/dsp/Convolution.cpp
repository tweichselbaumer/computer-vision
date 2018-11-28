#include "Convolution.h"

Convolution::Convolution(double *pH, int nSize)
{
	this->nSize_ = nSize;
	this->pH_ = (double*)calloc(nSize, sizeof(double) * 2);
	this->pX_ = (double*)calloc(nSize, sizeof(double) * 2);
	memcpy(this->pH_, pH, nSize);
}

Convolution::~Convolution()
{
	free(this->pH_);
	free(this->pX_);
}

double Convolution::next(double x)
{
	if (nPtr_ >= nSize_)
		nPtr_ = 0;

	pX_[nPtr_] = x;
	pX_[nPtr_ + nSize_] = x;

	double sum = 0;
	int j = nPtr_ + nSize_;
	for (int i = 0; i < nSize_; i++)
	{
		sum = sum + pH_[i] * pX_[j];
		j--;
	}
	nPtr_++;
	return sum;
}