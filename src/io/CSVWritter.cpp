#include "CSVWritter.h"

CSVWritter::CSVWritter(vector<string> headers, string filename)
{
	headers_ = headers;
	filename_ = filename;
}

void CSVWritter::writeValues(double* pValues, uint16_t nSize)
{
	for (int i = 0; i < nSize; i++)
	{
		file_ << pValues[i];
		if (i < nSize - 1)
		{
			file_ << ", ";
		}
	}
	file_ << "\n";
}

void CSVWritter::open()
{
	file_.open(filename_, ios_base::out | ios_base::trunc);
	file_ << boost::algorithm::join(headers_, ", ");
	file_ << "\n";
}

void CSVWritter::close()
{
	file_.close();
}