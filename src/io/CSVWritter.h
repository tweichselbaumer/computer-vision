#ifndef _CSV_WRITTER_H
#define _CSV_WRITTER_H

#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <boost/algorithm/string/join.hpp>

using namespace std;

class CSVWritter
{
public:
	CSVWritter(vector<string> headers, string filename);
	void writeValues(long timestamp, double* pValues, uint16_t nSize);
	void open();
	void close();
private:
	ofstream file_;
	string filename_;
	vector<string> headers_;
};
#endif
