#ifndef _SETTINGS_H
#define _SETTINGS_H

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

#include "../../thirdParty/nlohmann/json.hpp"

#include <boost/filesystem.hpp>

using json = nlohmann::json;
using namespace std;

class Settings
{
public:
	Settings(string filename);
	void load();
	void save();
	struct imu_parameter {
		double acc_scale;
	};
private:
	string filename_;
};
#endif
