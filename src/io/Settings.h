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
	struct {
		double accelerometer_scale;
		double gyroscope_scale;
		double temperature_scale;
		double temperature_offset;
	}imu_parameter;
private:
	string filename_;
};
#endif
