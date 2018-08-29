#include "Settings.h"

Settings::Settings(string filename)
{
	filename_ = filename;
}

void Settings::load()
{
	if (boost::filesystem::exists(filename_))
	{
		std::ifstream i(filename_);
		json j;
		i >> j;

		if (j["imu_parameter"]["accelerometer_scale"].is_number())
		{
			imu_parameter.accelerometer_scale = j["imu_parameter"]["accelerometer_scale"].get<double>();
		}
		if (j["imu_parameter"]["gyroscope_scale"].is_number())
		{
			imu_parameter.gyroscope_scale = j["imu_parameter"]["gyroscope_scale"].get<double>();
		}
		if (j["imu_parameter"]["temperature_scale"].is_number())
		{
			imu_parameter.temperature_scale = j["imu_parameter"]["temperature_scale"].get<double>();
		}
		if (j["imu_parameter"]["temperature_offset"].is_number())
		{
			imu_parameter.temperature_offset = j["imu_parameter"]["temperature_offset"].get<double>();
		}
		if (j["record_remote"].is_boolean())
		{
			recordRemote = j["record_remote"].get<bool>();
		}
	}
}

void Settings::save()
{
	std::ofstream o(filename_);

	json j;

	j["imu_parameter"]["accelerometer_scale"] = imu_parameter.accelerometer_scale;
	j["imu_parameter"]["gyroscope_scale"] = imu_parameter.gyroscope_scale;
	j["imu_parameter"]["temperature_scale"] = imu_parameter.temperature_scale;
	j["imu_parameter"]["temperature_offset"] = imu_parameter.temperature_offset;
	j["record_remote"] = recordRemote;

	o << j << std::endl;
}