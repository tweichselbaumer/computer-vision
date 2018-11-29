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

		if (j["imu_filter_paramerter"]["a"].is_array())
		{
			int i = 0;
			for (json::iterator it = j["imu_filter_paramerter"]["a"].begin(); it != j["imu_filter_paramerter"]["a"].end(); ++it)
			{
				i++;
			}
			imu_filter_paramerter.nA = i;
			imu_filter_paramerter.pA = (double*)calloc(imu_filter_paramerter.nA, sizeof(double));
			i = 0;
			for (json::iterator it = j["imu_filter_paramerter"]["a"].begin(); it != j["imu_filter_paramerter"]["a"].end(); ++it)
			{
				imu_filter_paramerter.pA[i++] = (double)*it;
			}
		}
		if (j["imu_filter_paramerter"]["b"].is_array())
		{
			int i = 0;
			for (json::iterator it = j["imu_filter_paramerter"]["b"].begin(); it != j["imu_filter_paramerter"]["b"].end(); ++it)
			{
				i++;
			}
			imu_filter_paramerter.nB = i;
			imu_filter_paramerter.pB = (double*)calloc(imu_filter_paramerter.nB, sizeof(double));
			i = 0;
			for (json::iterator it = j["imu_filter_paramerter"]["b"].begin(); it != j["imu_filter_paramerter"]["b"].end(); ++it)
			{
				imu_filter_paramerter.pB[i++] = (double)*it;
			}
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

	for (int i = 0; i < imu_filter_paramerter.nA; i++)
	{
		j["imu_filter_paramerter"]["a"].push_back(imu_filter_paramerter.pA[i]);
	}

	for (int i = 0; i < imu_filter_paramerter.nB; i++)
	{
		j["imu_filter_paramerter"]["b"].push_back(imu_filter_paramerter.pB[i]);
	}

	o << j << std::endl;
}