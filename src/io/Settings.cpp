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

		if (j["imu_parameter"]["accelerometer_walk"].is_number())
		{
			imu_parameter.accelerometer_walk = j["imu_parameter"]["accelerometer_walk"].get<double>();
		}
		if (j["imu_parameter"]["accelerometer_noise"].is_number())
		{
			imu_parameter.accelerometer_noise = j["imu_parameter"]["accelerometer_noise"].get<double>();
		}
		if (j["imu_parameter"]["gyroscope_noise"].is_number())
		{
			imu_parameter.gyroscope_noise = j["imu_parameter"]["gyroscope_noise"].get<double>();
		}
		if (j["imu_parameter"]["gyroscope_walk"].is_number())
		{
			imu_parameter.gyroscope_walk = j["imu_parameter"]["gyroscope_walk"].get<double>();
		}

		if (j["record_remote"].is_boolean())
		{
			recordRemote = j["record_remote"].get<bool>();
		}
		if (j["reproducible_execution"].is_boolean())
		{
			reproducibleExecution = j["reproducible_execution"].get<bool>();
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

		imu_calibration.T_cam_imu = (double*)calloc(4 * 4, sizeof(double));
		if (j["imu_calibration"]["T_cam_imu"].is_array())
		{
			int i = 0;
			for (json::iterator it = j["imu_calibration"]["T_cam_imu"].begin(); it != j["imu_calibration"]["T_cam_imu"].end(); ++it)
			{
				imu_calibration.T_cam_imu[i++] = (double)*it;
			}
		}

		imu_calibration.R_acc_imu = (double*)calloc(3 * 3, sizeof(double));
		if (j["imu_calibration"]["R_acc_imu"].is_array())
		{
			int i = 0;
			for (json::iterator it = j["imu_calibration"]["R_acc_imu"].begin(); it != j["imu_calibration"]["R_acc_imu"].end(); ++it)
			{
				imu_calibration.R_acc_imu[i++] = (double)*it;
			}
		}

		imu_calibration.M_inv_acc = (double*)calloc(3 * 3, sizeof(double));
		if (j["imu_calibration"]["M_inv_acc"].is_array())
		{
			int i = 0;
			for (json::iterator it = j["imu_calibration"]["M_inv_acc"].begin(); it != j["imu_calibration"]["M_inv_acc"].end(); ++it)
			{
				imu_calibration.M_inv_acc[i++] = (double)*it;
			}
		}

		imu_calibration.M_inv_gyro = (double*)calloc(3 * 3, sizeof(double));
		if (j["imu_calibration"]["M_inv_gyro"].is_array())
		{
			int i = 0;
			for (json::iterator it = j["imu_calibration"]["M_inv_gyro"].begin(); it != j["imu_calibration"]["M_inv_gyro"].end(); ++it)
			{
				imu_calibration.M_inv_gyro[i++] = (double)*it;
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

	j["imu_parameter"]["gyroscope_noise"] = imu_parameter.gyroscope_noise;
	j["imu_parameter"]["gyroscope_walk"] = imu_parameter.gyroscope_walk;
	j["imu_parameter"]["accelerometer_noise"] = imu_parameter.accelerometer_noise;
	j["imu_parameter"]["accelerometer_walk"] = imu_parameter.accelerometer_walk;

	j["record_remote"] = recordRemote;
	j["reproducible_execution"] = reproducibleExecution;

	for (int i = 0; i < imu_filter_paramerter.nA; i++)
	{
		j["imu_filter_paramerter"]["a"].push_back(imu_filter_paramerter.pA[i]);
	}

	for (int i = 0; i < imu_filter_paramerter.nB; i++)
	{
		j["imu_filter_paramerter"]["b"].push_back(imu_filter_paramerter.pB[i]);
	}

	for (int i = 0; i < 4 * 4; i++)
	{
		j["imu_calibration"]["T_cam_imu"].push_back(imu_calibration.T_cam_imu[i]);
	}

	for (int i = 0; i < 3 * 3; i++)
	{
		j["imu_calibration"]["R_acc_imu"].push_back(imu_calibration.R_acc_imu[i]);
	}

	for (int i = 0; i < 3 * 3; i++)
	{
		j["imu_calibration"]["M_inv_gyro"].push_back(imu_calibration.M_inv_gyro[i]);
	}

	for (int i = 0; i < 3 * 3; i++)
	{
		j["imu_calibration"]["M_inv_acc"].push_back(imu_calibration.M_inv_acc[i]);
	}

	o << j << std::endl;
}