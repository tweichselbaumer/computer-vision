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
		double accelerometer_noise;
		double accelerometer_walk;
		double gyroscope_noise;
		double gyroscope_walk;
	}imu_parameter;
	struct {
		double* T_cam_imu;
		double* R_acc_imu;
		double* M_inv_gyro;
		double* M_inv_acc;
	}imu_calibration;
	struct {
		double* pA;
		double* pB;
		int nA=0;
		int nB=0;
	}imu_filter_paramerter;
	bool recordRemote;
	bool reproducibleExecution;
	struct {
		bool publish_keyframe_immediat = false;
	} static static_settings;
private:
	string filename_;
};
#endif
