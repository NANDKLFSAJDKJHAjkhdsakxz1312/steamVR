#include <string>
#include "vr_devices.h"  // 确保包含 vr_controller_data 的定义

struct Pose3D
{
	Eigen::Vector3d position;
	Eigen::Quaterniond orientation;  // 或叫 rotation

};

struct TrackerData
{
	std::string serial_number;
	int id;
	Pose3D pose;
};


