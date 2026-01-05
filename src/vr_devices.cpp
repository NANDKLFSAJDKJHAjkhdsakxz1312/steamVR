#include "vr_devices.h"
#include <thread>  // 补充sleep_for所需头文件



// ------------------------------
// vr_controller_data 成员函数实现
// ------------------------------
Eigen::Vector3d vr_controller_data::getPosition() const {
	return Eigen::Vector3d(pose_x, pose_y, pose_z);
}

Eigen::Quaterniond vr_controller_data::getQuaternion() const {
	// 注意：四元数构造顺序 (w, x, y, z) 需匹配存储顺序
	return Eigen::Quaterniond(pose_qw, pose_qx, pose_qy, pose_qz);
}

void vr_controller_data::setPosition(const Eigen::Vector3d& pos) {
	pose_x = pos.x();
	pose_y = pos.y();
	pose_z = pos.z();
}

void vr_controller_data::setQuaternion(const Eigen::Quaterniond& q) {
	pose_qx = q.x();
	pose_qy = q.y();
	pose_qz = q.z();
	pose_qw = q.w();
}

void vr_controller_data::reset() {
	pose_x = pose_y = pose_z = 0.0;
	pose_qx = pose_qy = pose_qz = 0.0;
	pose_qw = 1.0;

	menu_button = trigger_button = trackpad_touch = trackpad_button = grip_button = false;
	trackpad_x = trackpad_y = trigger = 0.0;
	role = 1;
	time = "";
}

// ------------------------------
// 日志函数实现 (inline 需和头文件声明匹配)
// ------------------------------
inline void logMessage(LogLevel level, const std::string& message) {
	std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
	std::time_t time_t = std::chrono::system_clock::to_time_t(now);
	std::chrono::milliseconds ms = std::chrono::duration_cast<std::chrono::milliseconds>(
		now.time_since_epoch()) % 1000;

	std::ostringstream oss;
	oss << std::put_time(std::localtime(&time_t), "%H:%M:%S");
	oss << '.' << std::setfill('0') << std::setw(3) << ms.count();

	switch (level) {
	case Info:
		std::cout << "[" << oss.str() << "][INFO]  " << message << std::endl;
		break;
	case Debug:
		std::cout << "[" << oss.str() << "][DEBUG] " << message << std::endl;
		break;
	case Warning:
		std::cerr << "[" << oss.str() << "][WARN]  " << message << std::endl;
		break;
	case Error:
		std::cerr << "[" << oss.str() << "][ERROR] " << message << std::endl;
		break;
	}
}

// ------------------------------
// vr_utils 静态成员函数实现
// ------------------------------
std::string vr_utils::getCurrentTimeWithMilliseconds() {
	std::chrono::system_clock::time_point now = std::chrono::system_clock::now();
	std::time_t nowAsTimeT = std::chrono::system_clock::to_time_t(now);
	std::chrono::milliseconds nowMs = std::chrono::duration_cast<std::chrono::milliseconds>(
		now.time_since_epoch()) % 1000;

	std::stringstream ss;
	ss << std::put_time(std::localtime(&nowAsTimeT), "%Y-%m-%d %H:%M:%S");
	ss << '.' << std::setfill('0') << std::setw(3) << nowMs.count();
	return ss.str();
}

bool vr_utils::deviceIsConnected(vr::IVRSystem* pHMD, vr::TrackedDeviceIndex_t unDeviceIndex) {
	if (!pHMD) return false;
	return pHMD->IsTrackedDeviceConnected(unDeviceIndex);
}

bool vr_utils::controllerIsConnected(vr::IVRSystem* pHMD, vr::TrackedDeviceIndex_t unDeviceIndex) {
	if (!pHMD) return false;
	return pHMD->GetTrackedDeviceClass(unDeviceIndex) == vr::TrackedDeviceClass_Controller;
}

void vr_utils::deviceConnectionCheck(vr::IVRSystem* pHMD) {
	if (!pHMD) return;

	for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
		if (deviceIsConnected(pHMD, i)) {
			vr::ETrackedDeviceClass trackedDeviceClass = pHMD->GetTrackedDeviceClass(i);
			logMessage(Debug, std::string("[CONNECTED DEVICE ") + std::to_string(i) +
				"]: Class = " + std::to_string(trackedDeviceClass));
		}
	}
}

vr::ETrackedControllerRole vr_utils::controllerRoleCheck(vr::IVRSystem* pHMD, vr::TrackedDeviceIndex_t i) {
	vr::ETrackedControllerRole controllerRole = vr::TrackedControllerRole_Invalid;

	if (!pHMD || !controllerIsConnected(pHMD, i)) {
		return controllerRole;
	}

	controllerRole = pHMD->GetControllerRoleForTrackedDeviceIndex(i);

	if (controllerRole != vr::TrackedControllerRole_Invalid) {
		std::string roleStr;
		switch (controllerRole) {
		case vr::TrackedControllerRole_LeftHand:
			roleStr = "Left Hand";
			break;
		case vr::TrackedControllerRole_RightHand:
			roleStr = "Right Hand";
			break;
		default:
			roleStr = "Unknown (" + std::to_string(controllerRole) + ")";
			break;
		}
		logMessage(Debug, std::string("[CONNECTED CONTROLLER ") + std::to_string(i) +
			"]: Role = " + roleStr);
	}
	return controllerRole;
}

void vr_utils::controllerConnectionCheck(vr::IVRSystem* pHMD) {
	if (!pHMD) return;

	for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
		controllerRoleCheck(pHMD, i);
	}
}

void vr_utils::HapticFeedback(vr::IVRSystem* pHMD, vr::TrackedDeviceIndex_t unControllerDeviceIndex, unsigned short durationMicroSec) {
	if (pHMD) {
		pHMD->TriggerHapticPulse(unControllerDeviceIndex, 0, durationMicroSec);
	}
}

void vr_utils::extractPoseFromMatrix(const vr::HmdMatrix34_t& vrMatrix, vr_controller_data& data) {
	Eigen::Vector3d position = VR2EigenTransforms::getPositionFromVRMatrix(vrMatrix);
	Eigen::Quaterniond quaternion = VR2EigenTransforms::getQuaternionFromVRMatrix(vrMatrix);

	data.setPosition(position);
	data.setQuaternion(quaternion);
}

vr_controller_data vr_utils::calculateRelativePose(const vr_controller_data& initial, const vr_controller_data& current) {
	vr_controller_data relativePose;
	relativePose.role = current.role;

	// 计算相对位置和姿态
	Eigen::Vector3d initialPos = initial.getPosition();
	Eigen::Quaterniond initialQuat = initial.getQuaternion();
	Eigen::Vector3d currentPos = current.getPosition();
	Eigen::Quaterniond currentQuat = current.getQuaternion();

	Eigen::Vector3d relativePos = currentPos - initialPos;
	Eigen::Quaterniond relativeQuat = initialQuat.inverse() * currentQuat;
	Eigen::Vector3d rotatedRelativePos = initialQuat.inverse() * relativePos;

	// 设置相对位姿
	relativePose.setPosition(rotatedRelativePos);
	relativePose.setQuaternion(relativeQuat);

	// 复制按钮/状态数据
	relativePose.menu_button = current.menu_button;
	relativePose.trigger_button = current.trigger_button;
	relativePose.trackpad_touch = current.trackpad_touch;
	relativePose.trackpad_button = current.trackpad_button;
	relativePose.grip_button = current.grip_button;
	relativePose.trackpad_x = current.trackpad_x;
	relativePose.trackpad_y = current.trackpad_y;
	relativePose.trigger = current.trigger;
	relativePose.time = current.time;

	return relativePose;
}

vr_controller_data vr_utils::filterPose(const vr_controller_data& current, const vr_controller_data& previous, double alpha) {
	vr_controller_data filtered = current;

	// 位置滤波 (指数移动平均)
	Eigen::Vector3d filteredPos = VR2EigenTransforms::filterPosition(
		current.getPosition(), previous.getPosition(), alpha);

	// 四元数滤波 (SLERP 球面插值)
	Eigen::Quaterniond filteredQuat = VR2EigenTransforms::filterQuaternion(
		current.getQuaternion(), previous.getQuaternion(), alpha);

	filtered.setPosition(filteredPos);
	filtered.setQuaternion(filteredQuat);

	return filtered;
}

// ------------------------------
// vive_input 类成员函数实现
// ------------------------------
vive_input::vive_input(std::mutex &mutex, std::condition_variable &cv, vr_controller_data &data, double publish_freq)
	: data_mutex(mutex), data_cv(cv), shared_data(data), controllerPublishFrequency(publish_freq) {
	// 初始化VR环境
	if (!initVR()) {
		shutdownVR();
		throw std::runtime_error("Failed to initialize Vive VR system!");
	}

	// 初始化控制器数据
	right_controller_data.reset();
	left_controller_data.reset();
	local_data.reset();
}

vive_input::~vive_input() {
	stopSendThread(); // 新增
	shutdownVR();
}

bool vive_input::initVR() {
	eError = vr::VRInitError_None;
	pHMD = vr::VR_Init(&eError, vr::VRApplication_Background);

	if (eError != vr::VRInitError_None) {
		std::string error_msg = vr::VR_GetVRInitErrorAsEnglishDescription(eError);
		logMessage(Error, std::string("VR Initialization Failed: ") + error_msg);
		return false;
	}

	logMessage(Info, "Vive VR system initialized successfully");
	vr_utils::deviceConnectionCheck(pHMD);
	return true;
}

bool vive_input::shutdownVR() {
	if (pHMD) {
		logMessage(Info, "Shutting down Vive VR system...");
		vr::VR_Shutdown();
		pHMD = nullptr;
	}
	return true;
}

void vive_input::setPublishFrequency(double freq) {
	if (freq > VRInputConfig::MAX_PUBLISH_FREQUENCY || freq < VRInputConfig::MIN_PUBLISH_FREQUENCY) {
		logMessage(Warning, std::string("Invalid publish frequency: ") + std::to_string(freq) +
			" Hz (Range: " + std::to_string(VRInputConfig::MIN_PUBLISH_FREQUENCY) +
			" - " + std::to_string(VRInputConfig::MAX_PUBLISH_FREQUENCY) + " Hz)");
		return;
	}

	controllerPublishFrequency = freq;
	logMessage(Info, std::string("Publish frequency updated to: ") + std::to_string(freq) + " Hz");

	// 重置发布时间初始化标记
	publish_time_initialized[VRInputConfig::RIGHT_CONTROLLER_INDEX] = false;
	publish_time_initialized[VRInputConfig::LEFT_CONTROLLER_INDEX] = false;
}

void vive_input::loadTrackerConfig() {
	try {
		YAML::Node config = YAML::LoadFile("D:\\workspace\\vive_pose2robot_joint_value\\config\\config.yaml");

		if (!config["trackers"]) {
			logMessage(Warning, "Tracker config file has no 'trackers' section!");
			return;
		}

		tracker_data.clear();
		for (const YAML::Node& tracker_node : config["trackers"]) {
			TrackerData tracker;
			tracker.serial_number = tracker_node["serial_number"].as<std::string>();
			tracker.id = tracker_node["id"].as<int>();
			tracker_data.push_back(tracker);

			logMessage(Info, std::string("Loaded tracker config: ID=") + std::to_string(tracker.id) +
				", Serial=" + tracker.serial_number);
		}

		config_loaded = true;
	}
	catch (const YAML::Exception& e) {
		logMessage(Error, std::string("Failed to load tracker config: ") + std::string(e.what()));
	}
}

void vive_input::processControllerButtons(uint32_t deviceIndex, vr::VRControllerState_t& controllerState, int roleIndex) {
	// 重置按钮状态
	local_data.menu_button = false;
	local_data.trigger_button = false;
	local_data.trackpad_button = false;
	local_data.grip_button = false;
	local_data.trackpad_touch = false;
	local_data.trackpad_x = 0.0;
	local_data.trackpad_y = 0.0;

	// 应用菜单按钮
	if ((1LL << vr::k_EButton_ApplicationMenu) & controllerState.ulButtonPressed) {
		logMessage(Debug, std::string("Controller ") + std::to_string(roleIndex) + ": Application Menu pressed");
		local_data.menu_button = true;
		first_run[roleIndex] = true;
		vr_utils::HapticFeedback(pHMD, deviceIndex, VRInputConfig::HAPTIC_FEEDBACK_DURATION);
	}

	// 扳机按钮
	if ((1LL << vr::k_EButton_SteamVR_Trigger) & controllerState.ulButtonPressed) {
		logMessage(Debug, std::string("Controller ") + std::to_string(roleIndex) + ": Trigger pressed");
		local_data.trigger_button = true;
	}

	// 触控板按钮
	if ((1LL << vr::k_EButton_SteamVR_Touchpad) & controllerState.ulButtonPressed) {
		logMessage(Debug, std::string("Controller ") + std::to_string(roleIndex) + ": Trackpad button pressed");
		local_data.trackpad_button = true;
		vr_utils::HapticFeedback(pHMD, deviceIndex, VRInputConfig::HAPTIC_FEEDBACK_DURATION);
	}

	// 握把按钮
	if ((1LL << vr::k_EButton_Grip) & controllerState.ulButtonPressed) {
		logMessage(Debug, std::string("Controller ") + std::to_string(roleIndex) + ": Grip pressed");
		local_data.grip_button = true;
	}

	// 触控板触摸
	if ((1LL << vr::k_EButton_SteamVR_Touchpad) & controllerState.ulButtonTouched) {
		local_data.trackpad_touch = true;
		local_data.trackpad_x = controllerState.rAxis[0].x;
		local_data.trackpad_y = controllerState.rAxis[0].y;
		logMessage(Debug, std::string("Controller ") + std::to_string(roleIndex) +
			": Trackpad touched (X=" + std::to_string(local_data.trackpad_x) +
			", Y=" + std::to_string(local_data.trackpad_y) + ")");
	}
}

void vive_input::processTriggerFeedback(float triggerValue, uint32_t deviceIndex, int roleIndex) {
	static int previousStep[VRInputConfig::MAX_CONTROLLERS] = { -1, -1 };

	const float stepSize = 1.0f / VRInputConfig::TRIGGER_FEEDBACK_STEPS;
	int currentStep = static_cast<int>(triggerValue / stepSize);

	local_data.trigger = triggerValue;
	logMessage(Debug, std::string("Controller ") + std::to_string(roleIndex) +
		": Trigger value = " + std::to_string(triggerValue));

	// 扳机值变化时触发震动反馈
	if (currentStep != previousStep[roleIndex]) {
		int vibrationDuration = static_cast<int>(triggerValue * VRInputConfig::TRIGGER_HAPTIC_MULTIPLIER);
		vr_utils::HapticFeedback(pHMD, deviceIndex, vibrationDuration);
		previousStep[roleIndex] = currentStep;
	}
}

bool vive_input::validatePositionChange(const Eigen::Vector3d& currentPosition, int roleIndex) {
	std::chrono::steady_clock::time_point current_time = std::chrono::steady_clock::now();

	// 首次运行不校验
	if (first_run[roleIndex]) {
		first_run[roleIndex] = false;
		prev_position[roleIndex] = currentPosition;
		prev_time[roleIndex] = current_time;
		return true;
	}

	// 计算位移和速度
	std::chrono::duration<float> time_diff = current_time - prev_time[roleIndex];
	float delta_time = time_diff.count();
	Eigen::Vector3d position_change = currentPosition - prev_position[roleIndex];
	float delta_distance = position_change.norm();
	float velocity = delta_distance / delta_time;

	logMessage(Debug, std::string("Controller ") + std::to_string(roleIndex) +
		": Delta Distance = " + std::to_string(delta_distance) +
		", Velocity = " + std::to_string(velocity) + " m/s");

	// 校验位移合理性
	if (delta_distance > VRInputConfig::POSITION_THRESHOLD) {
		logMessage(Warning, std::string("Controller ") + std::to_string(roleIndex) +
			": Unreasonable position change (" + std::to_string(delta_distance) +
			"m > " + std::to_string(VRInputConfig::POSITION_THRESHOLD) + "m)");
		vr_utils::HapticFeedback(pHMD, roleIndex, VRInputConfig::WARNING_HAPTIC_DURATION);
		return false;
	}

	// 更新历史数据
	prev_position[roleIndex] = currentPosition;
	prev_time[roleIndex] = current_time;
	return true;
}

void vive_input::runVR() {
	logMessage(Info, std::string("Starting Vive VR main loop (Publish Frequency: ") +
		std::to_string(controllerPublishFrequency) + " Hz)");
	// 新增：启动发送线程
	startSendThread();
	std::chrono::steady_clock::time_point lastLogTime = std::chrono::steady_clock::now();
	double publish_interval_ms = 1000.0 / controllerPublishFrequency;
	std::chrono::milliseconds publish_interval = std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::duration<double, std::milli>(publish_interval_ms));

	while (true) {
		// 1. 加载Tracker配置（仅首次）
		if (!config_loaded) {
			loadTrackerConfig();
		}

		// 2. 重置控制器检测状态
		controller_detected[VRInputConfig::RIGHT_CONTROLLER_INDEX] = false;
		controller_detected[VRInputConfig::LEFT_CONTROLLER_INDEX] = false;
		right_controller_updated = false;
		left_controller_updated = false;

		// 3. 获取所有设备位姿
		pHMD->GetDeviceToAbsoluteTrackingPose(vr::TrackingUniverseStanding, 0,
			trackedDevicePose, vr::k_unMaxTrackedDeviceCount);

		// 4. 遍历所有设备
		for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
			logMessage(Debug, std::string("[ALL DEVICE] 索引i = ") + std::to_string(i));
			// 跳过未连接/位姿无效的设备
			if (!trackedDevicePose[i].bDeviceIsConnected ||
				!trackedDevicePose[i].bPoseIsValid ||
				trackedDevicePose[i].eTrackingResult != vr::TrackingResult_Running_OK) {
				continue;
			}
			// ========== 新增：打印有效设备索引 ==========
			logMessage(Debug, std::string("[VALID DEVICE] 索引i = ") + std::to_string(i));

			// 4.1 处理Tracker设备
			if (pHMD->GetTrackedDeviceClass(i) == vr::TrackedDeviceClass_GenericTracker) {
				// ========== 新增：打印Tracker设备索引 ==========
				logMessage(Debug, std::string("[TRACKER DEVICE] 索引i = ") + std::to_string(i));
				// 获取Tracker序列号
				vr::TrackedPropertyError error = vr::TrackedProp_Success;
				uint32_t len = pHMD->GetStringTrackedDeviceProperty(
					i, vr::Prop_SerialNumber_String, nullptr, 0, &error);

				std::vector<char> buffer(len);
				pHMD->GetStringTrackedDeviceProperty(
					i, vr::Prop_SerialNumber_String, buffer.data(), len, &error);
				std::string serial(buffer.data());

				logMessage(Debug, std::string("Detected Tracker: Serial = ") + serial);

				// 匹配配置中的Tracker
				for (TrackerData& tracker : tracker_data) {
					if (tracker.serial_number == serial) {
						// 更新Tracker位姿
						vr::HmdMatrix34_t mat = trackedDevicePose[i].mDeviceToAbsoluteTracking;
						tracker.pose.position = VR2EigenTransforms::getPositionFromVRMatrix(mat);
						tracker.pose.orientation = VR2EigenTransforms::getQuaternionFromVRMatrix(mat);

						logMessage(Debug, std::string("Tracker ID=") + std::to_string(tracker.id) +
							": Pos(" + std::to_string(tracker.pose.position.x()) + "," +
							std::to_string(tracker.pose.position.y()) + "," +
							std::to_string(tracker.pose.position.z()) + "), " +
							"Quat(" + std::to_string(tracker.pose.orientation.x()) + "," +
							std::to_string(tracker.pose.orientation.y()) + "," +
							std::to_string(tracker.pose.orientation.z()) + "," +
							std::to_string(tracker.pose.orientation.w()) + ")");
						// 新增：将Tracker原始数据入队发送
						DeviceRawData tracker_raw = convertTrackerToRawData(tracker, mat);
						send_queue_.push(std::move(tracker_raw));
						break;
					}
				}
				continue;
			}

			// 4.2 处理控制器设备
			if (!vr_utils::controllerIsConnected(pHMD, i)) {
				continue;
			}

			// 获取控制器角色（左/右）
			vr::ETrackedControllerRole role = vr_utils::controllerRoleCheck(pHMD, i);
			int role_index = -1;
			if (role == vr::TrackedControllerRole_RightHand) {
				role_index = VRInputConfig::RIGHT_CONTROLLER_INDEX;
			}
			else if (role == vr::TrackedControllerRole_LeftHand) {
				role_index = VRInputConfig::LEFT_CONTROLLER_INDEX;
			}
			else {
				continue; // 未知角色的控制器跳过
			}

			// 标记控制器已检测到
			controller_detected[role_index] = true;

			// 4.3 解析控制器位姿
			vr::HmdMatrix34_t mat = trackedDevicePose[i].mDeviceToAbsoluteTracking;
			Eigen::Vector3d position = VR2EigenTransforms::getPositionFromVRMatrix(mat);
			Eigen::Quaterniond quaternion = VR2EigenTransforms::getQuaternionFromVRMatrix(mat);

			// 应用Y轴偏移
			position.y() -= VRInputConfig::Y_OFFSET;

			// 初始化本地控制器数据
			local_data.time = vr_utils::getCurrentTimeWithMilliseconds();
			local_data.role = role_index;
			local_data.setPosition(position);
			local_data.setQuaternion(quaternion);

			logMessage(Debug, std::string("Controller ") + std::to_string(role_index) +
				": Pos(" + std::to_string(position.x()) + "," +
				std::to_string(position.y()) + "," +
				std::to_string(position.z()) + ")");

			// 4.4 获取控制器状态并处理按钮
			vr::VRControllerState_t controllerState;
			pHMD->GetControllerState(i, &controllerState, sizeof(controllerState));
			processControllerButtons(i, controllerState, role_index);

			// 4.5 处理扳机反馈
			processTriggerFeedback(controllerState.rAxis[1].x, i, role_index);

			// 4.6 校验位姿合理性
			if (!validatePositionChange(position, role_index)) {
				continue; // 位姿不合理则跳过
			}

			// 4.7 存储控制器数据
			if (role_index == VRInputConfig::RIGHT_CONTROLLER_INDEX) {
				right_controller_data = local_data;
				right_controller_updated = true;
			}
			else {
				left_controller_data = local_data;
				left_controller_updated = true;
			}
			// 新增：将Controller原始数据入队发送
			DeviceRawData ctrl_raw = convertControllerToRawData(local_data, role_index);
			send_queue_.push(std::move(ctrl_raw));
		}

		// 5. 按频率发布控制器数据
		std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
		bool anyControllerDetected = controller_detected[0] || controller_detected[1];

		for (int role = 0; role < VRInputConfig::MAX_CONTROLLERS; role++) {
			bool has_data = (role == 0) ? right_controller_updated : left_controller_updated;
			if (!has_data) continue;

			// 初始化发布时间
			if (!publish_time_initialized[role]) {
				last_publish_time[role] = currentTime;
				publish_time_initialized[role] = true;
				continue;
			}

			// 检查是否达到发布间隔
			std::chrono::steady_clock::duration time_since_publish = currentTime - last_publish_time[role];
			if (time_since_publish >= publish_interval) {
				// 线程安全更新共享数据
				{
					std::lock_guard<std::mutex> lock(data_mutex);
					shared_data = (role == 0) ? right_controller_data : left_controller_data;
					shared_data.time = vr_utils::getCurrentTimeWithMilliseconds();
					shared_data.role = role;
				}

				// 通知消费线程
				data_cv.notify_one();

				std::string controllerType = (role == 0) ? "Right" : "Left";
				logMessage(Debug, std::string("Published ") + controllerType +
					" Controller data (Interval: " + std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(time_since_publish).count()) + "ms)");

				last_publish_time[role] = currentTime;
			}
		}

		// 6. 处理无控制器连接的情况
		if (!anyControllerDetected) {
			// 每隔1秒打印一次日志
			if (std::chrono::duration_cast<std::chrono::seconds>(currentTime - lastLogTime).count() >= VRInputConfig::LOG_INTERVAL_SECONDS) {
				logMessage(Info, std::string("No controllers detected - sleeping for ") +
					std::to_string(VRInputConfig::NO_CONTROLLER_SLEEP_MS) + "ms");
				lastLogTime = currentTime;
			}
			//std::this_thread::sleep_for(std::chrono::milliseconds(VRInputConfig::NO_CONTROLLER_SLEEP_MS));
		}
		else {
			// 有控制器时短睡眠，保证采集频率
			//std::this_thread::sleep_for(std::chrono::milliseconds(VRInputConfig::DATA_COLLECTION_SLEEP_MS));//todo睡眠影响
		}
	}
}


DeviceRawData vive_input::convertControllerToRawData(const vr_controller_data& ctrl_data, int role_index) {
	DeviceRawData raw_data;
	raw_data.type = DeviceRawData::TYPE_CONTROLLER;
	raw_data.id = role_index; // Controller用role_index作为编号（0=右，1=左）
	raw_data.position = ctrl_data.getPosition();
	raw_data.quaternion = ctrl_data.getQuaternion();
	raw_data.time = ctrl_data.time;
	// 复制Controller特有字段
	raw_data.menu_button = ctrl_data.menu_button;
	raw_data.trigger_button = ctrl_data.trigger_button;
	raw_data.trackpad_touch = ctrl_data.trackpad_touch;
	raw_data.trackpad_button = ctrl_data.trackpad_button;
	raw_data.grip_button = ctrl_data.grip_button;
	raw_data.trackpad_x = ctrl_data.trackpad_x;
	raw_data.trackpad_y = ctrl_data.trackpad_y;
	raw_data.trigger = ctrl_data.trigger;
	return raw_data;
}

DeviceRawData vive_input::convertTrackerToRawData(const TrackerData& tracker_data, const vr::HmdMatrix34_t& mat) {
	DeviceRawData raw_data;
	raw_data.type = DeviceRawData::TYPE_TRACKER;
	raw_data.id = tracker_data.id; // Tracker用配置的id作为编号
	raw_data.position = VR2EigenTransforms::getPositionFromVRMatrix(mat);
	raw_data.quaternion = VR2EigenTransforms::getQuaternionFromVRMatrix(mat);
	raw_data.time = vr_utils::getCurrentTimeWithMilliseconds();
	// Tracker无按钮字段，保持默认值
	return raw_data;
}

void vive_input::sendDataLoop() {
	logMessage(Info, "Starting device raw data send thread");
	// 初始连接
	if (!tcp_client_.isConnected() && send_thread_running_) {
		tcp_client_.autoReconnect(send_thread_running_);
	}

	while (send_thread_running_) {
		DeviceRawData data;
		if (send_queue_.pop(data)) {
			// 1. 数据转换为JSON格式
			json json_data;
			json_data["type"] = (data.type == DeviceRawData::TYPE_TRACKER) ? "tracker" : "controller";
			json_data["id"] = data.id;
			json_data["timestamp"] = data.time;
			// 位置
			json_data["position"]["x"] = data.position.x();
			json_data["position"]["y"] = data.position.y();
			json_data["position"]["z"] = data.position.z();
			// 四元数
			json_data["quaternion"]["x"] = data.quaternion.x();
			json_data["quaternion"]["y"] = data.quaternion.y();
			json_data["quaternion"]["z"] = data.quaternion.z();
			json_data["quaternion"]["w"] = data.quaternion.w();

			// Controller额外字段
			if (data.type == DeviceRawData::TYPE_CONTROLLER) {
				json_data["buttons"]["menu"] = data.menu_button;
				json_data["buttons"]["trigger_button"] = data.trigger_button;
				json_data["buttons"]["trackpad_touch"] = data.trackpad_touch;
				json_data["buttons"]["trackpad_button"] = data.trackpad_button;
				json_data["buttons"]["grip"] = data.grip_button;
				json_data["trackpad"]["x"] = data.trackpad_x;
				json_data["trackpad"]["y"] = data.trackpad_y;
				json_data["trigger_value"] = data.trigger;
			}

			// 2. 转换为字符串
			std::string send_str = json_data.dump();

			// 3. 发送数据（自动重连）
			if (!tcp_client_.isConnected()) {
				tcp_client_.autoReconnect(send_thread_running_);
			}

			if (tcp_client_.sendData(send_str)) {
				std::string device_type = (data.type == DeviceRawData::TYPE_TRACKER) ? "Tracker" : "Controller";
				// 修复1：LogLevel补全作用域 + 字符串拼接显式转换
				logMessage(LogLevel::Debug,
					"TCP sent " + device_type + " ID:" + std::to_string(data.id) + " data: " + send_str);
			}
			else {
				// 修复2：LogLevel补全作用域 + 拆分字符串拼接避免隐式转换
				std::string device_type = (data.type == DeviceRawData::TYPE_TRACKER) ? "Tracker" : "Controller";
				std::string error_msg = "TCP send failed for " + device_type + " ID:" + std::to_string(data.id);
				logMessage(LogLevel::Error, error_msg);  // 核心修复：LogLevel::Error 替代 Error
			}
		}
	}

	// 退出前断开连接
	tcp_client_.disconnect();
	logMessage(Info, "Exiting device raw data TCP send thread");
}


void vive_input::startSendThread() {
	if (send_thread_running_) {
		logMessage(Warning, "Send thread is already running");
		return;
	}

	try {
		send_thread_running_ = true;
		// 捕获线程构造/启动异常
		send_thread_ = std::thread(&vive_input::sendDataLoop, this);
		logMessage(Info, "Send thread started successfully");
	}
	catch (const std::system_error& e) {
		send_thread_running_ = false;
		// 打印错误码 + 描述（关键！定位具体原因）
		logMessage(Error, "Failed to start send thread: " + std::string(e.what()) +
			" (error code: " + std::to_string(e.code().value()) + ")");
	}
	catch (...) {
		send_thread_running_ = false;
		logMessage(Error, "Unknown error when starting send thread");
	}
}

void vive_input::stopSendThread() {
	if (!send_thread_running_) return;

	// 1. 发出停止信号
	send_thread_running_ = false;
	send_queue_.stop(); // 确保你的 SafeQueue 有 stop 机制能唤醒 pop 阻塞

	// 2. 正常等待线程结束
	if (send_thread_.joinable()) {
		logMessage(Info, "Waiting for send thread to join...");
		send_thread_.join();
	}

	logMessage(Info, "Send thread stopped safely");
}