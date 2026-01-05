#ifndef _VR_DEVICES_H_
#define _VR_DEVICES_H_
#ifdef _WIN32  // 仅Windows平台生效
#define _WINSOCK_DEPRECATED_NO_WARNINGS  // 屏蔽废弃函数警告
#define WIN32_LEAN_AND_MEAN              // 精简Windows头文件，避免冲突
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
// 链接WS2_32.lib（仅Windows需要）
#pragma comment(lib, "WS2_32.lib")
#endif

//模仿的demo: https://github.com/iltlo/vive_ros2
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>
#include <cmath>
#include <mutex>
#include <condition_variable>
#include <map>
#include <Eigen/Dense>
#include <yaml-cpp/yaml.h>
#include <Eigen/Geometry>
#include "data.h"
#include "openvr.h"
#include "VR2EigenTransforms.h"
#include <queue>
#include <atomic>
#include <mutex>
#include <condition_variable>
#include <vector>
#include <nlohmann/json.hpp>
#include <thread>

using json = nlohmann::json;


// TCP配置常量
const std::string TCP_SERVER_IP = "192.168.0.215";  
const uint16_t TCP_SERVER_PORT = 8888;          
const int TCP_RECONNECT_INTERVAL_MS = 1000;     
const int TCP_SEND_RETRY_TIMES = 3;             
const int TCP_SEND_TIMEOUT_MS = 500;            



enum LogLevel {
	Info,
	Debug,
	Warning,
	Error
};

namespace VRInputConfig {
	constexpr double DEFAULT_PUBLISH_FREQUENCY = 50.0;
	constexpr double MIN_PUBLISH_FREQUENCY = 0.1;
	constexpr double MAX_PUBLISH_FREQUENCY = 1000.0;
	constexpr int TRIGGER_FEEDBACK_STEPS = 6;
	constexpr int HAPTIC_FEEDBACK_DURATION = 200;
	constexpr int TRIGGER_HAPTIC_MULTIPLIER = 3000;
	constexpr int WARNING_HAPTIC_DURATION = 20;
	constexpr float POSITION_THRESHOLD = 0.05f;
	constexpr float DISTANCE_THRESHOLD = 0.1f;
	constexpr int NO_CONTROLLER_SLEEP_MS = 100;
	constexpr int DATA_COLLECTION_SLEEP_MS = 5;
	constexpr int CONTROLLER_DELAY_MS = 1;
	constexpr int LOG_INTERVAL_SECONDS = 1;
	constexpr double Y_OFFSET = 0.6;

	// Controller indices
	constexpr int RIGHT_CONTROLLER_INDEX = 0;
	constexpr int LEFT_CONTROLLER_INDEX = 1;
	constexpr int MAX_CONTROLLERS = 2;
}

struct vr_controller_data {
	double pose_x, pose_y, pose_z;          // Position
	double pose_qx, pose_qy, pose_qz, pose_qw; // Quaternion orientation

	bool menu_button = false;
	bool trigger_button = false;
	bool trackpad_touch = false;
	bool trackpad_button = false;
	bool grip_button = false;

	double trackpad_x = 0.0, trackpad_y = 0.0;
	double trigger = 0.0;

	int role = 1;  // 0 for right, 1 for left
	std::string time;

	Eigen::Vector3d getPosition() const;
	Eigen::Quaterniond getQuaternion() const;
	void setPosition(const Eigen::Vector3d& pos);
	void setQuaternion(const Eigen::Quaterniond& q);
	void reset();
};

// Thread-safe logging function with timestamps
inline void logMessage(LogLevel level, const std::string& message);

// ------------------------------
// 线程安全TCP客户端类
// ------------------------------
class ThreadSafeTCPClient {
public:
	ThreadSafeTCPClient(const std::string& server_ip, uint16_t server_port)
		: server_ip_(server_ip), server_port_(server_port), sock_fd_(INVALID_SOCKET), is_connected_(false) {
		// 初始化Winsock2
		WSADATA wsaData;
		int ret = WSAStartup(MAKEWORD(2, 2), &wsaData);
		if (ret != 0) {
			logMessage(Error, "WSAStartup failed: " + std::to_string(ret));
		}
	}

	~ThreadSafeTCPClient() {
		std::lock_guard<std::mutex> lock(mutex_);
		// 1. 关闭 Socket
		if (sock_fd_ != INVALID_SOCKET) {
			closesocket(sock_fd_);
			sock_fd_ = INVALID_SOCKET;
		}
		// 2. 清理 Winsock（仅全局调用一次，避免重复调用）
		static bool wsa_cleaned = false;
		if (!wsa_cleaned) {
			WSACleanup();
			wsa_cleaned = true;
		}
		is_connected_ = false;
	}

	// 连接服务器（Windows版）
	bool connect() {
		printf("137");
		std::lock_guard<std::mutex> lock(mutex_);
		printf("139");
		disconnect(); // 先断开旧连接
		printf("140");
		// 创建socket
		sock_fd_ = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
		if (sock_fd_ == INVALID_SOCKET) {
			logMessage(Error, "TCP socket create failed: " + std::to_string(WSAGetLastError()));
			return false;
		}
		printf("146");
		// 设置非阻塞（Windows版）
		u_long mode = 1; // 1=非阻塞，0=阻塞
		if (ioctlsocket(sock_fd_, FIONBIO, &mode) == SOCKET_ERROR) {
			logMessage(Warning, "Set non-blocking failed: " + std::to_string(WSAGetLastError()));
		}

		// 服务器地址配置
		sockaddr_in server_addr;
		ZeroMemory(&server_addr, sizeof(server_addr));
		server_addr.sin_family = AF_INET;
		server_addr.sin_port = htons(server_port_);
		if (inet_pton(AF_INET, server_ip_.c_str(), &server_addr.sin_addr) <= 0) {
			logMessage(Error, "TCP invalid server IP: " + server_ip_);
			disconnect();
			return false;
		}

		// 连接服务器（非阻塞）
		int ret = ::connect(sock_fd_, (sockaddr*)&server_addr, sizeof(server_addr));
		printf("166");
		if (ret == SOCKET_ERROR && WSAGetLastError() != WSAEWOULDBLOCK) {
			logMessage(Error, "TCP connect failed: " + std::to_string(WSAGetLastError()));
			disconnect();
			return false;
		}

		// 等待连接完成（超时控制）
		fd_set write_fds;
		FD_ZERO(&write_fds);
		FD_SET(sock_fd_, &write_fds);
		timeval timeout;
		timeout.tv_sec = 0;
		timeout.tv_usec = TCP_SEND_TIMEOUT_MS * 1000;

		ret = select(0, nullptr, &write_fds, nullptr, &timeout);
		if (ret <= 0) {
			logMessage(Error, "TCP connect timeout (" + std::to_string(TCP_SEND_TIMEOUT_MS) + "ms)");
			disconnect();
			return false;
		}

		// 检查连接是否成功
		int error = 0;
		socklen_t len = sizeof(error);
		getsockopt(sock_fd_, SOL_SOCKET, SO_ERROR, (char*)&error, &len);
		if (error != 0) {
			logMessage(Error, "TCP connect failed: " + std::to_string(error));
			disconnect();
			return false;
		}

		// 恢复阻塞模式
		mode = 0;
		ioctlsocket(sock_fd_, FIONBIO, &mode);

		is_connected_ = true;
		logMessage(Info, "TCP connected to " + server_ip_ + ":" + std::to_string(server_port_));
		printf("204");
		return true;
	}

	// 断开连接（Windows版）
	// 断开连接（无锁版，仅在已加锁的上下文中调用）
	void disconnect() {
		printf("212\n");
		// 移除内部的lock_guard！
		printf("215\n");
		if (sock_fd_ != INVALID_SOCKET) {
			printf("218\n");
			closesocket(sock_fd_);
			printf("220\n");
			sock_fd_ = INVALID_SOCKET;
			printf("222\n");
		}
		is_connected_ = false;
		printf("disconnect执行完成\n");
	}

	// 发送数据（Windows版）
	// Windows专用sendData函数（仅用int，无ssize_t）
	bool sendData(const std::string& data) {
		std::lock_guard<std::mutex> lock(mutex_);
		if (!is_connected_) return false;

		// 拼接换行符，方便服务端解析
		std::string send_data = data + "\n";
		const char* buf = send_data.c_str();
		int total_sent = 0;          // 替换ssize_t为int
		int remaining = (int)send_data.size(); // 替换ssize_t为int

		// 重试机制（Windows专用）
		for (int retry = 0; retry < TCP_SEND_RETRY_TIMES; retry++) {
			while (remaining > 0) {
				// Windows send函数原生返回int，直接调用
				int sent = send(sock_fd_, buf + total_sent, remaining, 0);

				// 发送失败处理（Windows SOCKET_ERROR）
				if (sent == SOCKET_ERROR) {
					int err_code = WSAGetLastError();
					logMessage(LogLevel::Warning,
						"TCP send failed (retry " + std::to_string(retry + 1) +
						"): ErrorCode=" + std::to_string(err_code));
					break;
				}

				// 更新已发送/剩余字节数
				total_sent += sent;
				remaining -= sent;
			}

			// 全部发送成功则返回
			if (remaining == 0) {
				return true;
			}

			// 重试间隔（Windows sleep_for）
			std::this_thread::sleep_for(std::chrono::milliseconds(100));
		}

		// 多次重试失败，标记连接断开
		is_connected_ = false;
		logMessage(LogLevel::Error,
			"TCP send failed after " + std::to_string(TCP_SEND_RETRY_TIMES) + " retries, disconnecting");
		return false;
	}

	// 检查连接状态
	bool isConnected() const {
		std::lock_guard<std::mutex> lock(mutex_);
		return is_connected_;
	}

	// 自动重连
	void autoReconnect(std::atomic_bool& running_flag) { // 接收退出标记参数
	// 重连循环中检测外部传入的退出标记
		while (!isConnected() && running_flag) {
			logMessage(Warning, "TCP reconnecting in " + std::to_string(TCP_RECONNECT_INTERVAL_MS) + "ms...");
			//std::this_thread::sleep_for(std::chrono::milliseconds(TCP_RECONNECT_INTERVAL_MS));
			printf("enter reconnection part");
			if (connect()) break;
		}
		// 线程退出时停止重连
		printf("exit from autoreconnection");
		if (!running_flag) {
			logMessage(Info, "Send thread stopped, TCP reconnect aborted");
		}
	}

private:
	std::string server_ip_;
	uint16_t server_port_;
	SOCKET sock_fd_ = INVALID_SOCKET;          // Windows socket句柄
	bool is_connected_;
	mutable std::mutex mutex_;
};

class vr_utils {
public:
	static std::string getCurrentTimeWithMilliseconds();
	static bool deviceIsConnected(vr::IVRSystem* pHMD, vr::TrackedDeviceIndex_t unDeviceIndex);
	static bool controllerIsConnected(vr::IVRSystem* pHMD, vr::TrackedDeviceIndex_t unDeviceIndex);
	static void deviceConnectionCheck(vr::IVRSystem* pHMD);
	static vr::ETrackedControllerRole controllerRoleCheck(vr::IVRSystem* pHMD, vr::TrackedDeviceIndex_t i);
	static void controllerConnectionCheck(vr::IVRSystem* pHMD);
	static void HapticFeedback(vr::IVRSystem* pHMD, vr::TrackedDeviceIndex_t unControllerDeviceIndex, unsigned short durationMicroSec);
	static void extractPoseFromMatrix(const vr::HmdMatrix34_t& vrMatrix, vr_controller_data& data);
	static vr_controller_data calculateRelativePose(const vr_controller_data& initial, const vr_controller_data& current);
	static vr_controller_data filterPose(const vr_controller_data& current, const vr_controller_data& previous, double alpha = 0.8);
};

struct DeviceRawData {
    enum DeviceType {
        TYPE_TRACKER,
        TYPE_CONTROLLER
    } type;                  // 设备类型
    int id;                  // 设备编号（Tracker的id / Controller的role_index）
    Eigen::Vector3d position; 
    Eigen::Quaterniond quaternion; 
    std::string time;        
    // Controller特有字段（Tracker时为空/0）
    bool menu_button = false;
    bool trigger_button = false;
    bool trackpad_touch = false;
    bool trackpad_button = false;
    bool grip_button = false;
    float trackpad_x = 0.0f;
    float trackpad_y = 0.0f;
    float trigger = 0.0f;
};

template <typename T>
class ThreadSafeQueue {
public:
	void push(T&& item) {
		std::lock_guard<std::mutex> lock(mutex_);
		queue_.push(std::move(item));
		cv_.notify_one();
	}

	bool pop(T& item, std::chrono::milliseconds timeout = std::chrono::milliseconds(100)) {
		std::unique_lock<std::mutex> lock(mutex_);
		if (cv_.wait_for(lock, timeout, [this]() { return !queue_.empty() || stop_flag_; })) {
			if (stop_flag_ && queue_.empty()) return false;
			item = std::move(queue_.front());
			queue_.pop();
			return true;
		}
		return false;
	}

	void stop() {
		std::lock_guard<std::mutex> lock(mutex_);
		stop_flag_ = true;
		cv_.notify_one();
	}

	bool isStopped() const { return stop_flag_; }

private:
	std::queue<T> queue_;
	mutable std::mutex mutex_;
	std::condition_variable cv_;
	std::atomic_bool stop_flag_{ false };
};

class vive_input{
public:
	vive_input(std::mutex &mutex, std::condition_variable &cv, vr_controller_data &data, double publish_freq = VRInputConfig::DEFAULT_PUBLISH_FREQUENCY);
	~vive_input();
	void runVR();
	void setPublishFrequency(double freq);
	// 新增：启动发送线程
	void startSendThread();
	// 新增：停止发送线程
	void stopSendThread();

private:
	vr::IVRSystem *pHMD = nullptr;
	vr::EVRInitError eError = vr::VRInitError_None;
	vr::TrackedDevicePose_t trackedDevicePose[vr::k_unMaxTrackedDeviceCount];

	std::mutex &data_mutex;
	std::condition_variable &data_cv;
	vr_controller_data &shared_data;
	vr_controller_data local_data;

	// Track both controllers separately (right = 0, left = 1)
	bool controller_detected[VRInputConfig::MAX_CONTROLLERS] = { false, false };
	Eigen::Vector3d prev_position[VRInputConfig::MAX_CONTROLLERS];  // Use Eigen for better vector operations
	std::chrono::steady_clock::time_point prev_time[VRInputConfig::MAX_CONTROLLERS];
	bool first_run[VRInputConfig::MAX_CONTROLLERS] = { true, true };

	// Store data for both controllers separately
	vr_controller_data right_controller_data; // For right controller (role_index = 0)
	vr_controller_data left_controller_data;  // For left controller (role_index = 1)
	bool right_controller_updated = false;
	bool left_controller_updated = false;

	double controllerPublishFrequency = VRInputConfig::DEFAULT_PUBLISH_FREQUENCY;
	std::chrono::steady_clock::time_point last_publish_time[VRInputConfig::MAX_CONTROLLERS]; // For each controller
	bool publish_time_initialized[VRInputConfig::MAX_CONTROLLERS] = { false, false };

	bool initVR();
	bool shutdownVR();
	void processControllerButtons(uint32_t deviceIndex, vr::VRControllerState_t& controllerState, int roleIndex);
	void processTriggerFeedback(float triggerValue, uint32_t deviceIndex, int roleIndex);
	bool validatePositionChange(const Eigen::Vector3d& currentPosition, int roleIndex);



	std::vector<TrackerData> tracker_data;
	void loadTrackerConfig();
	bool config_loaded = false;

	// 新增：发送线程相关
	ThreadSafeQueue<DeviceRawData> send_queue_;  // 待发送数据队列
	std::thread send_thread_;                   // 发送线程
	std::atomic_bool send_thread_running_{ false }; // 发送线程运行标记

	// 新增：发送数据处理函数（线程入口）
	void sendDataLoop();
	// 新增：将Controller数据转换为DeviceRawData
	DeviceRawData convertControllerToRawData(const vr_controller_data& ctrl_data, int role_index);
	// 新增：将Tracker数据转换为DeviceRawData
	DeviceRawData convertTrackerToRawData(const TrackerData& tracker_data, const vr::HmdMatrix34_t& mat);
	
	//trakcer相关的还可以模仿的demo: https://github.com/cnlohr/openvr-dump-poses    https://github.com/zodsoft/LSLOpenVR
	// 新增：TCP客户端
	ThreadSafeTCPClient tcp_client_{ TCP_SERVER_IP, TCP_SERVER_PORT };
	//这些函数还没有实现。
	//void processControllerData(uint32_t deviceIndex, int roleIndex);
	//void publishControllerDataAtFrequency();
	//void handleControllerDetection(bool anyControllerDetected, std::chrono::steady_clock::time_point currentTime, std::chrono::steady_clock::time_point& lastLogTime);
};

#endif _VR_DEVICES_H_