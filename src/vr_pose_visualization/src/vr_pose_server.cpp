#include <iostream>
#include <string>
#include <cstring>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <thread>
#include <nlohmann/json.hpp>

// ROS头文件
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

using json = nlohmann::json;

// 全局ROS节点句柄（供线程使用）
ros::NodeHandle* g_nh = nullptr;
// TF广播器（发布坐标变换）
tf2_ros::TransformBroadcaster* g_tf_broadcaster = nullptr;

struct DeviceRawData {
    enum DeviceType {
        TYPE_TRACKER,
        TYPE_CONTROLLER
    } type;
    int id;
    double position_x, position_y, position_z;
    double quaternion_x, quaternion_y, quaternion_z, quaternion_w;
    std::string time;
    bool menu_button = false;
    bool trigger_button = false;
    bool trackpad_touch = false;
    bool trackpad_button = false;
    bool grip_button = false;
    float trackpad_x = 0.0f;
    float trackpad_y = 0.0f;
    float trigger = 0.0f;
};

// 发布位姿到ROS话题和TF
void publishPose(const DeviceRawData& data) {
    if (!g_nh) return;

    // 1. 构造PoseStamped消息（RViz可直接订阅）
    geometry_msgs::PoseStamped pose_msg;
    // 设置帧ID（RViz中需对应坐标系，默认map为全局坐标系）
    pose_msg.header.frame_id = "map";
    // 设置时间戳（使用ROS当前时间，也可解析data.time转为ros::Time）
    pose_msg.header.stamp = ros::Time::now();
    
    // 位置
    pose_msg.pose.position.x = data.position_x;
    pose_msg.pose.position.y = data.position_y;
    pose_msg.pose.position.z = data.position_z;
    
    // 四元数（注意：ROS四元数是x,y,z,w顺序，与你的数据一致）
    pose_msg.pose.orientation.x = data.quaternion_x;
    pose_msg.pose.orientation.y = data.quaternion_y;
    pose_msg.pose.orientation.z = data.quaternion_z;
    pose_msg.pose.orientation.w = data.quaternion_w;

    // 按设备类型+ID创建话题名（如/controller_0/pose、/tracker_1/pose）
    std::string topic_name = (data.type == DeviceRawData::TYPE_CONTROLLER) 
        ? "/controller_" + std::to_string(data.id) + "/pose"
        : "/tracker_" + std::to_string(data.id) + "/pose";
    ros::Publisher pub = g_nh->advertise<geometry_msgs::PoseStamped>(topic_name, 10, true);
    pub.publish(pose_msg);

    // 2. 发布TF变换（可选，用于RViz中显示坐标系）
    geometry_msgs::TransformStamped tf_msg;
    tf_msg.header = pose_msg.header;
    // 子坐标系名（如controller_0、tracker_1）
    tf_msg.child_frame_id = (data.type == DeviceRawData::TYPE_CONTROLLER)
        ? "controller_" + std::to_string(data.id)
        : "tracker_" + std::to_string(data.id);
    // 位置
    tf_msg.transform.translation.x = data.position_x;
    tf_msg.transform.translation.y = data.position_y;
    tf_msg.transform.translation.z = data.position_z;
    // 四元数
    tf_msg.transform.rotation = pose_msg.pose.orientation;
    g_tf_broadcaster->sendTransform(tf_msg);
}

void handleClient(int client_socket) {
    std::cout << "Client connected." << std::endl;

    char buffer[4096];
    std::string incomplete_data;

    while (ros::ok() && true) { // 加入ros::ok()，ROS退出时终止线程
        ssize_t bytes_read = read(client_socket, buffer, sizeof(buffer) - 1);
        if (bytes_read <= 0) {
            if (bytes_read == 0) {
                std::cout << "Client closed the connection." << std::endl;
            } else {
                std::cerr << "Read error: " << strerror(errno) << std::endl;
            }
            break;
        }

        buffer[bytes_read] = '\0';
        incomplete_data += buffer;

        // 按行分割
        size_t pos;
        while ((pos = incomplete_data.find('\n')) != std::string::npos) {
            std::string line = incomplete_data.substr(0, pos);
            incomplete_data.erase(0, pos + 1);

            try {
                auto j = json::parse(line);

                DeviceRawData data;
                data.type = (j["type"] == "controller") ? DeviceRawData::TYPE_CONTROLLER : DeviceRawData::TYPE_TRACKER;
                data.id = j["id"];

                // position
                data.position_x = j["position"]["x"];
                data.position_y = j["position"]["y"];
                data.position_z = j["position"]["z"];

                // quaternion
                data.quaternion_w = j["quaternion"]["w"];
                data.quaternion_x = j["quaternion"]["x"];
                data.quaternion_y = j["quaternion"]["y"];
                data.quaternion_z = j["quaternion"]["z"];

                // time
                data.time = j["timestamp"];

                if (data.type == DeviceRawData::TYPE_CONTROLLER) {
                    // buttons
                    data.menu_button = j["buttons"]["menu"];
                    data.trigger_button = j["buttons"]["trigger_button"];
                    data.trackpad_touch = j["buttons"]["trackpad_touch"];
                    data.trackpad_button = j["buttons"]["trackpad_button"];
                    data.grip_button = j["buttons"]["grip"];

                    // trackpad
                    data.trackpad_x = j["trackpad"]["x"];
                    data.trackpad_y = j["trackpad"]["y"];

                    // trigger
                    data.trigger = j["trigger_value"];
                }

                // 打印数据（可选）
                std::cout << "Received data:" << std::endl;
                std::cout << "Type: " << (data.type == DeviceRawData::TYPE_CONTROLLER ? "Controller" : "Tracker") << std::endl;
                std::cout << "ID: " << data.id << std::endl;
                std::cout << "Position: (" << data.position_x << ", " << data.position_y << ", " << data.position_z << ")" << std::endl;
                std::cout << "Quaternion: (" << data.quaternion_x << ", " << data.quaternion_y << ", " << data.quaternion_z << ", " << data.quaternion_w << ")" << std::endl;
                std::cout << "Time: " << data.time << std::endl;
                if (data.type == DeviceRawData::TYPE_CONTROLLER) {
                    std::cout << "Buttons: Menu=" << data.menu_button
                              << ", Trigger=" << data.trigger_button
                              << ", TrackpadTouch=" << data.trackpad_touch
                              << ", TrackpadBtn=" << data.trackpad_button
                              << ", Grip=" << data.grip_button << std::endl;
                    std::cout << "Trackpad: (" << data.trackpad_x << ", " << data.trackpad_y << ")" << std::endl;
                    std::cout << "Trigger: " << data.trigger << std::endl;
                }
                std::cout << "------------------------" << std::endl;

                // 发布位姿到ROS
                publishPose(data);

            } catch (const std::exception& e) {
                std::cerr << "Parse error: " << e.what() << " Line: " << line << std::endl;
            }
        }
    }

    close(client_socket);
}

int main(int argc, char** argv) {
    // 1. 初始化ROS节点
    ros::init(argc, argv, "vr_pose_server");
    ros::NodeHandle nh;
    g_nh = &nh;
    tf2_ros::TransformBroadcaster tf_broadcaster;
    g_tf_broadcaster = &tf_broadcaster;

    // 2. 初始化TCP服务端
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) {
        perror("socket");
        return 1;
    }

    // 关键：设置端口复用，解决Address already in use问题
    int opt = 1;
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt)) < 0) {
        perror("setsockopt");
        close(server_fd);
        return 1;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(8888);

    if (bind(server_fd, (sockaddr*)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(server_fd);
        return 1;
    }

    if (listen(server_fd, 5) < 0) {
        perror("listen");
        close(server_fd);
        return 1;
    }

    std::cout << "Server listening on port 8888..." << std::endl;
    std::cout << "ROS node initialized: vr_pose_server" << std::endl;

    // 3. 循环接收客户端连接
    while (ros::ok()) {
        sockaddr_in client_addr{};
        socklen_t client_len = sizeof(client_addr);
        int client_socket = accept(server_fd, (sockaddr*)&client_addr, &client_len);
        if (client_socket < 0) {
            perror("accept");
            continue;
        }

        // 启动线程处理客户端数据
        std::thread t(handleClient, client_socket);
        t.detach();

        // 处理ROS回调（非必需，但若有订阅逻辑需加）
        ros::spinOnce();
    }

    close(server_fd);
    return 0;
}
