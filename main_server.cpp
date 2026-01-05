#include <iostream>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <thread>
#include <cstring>
#include <nlohmann/json.hpp>

// 链接 Winsock2 库（Windows 必须）
#pragma comment(lib, "ws2_32.lib")

using json = nlohmann::json;
const uint16_t SERVER_PORT = 8888;
const int BUFFER_SIZE = 4096;

// 处理客户端连接（线程函数）
void handleClient(SOCKET client_fd) {
	char buffer[BUFFER_SIZE];
	std::string leftover_data; // 缓存未解析完的数据包（处理粘包）

	while (true) {
		// Windows recv 返回 int，替代 Linux ssize_t
		int recv_len = recv(client_fd, buffer, BUFFER_SIZE - 1, 0);

		// 客户端断开/出错
		if (recv_len <= 0) {
			int err_code = WSAGetLastError();
			if (err_code == WSAECONNRESET || recv_len == 0) {
				std::cout << "Client disconnected" << std::endl;
			}
			else {
				std::cerr << "Recv failed: " << WSAGetLastError() << std::endl;
			}
			closesocket(client_fd); // Windows 关闭 socket
			break;
		}

		// 拼接缓存数据 + 新接收数据
		buffer[recv_len] = '\0';
		leftover_data += std::string(buffer);

		// 按换行符分割数据包（处理粘包/拆包）
		size_t pos = 0;
		while ((pos = leftover_data.find('\n')) != std::string::npos) {
			std::string line = leftover_data.substr(0, pos);
			leftover_data.erase(0, pos + 1);

			try {
				// 解析 JSON 数据
				json data = json::parse(line);
				std::cout << "Received " << data["type"] << " ID:" << data["id"] << std::endl;
				std::cout << "Position: ("
					<< data["position"]["x"] << ","
					<< data["position"]["y"] << ","
					<< data["position"]["z"] << ")" << std::endl;
				std::cout << "Quaternion: ("
					<< data["quaternion"]["x"] << ","
					<< data["quaternion"]["y"] << ","
					<< data["quaternion"]["z"] << ","
					<< data["quaternion"]["w"] << ")" << std::endl;

				// Controller 特有字段
				if (data["type"] == "controller") {
					std::cout << "Trigger value: " << data["trigger_value"] << std::endl;
				}
				std::cout << "-------------------------" << std::endl;
			}
			catch (const std::exception& e) {
				std::cerr << "JSON parse error: " << e.what() << std::endl;
			}
		}
	}
}

int main() {
	// 1. 初始化 Winsock2
	WSADATA wsaData;
	int ret = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (ret != 0) {
		std::cerr << "WSAStartup failed: " << ret << std::endl;
		return -1;
	}

	// 2. 创建服务器 Socket（Windows SOCKET 类型，替代 int）
	SOCKET server_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP);
	if (server_fd == INVALID_SOCKET) {
		std::cerr << "Socket create failed: " << WSAGetLastError() << std::endl;
		WSACleanup();
		return -1;
	}

	// 3. 设置端口复用（Windows 版本）
	BOOL opt = TRUE;
	if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, (char*)&opt, sizeof(opt)) == SOCKET_ERROR) {
		std::cerr << "Set socket option failed: " << WSAGetLastError() << std::endl;
		closesocket(server_fd);
		WSACleanup();
		return -1;
	}

	// 4. 绑定地址
	sockaddr_in server_addr;
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = INADDR_ANY; // 监听所有网卡
	server_addr.sin_port = htons(SERVER_PORT);

	if (bind(server_fd, (sockaddr*)&server_addr, sizeof(server_addr)) == SOCKET_ERROR) {
		std::cerr << "Bind failed: " << WSAGetLastError() << std::endl;
		closesocket(server_fd);
		WSACleanup();
		return -1;
	}

	// 5. 监听客户端连接
	if (listen(server_fd, 5) == SOCKET_ERROR) {
		std::cerr << "Listen failed: " << WSAGetLastError() << std::endl;
		closesocket(server_fd);
		WSACleanup();
		return -1;
	}

	std::cout << "TCP server listening on port " << SERVER_PORT << " (Windows)" << std::endl;

	// 6. 循环接受客户端连接
	while (true) {
		sockaddr_in client_addr;
		int client_len = sizeof(client_addr);
		SOCKET client_fd = accept(server_fd, (sockaddr*)&client_addr, &client_len);

		if (client_fd == INVALID_SOCKET) {
			std::cerr << "Accept failed: " << WSAGetLastError() << std::endl;
			continue;
		}

		// 打印客户端 IP + 端口
		char client_ip[INET_ADDRSTRLEN];
		inet_ntop(AF_INET, &client_addr.sin_addr, client_ip, sizeof(client_ip));
		std::cout << "Client connected: " << client_ip << ":" << ntohs(client_addr.sin_port) << std::endl;

		// 启动线程处理客户端（detach 避免线程资源泄漏）
		std::thread client_thread(handleClient, client_fd);
		client_thread.detach();
	}

	// 7. 清理资源（实际不会执行到，仅示例）
	closesocket(server_fd);
	WSACleanup();
	return 0;
}