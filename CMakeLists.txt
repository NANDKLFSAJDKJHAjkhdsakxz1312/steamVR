cmake_minimum_required (VERSION 3.8)

# ========== 仅保留核心修复：解决 no_init_all + 兼容 SDK ==========
# 指定兼容的 Windows SDK 版本（Ninja/VS 通用）
set(CMAKE_SYSTEM_VERSION 10.0.19041.0 CACHE STRING "" FORCE)
# 屏蔽 no_init_all 特性（关键：解决 winnt.h 报错）
add_definitions(-Dno_init_all)

project (vive_pose2robot_joint_value)

# ========== C++ 标准 + MSVC 编译配置 ==========
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(MSVC)
    # 禁用 Windows 不安全函数/废弃 Socket 警告
    add_definitions(-D_CRT_SECURE_NO_WARNINGS)
    add_definitions(-D_WINSOCK_DEPRECATED_NO_WARNINGS)
    # 动态运行时库（MD/MDd），与 yaml-cpp 匹配
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} /MDd")
    set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} /MD")
    set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO} /MD")
endif()

# ========== Qt 配置 ==========
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(Qt5_DIR "C:\\Qt\\Qt5.14.2\\5.14.2\\msvc2017_64\\lib\\cmake\\Qt5")
find_package(Qt5 COMPONENTS Widgets Core Gui Network REQUIRED)

# ========== 第三方库配置（仅查找，不关联目标） ==========
# 1. Eigen3
set(Eigen3_DIR "D:\\software\\eigen_5_0_1_install\\share\\eigen3\\cmake")
find_package(Eigen3 5.0.1 REQUIRED NO_MODULE)

# 2. OpenVR
set(OpenVR_DIR "D:\\software\\openvr_2_12_14_install\\cmake")
find_package(OpenVR REQUIRED)

# 3. yaml-cpp（仅查找库路径，先不关联目标）
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
    set(YAML_LIB_NAME "yaml-cppd")
else()
    set(YAML_LIB_NAME "yaml-cpp")
endif()
find_library(YAML_CPP_LIBRARY
    NAMES ${YAML_LIB_NAME}
    PATHS "D:\\software\\yaml-cpp_0_8_0_install\\lib"
    NO_DEFAULT_PATH
    REQUIRED
)

# ========== 包含目录（全局，所有目标共享） ==========
include_directories(./inc)
# nlohmann/json 头文件（JSON 库依赖，替换为你的实际路径）
set(NLOHMANN_JSON_INCLUDE_DIR "D:\\software\\nlohmann_json_3_11_2\\include")
include_directories(${NLOHMANN_JSON_INCLUDE_DIR})
# yaml-cpp 头文件（全局包含）
include_directories("D:\\software\\yaml-cpp_0_8_0_install\\include")

# ========== 可执行文件（必须先定义目标！） ==========
add_executable(${PROJECT_NAME}_node
    main.cpp
    ./src/vr_devices.cpp
    ./src/Matrices.cpp
)

add_executable(${PROJECT_NAME}_server
    main_server.cpp
 

)

# ========== 目标专属配置（必须在 add_executable 之后） ==========
# yaml-cpp 静态库宏
target_compile_definitions(${PROJECT_NAME}_node PRIVATE
    YAML_CPP_STATIC_DEFINE  # yaml-cpp 静态库宏
    WIN32_LEAN_AND_MEAN     # 精简 Windows 头文件
)

# 链接库（所有依赖库）
target_link_libraries(${PROJECT_NAME}_node
    # Windows 系统库（Socket/系统调用）
    ws2_32.lib
    advapi32.lib
    # 第三方库
    Eigen3::Eigen
    OpenVR::OpenVR
    ${YAML_CPP_LIBRARY}
    # Qt 库
    Qt5::Core
    Qt5::Gui
    Qt5::Widgets
    Qt5::Network
)

