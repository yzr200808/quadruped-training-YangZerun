# =============================================================================
#  rl_sar —— “控制器节点 rl_sim”这个功能包的编译脚本（ROS2 / ament_cmake）
#
#  整个文件只干 5 件事：找依赖 → 编 2 个核心库 → 编出 rl_sim → 安装 → 注册成 ROS2 包
#
#  怎么编译（在工作空间根目录 /home/yzr/work/大作业/finally 下执行）：
#      colcon build --packages-select mit_interfaces rl_sim
#      source install/setup.bash
#
#  注意：本包必须有 libtorch 才能编译，见下面【3】里的提示。
# =============================================================================

cmake_minimum_required(VERSION 3.8)   # ament_cmake 要求 CMake >= 3.8
project(rl_sar VERSION 3.0.0 LANGUAGES CXX)   # 包名 rl_sar，必须和 package.xml / 目录名一致

# -----------------------------------------------------------------------------
# 【1】编译分支与全局编译选项
#      USE_ROS2               ：打开 rl_sim.cpp / rl_sim.hpp 里 #if defined(USE_ROS2) 的那套代码
#      CMAKE_CURRENT_SOURCE_DIR：rl_sdk.cpp 用它拼出 policy/*.yaml 和 *.pt 的绝对路径，
#                                漏了它，程序运行时会报“配置文件/模型不存在”
#      WITHOUT_NUMPY          ：让 matplotlibcpp.h 不再依赖 numpy（绘图开关 PLOT 默认是关的）
# -----------------------------------------------------------------------------
if(NOT "$ENV{ROS_DISTRO}" MATCHES "foxy|humble")
    message(WARNING "当前 ROS_DISTRO='$ENV{ROS_DISTRO}'，本文件按 ROS2 编译；要编 ROS1 请自行调整")
endif()
add_compile_definitions(USE_ROS2)
add_compile_definitions(WITHOUT_NUMPY)
add_definitions(-DCMAKE_CURRENT_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
set(CMAKE_CXX_STANDARD 17)            # libtorch 2.x 的头文件需要 C++17
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# 【2】找 ROS2 依赖 —— 这里是包含各种库（节点通信、ROS 消息、以及你自己写的 mit_interfaces）
# -----------------------------------------------------------------------------
find_package(ament_cmake REQUIRED)      # ROS2 的构建工具（必须有）
find_package(rclcpp REQUIRED)           # 节点、发布/订阅、参数
find_package(geometry_msgs REQUIRED)    # /cmd_vel（导航模式用）
find_package(sensor_msgs REQUIRED)      # /joy 手柄、/imu/data 姿态
find_package(std_msgs REQUIRED)         # mit_interfaces 里的 Header
find_package(std_srvs REQUIRED)         # rl_sim.hpp 里 include 了 Empty（ROS1 残留，保留不报错）
find_package(rcl_interfaces REQUIRED)   # rl_sim.hpp 里 include 了 GetParameters（同上）
find_package(mit_interfaces REQUIRED)   # 你自己写的 /mit_cmd、/mit_state 消息

# -----------------------------------------------------------------------------
# 【3】找非 ROS 依赖 —— 这里的库不走 rosdep，全部由 CMake 直接找
#      yaml-cpp ：读 policy/*.yaml（系统已装）
#      TBB      ：线程安全的输出队列（系统已装）
#      Python3  ：matplotlibcpp.h 开头就 #include <Python.h>，所以要 Python 头文件
#      Torch    ：libtorch，神经网络推理库，需要自己下载（找不到就给一段提示）
# -----------------------------------------------------------------------------
find_package(yaml-cpp REQUIRED)
find_package(TBB REQUIRED)
find_package(Threads REQUIRED)
find_package(Python3 COMPONENTS Development REQUIRED)

find_package(Torch QUIET)
if(NOT Torch_FOUND)
    message(FATAL_ERROR
        "\n找不到 libtorch！先下载它并把路径告诉 CMake，再重新编译：\n"
        "    cd ~ && wget https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-2.0.1%2Bcpu.zip\n"
        "    unzip libtorch-cxx11-abi-shared-with-deps-2.0.1+cpu.zip -d ~/\n"
        "    echo 'export Torch_DIR=$HOME/libtorch' >> ~/.bashrc && source ~/.bashrc\n")
endif()
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")   # libtorch 要求的 ABI 等编译开关

# -----------------------------------------------------------------------------
# 【4】头文件搜索路径 —— 把我们自己各个模块的目录告诉编译器
# -----------------------------------------------------------------------------
include_directories(
    include                                # rl_sim.hpp
    library/core/matplotlibcpp             # 画图（PLOT 用）
    library/core/observation_buffer        # 6 帧观测缓冲
    library/core/rl_sdk                    # 核心算法
    library/core/loop                      # 定时线程
    library/core/fsm                       # 状态机框架
    policy                                 # fsm.hpp 汇总入口
    ${TORCH_INCLUDE_DIRS}
)

# -----------------------------------------------------------------------------
# 【5】核心库 1：rl_sdk —— 观测拼装、动作解码、读 yaml、加载 .pt、状态机调度
# -----------------------------------------------------------------------------
add_library(rl_sdk library/core/rl_sdk/rl_sdk.cpp)
target_link_libraries(rl_sdk PUBLIC
    ${TORCH_LIBRARIES}
    TBB::tbb
    yaml-cpp
    Threads::Threads
)

# -----------------------------------------------------------------------------
# 【6】核心库 2：observation_buffer —— 把最新的 6 帧观测堆成网络输入（45×6 = 270 维）
# -----------------------------------------------------------------------------
add_library(observation_buffer library/core/observation_buffer/observation_buffer.cpp)
target_link_libraries(observation_buffer PUBLIC ${TORCH_LIBRARIES})

# -----------------------------------------------------------------------------
# 【7】可执行文件 rl_sim —— 控制器节点本体（订阅 /mit_state、/imu/data、/joy，发布 /mit_cmd）
# -----------------------------------------------------------------------------
add_executable(rl_sim src/rl_sim.cpp)
target_link_libraries(rl_sim
    rl_sdk
    observation_buffer
    ${TORCH_LIBRARIES}
    yaml-cpp
    Threads::Threads
    Python3::Python
)
ament_target_dependencies(rl_sim
    rclcpp
    geometry_msgs
    sensor_msgs
    std_msgs
    std_srvs
    rcl_interfaces
    mit_interfaces
)

# -----------------------------------------------------------------------------
# 【8】安装 —— 让 ros2 run / ros2 launch 找得到可执行文件和包清单
#      说明：policy/ 里的 yaml 和 *.pt 不需要安装，代码是按源码目录的绝对路径直接读的
# -----------------------------------------------------------------------------
install(TARGETS rl_sdk observation_buffer
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)
install(TARGETS rl_sim DESTINATION lib/${PROJECT_NAME})
install(FILES package.xml DESTINATION share/${PROJECT_NAME})

# -----------------------------------------------------------------------------
# 【9】注册成 ament 包 —— 必须写在文件的最后
# -----------------------------------------------------------------------------
ament_package()
