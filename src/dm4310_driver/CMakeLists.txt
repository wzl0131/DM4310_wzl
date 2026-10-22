cmake_minimum_required(VERSION 3.8)
project(dm4310_driver)

# ---- C++ 标准：ROS 2 要 C++17 ----
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

# ---- 编译警告 ----
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ---- 找 ROS 2 的包 ----
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)   # ⭐ msg 生成

# ============================================================
#  DM-LinkX-4C SDK（dmcan）接入
# ============================================================
find_package(PkgConfig REQUIRED)
pkg_check_modules(LIBUSB REQUIRED libusb-1.0)

set(DM_SDK_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/DMLink)

# ============================================================
#  ⭐ 把 libdm_device.so 也复制一份到【构建目录】里
#
#  原因：用 `colcon build --symlink-install` 时，
#        install/.../可执行文件 是指向 build/.../可执行文件 的符号链接，
#        而 glibc 的 $ORIGIN 是按【真实路径】算的，也就是 build 目录。
#        build 目录里没有 .so → 运行时报
#        "libdm_device.so: cannot open shared object file"
#
#  所以每个可执行文件构建完，顺手把 .so 拷到它旁边。
# ============================================================
function(dm_copy_sdk_lib target)
  add_custom_command(TARGET ${target} POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            ${DM_SDK_DIR}/libdm_device.so
            $<TARGET_FILE_DIR:${target}>/libdm_device.so
    COMMENT "copy libdm_device.so next to ${target}"
  )
endfunction()

# ============================================================
#  ⭐ 类的三层结构（完全参照队里 dogdualarms 的格式）
#
#    can/    CAN 总线封装（CanBus 类 + 发帧函数）
#    motor/  Motor 基类 + DM 派生（MIT 打包/解析/使能）
#    node    业务层（ROS 2 话题 / 定时器 / 安全逻辑）
# ============================================================
set(CAN_SOURCES
  src/can/can.cpp
)

set(MOTOR_SOURCES
  src/motor/motor.cpp
  src/motor/DM_motor.cpp
)

# ============================================================
#  dm4310_node：主节点
# ============================================================
add_executable(dm4310_node
  src/dm4310_node.cpp
  ${CAN_SOURCES}
  ${MOTOR_SOURCES}
)

target_include_directories(dm4310_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include      # can/can.hpp, motor/motor.hpp
  ${DM_SDK_DIR}                            # dmcan.h
)

target_link_libraries(dm4310_node
  ${DM_SDK_DIR}/libdm_device.so
  ${LIBUSB_LIBRARIES}
)

# 运行时能找到 .so（$ORIGIN = 可执行文件所在目录）
set_target_properties(dm4310_node PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE
)

dm_copy_sdk_lib(dm4310_node)

ament_target_dependencies(dm4310_node
  rclcpp
  std_msgs
  sensor_msgs
)

# ============================================================
#  ⭐ 自定义消息 DmMitCommand
#
#  一个消息装 MIT 的五个参数（position/velocity/kp/kd/torque）+ mode，
#  取代原来三个独立的 Float64 话题。
# ============================================================
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DmMitCommand.msg"
)

# 让 dm4310_node 能 include 生成出来的头文件
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(dm4310_node "${cpp_typesupport_target}")

# ============================================================
#  dm_scan：CAN_ID / MST_ID 扫描器（硬件到手第一件事跑）
# ============================================================
add_executable(dm_scan
  src/dm_scan.cpp
  ${CAN_SOURCES}
)

target_include_directories(dm_scan PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${DM_SDK_DIR}
)

target_link_libraries(dm_scan
  ${DM_SDK_DIR}/libdm_device.so
  ${LIBUSB_LIBRARIES}
)

dm_copy_sdk_lib(dm_scan)

set_target_properties(dm_scan PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# ============================================================
#  sdk_probe：硬件探针（只验 SDK + 设备，不碰电机）
# ============================================================
add_executable(sdk_probe src/sdk_probe.cpp)

target_include_directories(sdk_probe PRIVATE ${DM_SDK_DIR})

target_link_libraries(sdk_probe
  ${DM_SDK_DIR}/libdm_device.so
  ${LIBUSB_LIBRARIES}
)

dm_copy_sdk_lib(sdk_probe)

set_target_properties(sdk_probe PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_WITH_INSTALL_RPATH TRUE
)

# ---- 安装 ----
install(TARGETS dm4310_node dm_scan sdk_probe
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES ${DM_SDK_DIR}/libdm_device.so
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# ⭐ 导出 msg 类型支持（否则下游包用不了 DmMitCommand）
ament_export_dependencies(rosidl_default_runtime)

ament_package()
