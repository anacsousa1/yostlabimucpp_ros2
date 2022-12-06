cmake_minimum_required(VERSION 3.5)
project(yostlabs_3space_imu)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(lifecycle_msgs REQUIRED)


set(SOURCE_FILES
    src/threespace_api_export.cpp
    src/threespace_api.cpp
    src/threespace_device.cpp
    src/threespace_dongle.cpp
    src/threespace_sensor.cpp
    src/yost/yost_core_api.cpp
    src/yost/yost_fair_mutex.cpp
    src/yost/yost_math.cpp
    src/serial/serial_enumerator_linux.cpp
    src/serial/serial_enumerator_osx.cpp
    src/serial/serial_enumerator_win.cpp
    src/serial/serial_enumerator.cpp
    src/serial/serial.cc
    src/serial/impl/unix.cc
    src/serial/impl/win.cc
)

set(INCLUDE_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/
    ${CMAKE_CURRENT_SOURCE_DIR}/include/yost/
    ${CMAKE_CURRENT_SOURCE_DIR}/include/rapidxml/
    ${CMAKE_CURRENT_SOURCE_DIR}/include/serial/
    ${CMAKE_CURRENT_SOURCE_DIR}/include/serial/impl
    ${CMAKE_CURRENT_SOURCE_DIR}/include/serial/yost
)

##################### exe
add_executable(yostlabs_imu
  src/yostlabs_3space_node.cpp
  ${SOURCE_FILES}
)

ament_target_dependencies(yostlabs_imu
  rclcpp
  std_msgs
)

##################### include
target_include_directories(yostlabs_imu
    PRIVATE
    ${INCLUDE_FILES}
)

##################### target
install(TARGETS
  yostlabs_imu
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
