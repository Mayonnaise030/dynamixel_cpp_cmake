cmake_minimum_required(VERSION 3.5)
project(SCARA_ROSVERS)

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

find_package(dynamixel_sdk REQUIRED)
find_package(dynamixel_sdk_custom_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

include_directories(include)

# Create Motor Package
add_executable(motor src/motor/motor_sub.cpp)
ament_target_dependencies(motor
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
  std_msgs
)

# Create SCARA Package
add_executable(SCARA src/SCARA/SCARA_pub.cpp)
ament_target_dependencies(SCARA 
  dynamixel_sdk_custom_interfaces
  dynamixel_sdk
  rclcpp
  std_msgs
)

install(TARGETS
  motor
  SCARA
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
