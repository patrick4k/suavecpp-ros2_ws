cmake_minimum_required(VERSION 3.8)
project(suave_launch)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Run this on any executable to ensure correct installation
macro(include_compile_install exec)
  target_include_directories(${exec} PUBLIC
          $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
          $<INSTALL_INTERFACE:include>)
  target_compile_features(${exec} PUBLIC c_std_99 cxx_std_17)  # Require C99 and C++17

  message(${exec})
  install(TARGETS ${exec}
          DESTINATION lib/${PROJECT_NAME})
endmacro()

# DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(MAVSDK REQUIRED)

# SUAVE_MAIN
add_executable(suave_main src/suave_main.cpp
        src/vio/
        src/common
        src/util/Util.cpp
        src/util/Util.h
        src/vio/MavVIOBridge.h
        src/vio/MavVIOBridge.cpp
        src/common/IMavController.h
        src/nav/TakeoffLandFlightPlan.cpp
        src/nav/TakeoffLandFlightPlan.h
        src/common/TaskResult.h
        src/ros/IRosController.h
        src/ros/RosSubscriber.cpp
        src/ros/RosSubscriber.h
        src/vio/VIOBridge.cpp
        src/vio/VIOBridge.h
        src/common/ISuaveController.h
        src/nav/SuaveOffboard.cpp
        src/nav/SuaveOffboard.h
        src/util/Result.h
        src/util/Timer.h
        src/common/ITask.h
        src/ros/RosNodeSpinner.cpp
        src/ros/RosNodeSpinner.h
        src/mavutil/Drone.cpp
        src/mavutil/Drone.h
        src/controllers/SuaveVIOTestFlight.cpp
        src/controllers/SuaveVIOTestFlight.h
        src/mavutil/MavUtil.h
        src/mavutil/MavUtil.cpp
        src/common/SystemTask.cpp
        src/common/SystemTask.h
)
ament_target_dependencies(suave_main MAVSDK)
ament_target_dependencies(suave_main rclcpp)
ament_target_dependencies(suave_main nav_msgs)
target_link_libraries(suave_main
        MAVSDK::mavsdk
)
include_compile_install(suave_main)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
