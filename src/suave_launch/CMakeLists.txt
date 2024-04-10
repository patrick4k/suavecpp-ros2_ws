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
#        src/vio/suave_vio_bridge.cpp
        src/vio/OdomGeometry.cpp
        src/vio/OdomGeometry.h
        src/util/Util.cpp
        src/util/Util.h
        src/vio/MavVIOBridge.h
        src/vio/MavVIOBridge.cpp
        src/nav/IFlightPlan.h
        src/nav/TakeoffLandFlightPlan.cpp
        src/nav/TakeoffLandFlightPlan.h)
ament_target_dependencies(suave_main MAVSDK)
ament_target_dependencies(suave_main rclcpp)
ament_target_dependencies(suave_main nav_msgs)
target_link_libraries(suave_main
        MAVSDK::mavsdk
)
include_compile_install(suave_main)

# SUAVE_VIO_BRIDGE
#add_executable(suave_vio_bridge src/vio/ src/vio/suave_vio_bridge.cpp
#        src/vio/OdomGeometry.cpp
#        src/vio/OdomGeometry.h
#        src/util/Util.cpp
#        src/util/Util.h)
#ament_target_dependencies(suave_vio_bridge rclcpp)
#ament_target_dependencies(suave_vio_bridge nav_msgs)
#ament_target_dependencies(suave_vio_bridge MAVSDK)
#target_link_libraries(suave_vio_bridge
#        MAVSDK::mavsdk
#)
#include_compile_install(suave_vio_bridge)

# ^^^ ADD NEW NODE HERE ^^^
#[[
add_executable(<node name> [source files...])
ament_target_dependencies(suave_vio_bridge <node dependency>)
target_link_libraries(suave_vio_bridge
        <node libraries to link>
)
include_compile_install(<node name>)
]]

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
