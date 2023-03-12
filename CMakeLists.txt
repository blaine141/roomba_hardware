cmake_minimum_required(VERSION 3.8)
project(roomba_hardware)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


cmake_host_system_information(RESULT HOSTNAME QUERY HOSTNAME)

if(HOSTNAME STREQUAL "radxa-zero")
  add_executable(compass src/compass.cpp)
  target_link_libraries(compass -lmraa)
  ament_target_dependencies(compass rclcpp std_msgs geometry_msgs)

  install(TARGETS
    compass
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

# Install launch files.
install(DIRECTORY
  launch
  cfg
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
