cmake_minimum_required(VERSION 3.8)
project(phi_turtlebot3_exploration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR scripts
)

install(PROGRAMS
  scripts/potential_field.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME potential_field
)

install(DIRECTORY include/
  DESTINATION include/
)

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/launch")
  install(DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}/
  )
endif()

ament_package()
