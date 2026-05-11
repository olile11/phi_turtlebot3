cmake_minimum_required(VERSION 3.8)
project(phi_turtlebot3_linefollower)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)

ament_python_install_package(${PROJECT_NAME}
  PACKAGE_DIR scripts
)

# line2.py é o nó ROS canônico (subscribe câmera, publish cmd_vel via
# bot_control). Instalado como executável `line_follower`.
install(PROGRAMS
  scripts/line2.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME line_follower
)

install(PROGRAMS
  scripts/robot_controller.py
  scripts/Line_follow.py
  DESTINATION lib/${PROJECT_NAME}
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

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
