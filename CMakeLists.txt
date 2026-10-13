cmake_minimum_required(VERSION 3.8)
project(ur10_proxskin_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

add_library(arm_control_lib SHARED
  src/arm_function.cpp
)
target_include_directories(arm_control_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(arm_control_lib
  rclcpp
  std_msgs
  sensor_msgs
)

add_executable(proxskin_test src/proxskin_test.cpp)
target_link_libraries(proxskin_test
  arm_control_lib
)
ament_target_dependencies(proxskin_test
  rclcpp
  std_msgs
  sensor_msgs
)

add_executable(proxskin_vertical_test src/proxskin_vertical_test.cpp)
target_link_libraries(proxskin_vertical_test
  arm_control_lib
)
ament_target_dependencies(proxskin_vertical_test
  rclcpp
  std_msgs
  sensor_msgs
)

install(TARGETS
  arm_control_lib
  proxskin_test
  proxskin_vertical_test
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(PROGRAMS scripts/web_control.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME web_control
)

install(FILES control.html how_to_use.html how_to_use.txt
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_executable(test_experiment_trajectory test/test_experiment_trajectory.cpp)
  add_test(NAME experiment_trajectory COMMAND test_experiment_trajectory)
  add_executable(test_pose_alignment test/test_pose_alignment.cpp)
  add_test(NAME pose_alignment COMMAND test_pose_alignment)
  add_executable(test_experiment_csv test/test_experiment_csv.cpp)
  add_test(NAME experiment_csv COMMAND test_experiment_csv)
  add_executable(test_ecan_frame test/test_ecan_frame.cpp)
  add_test(NAME ecan_frame COMMAND test_ecan_frame)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME experiment_launch COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_experiment_launch.py)
  add_test(NAME web_control COMMAND ${Python3_EXECUTABLE}
    ${CMAKE_CURRENT_SOURCE_DIR}/test/test_web_control.py)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
