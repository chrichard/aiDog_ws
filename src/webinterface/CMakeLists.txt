cmake_minimum_required(VERSION 3.8)
project(webinterface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
#find_package(OpenCV 4 REQUIRED)


set(protocol_DIR "/home/chen/cyberdog_ws/install/protocol/share/protocol")

find_package(protocol REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
MESSAGE(STATUS "OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
MESSAGE(STATUS "OPENCV_LIBRARY_DIRS = ${OpenCV_LIBRARIES}")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

aux_source_directory(./src SOURCES)

#add_executable(webinterface ${SOURCES})
add_executable(action src/pubdef.cpp src/action.cpp)
ament_target_dependencies(action rclcpp std_msgs sensor_msgs protocol cv_bridge image_transport  OpenCV)

install(TARGETS
  action
  DESTINATION lib/${PROJECT_NAME})

ament_package()
