cmake_minimum_required(VERSION 3.8)
project(orb_slam3_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
add_definitions(-w) # Supress warnings

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)

#------------------------------ ORB SLAM3 thingies ------------------------------------------
set(ORB_PATH ~/ORB_SLAM3) ######### << Change this to your ORB_SLAM3 path

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${ORB_PATH}
  ${ORB_PATH}/include
  ${ORB_PATH}/include/CameraModels
  ${ORB_PATH}/Thirdparty/Sophus/sophus 
  ${Pangolin_INCLUDE_DIRS}
)

set(LIBS 
${OpenCV_LIBS} 
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
${ORB_PATH}/Thirdparty/DBoW2/lib/libDBoW2.so
${ORB_PATH}/Thirdparty/g2o/lib/libg2o.so
${ORB_PATH}/lib/libORB_SLAM3.so
-lboost_system
)

#------------------------- Exec -----------------------------
add_executable(orb_slam3_ros2 src/orb_slam3_ros2.cpp)
ament_target_dependencies(orb_slam3_ros2 rclcpp cv_bridge image_transport nav_msgs geometry_msgs)
target_link_libraries(orb_slam3_ros2
  ${LIBS}
)

#----------------------- Install -----------------------------
install(TARGETS
  orb_slam3_ros2
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files.
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

# Install config files.
install(DIRECTORY
  config
  DESTINATION lib/${PROJECT_NAME}/
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}/
)


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
