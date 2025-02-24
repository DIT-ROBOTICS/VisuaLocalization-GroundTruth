cmake_minimum_required(VERSION 3.8)
project(gstcam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(yaml-cpp REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED gstreamer-1.0)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${GSTREAMER_INCLUDE_DIRS})
link_directories(${GSTREAMER_LIBRARY_DIRS})
add_definitions(${GSTREAMER_CFLAGS_OTHER})
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(gst_video_publisher src/gst_video_publisher.cpp)
ament_target_dependencies(gst_video_publisher rclcpp sensor_msgs cv_bridge)
target_link_libraries(gst_video_publisher ${OpenCV_LIBRARIES} gstapp-1.0 gstreamer-1.0 gobject-2.0 glib-2.0 yaml-cpp)

install(TARGETS gst_video_publisher DESTINATION lib/${PROJECT_NAME})

ament_package()
