cmake_minimum_required(VERSION 3.5)
project(carla_video_decode)

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
find_package(sensor_msgs REQUIRED)

# Find ffmpeg libraries
find_package(PkgConfig  REQUIRED)
set(ENV{PKG_CONFIG_PATH} "/opt/ffmpeg/lib/pkgconfig")
pkg_search_module(PC_AVFORMAT REQUIRED libavformat)
pkg_search_module(PC_AVCODEC REQUIRED libavcodec)
pkg_search_module(PC_AVUTIL REQUIRED libavutil)
pkg_search_module(PC_SWSCALE REQUIRED libswscale)

add_executable(
  video_dec_node
  src/video_dec_node.cpp
  src/video_decoder.cpp
  src/pixfmt_convert.cpp
)

target_include_directories(video_dec_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  video_dec_node
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
)

include_directories(
  ${PC_AVFORMAT_INCLUDE_DIRS}
  ${PC_AVCODEC_INCLUDE_DIRS}
  ${PC_AVUTIL_INCLUDE_DIRS}
  ${PC_SWSCALE_INCLUDE_DIRS}
)

target_link_libraries(
  video_dec_node
  ${PC_AVFORMAT_LDFLAGS}
  ${PC_AVCODEC_LDFLAGS}
  ${PC_AVUTIL_LDFLAGS}
  ${PC_SWSCALE_LDFLAGS}
)

install(TARGETS video_dec_node
  DESTINATION lib/${PROJECT_NAME})

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

ament_package()
