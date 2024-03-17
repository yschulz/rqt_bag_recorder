cmake_minimum_required(VERSION 3.8)
project(rqt_bag_recorder)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_INCLUDE_CURRENT_DIR ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(qt_gui_cpp REQUIRED)
find_package(rqt_gui_cpp REQUIRED)
find_package(Qt5Widgets REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(libstatistics_collector REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_msgs REQUIRED)

set(rqt_bag_recorder_SRCS
  src/rqt_bag_recorder.cpp
  src/bag_recorder_widgets.cpp
)

set(rqt_bag_recorder_HDRS
  include/rqt_bag_recorder/rqt_bag_recorder.hpp
  include/rqt_bag_recorder/bag_recorder_widgets.hpp
)

set(rqt_bag_recorder_UIS
  resource/rqt_bag_recorder.ui
)

qt5_wrap_cpp(rqt_bag_recorder_MOCS ${rqt_bag_recorder_HDRS})

qt5_wrap_ui(rqt_bag_recorder_UIS_H ${rqt_bag_recorder_UIS})


add_library(${PROJECT_NAME} SHARED
  ${rqt_bag_recorder_SRCS}
  ${rqt_bag_recorder_MOCS}
  ${rqt_bag_recorder_UIS_H}
)

target_link_libraries(${PROJECT_NAME}
  ${rclcpp_TARGETS}
  ${qt_gui_cpp_TARGETS}
  ${rqt_gui_cpp_TARGETS}
  ${rosbag2_cpp_TARGETS}
  ${libstatistics_collector_TARGETS}
  Qt5::Widgets
  ${YAML_CPP_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib/${PROJECT_NAME}
  LIBRARY DESTINATION lib/${PROJECT_NAME}
  RUNTIME DESTINATION bin/${PROJECT_NAME})

  install(PROGRAMS scripts/rqt_bag_recorder
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(FILES plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY resource
  DESTINATION share/${PROJECT_NAME}
)


add_executable(writer_test src/test_bag_rec.cpp)
ament_target_dependencies(writer_test
  rclcpp
  rosbag2_cpp
  std_msgs
)

install(TARGETS
    writer_test
  	DESTINATION lib/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(rqt_gui "plugin.xml")

# Export old-style CMake variables
ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(${PROJECT_NAME})

# Export new-style CMake variables
ament_export_targets(${PROJECT_NAME})


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()