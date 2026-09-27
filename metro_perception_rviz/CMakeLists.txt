cmake_minimum_required(VERSION 3.8)
project(metro_perception_rviz)
find_package(ament_cmake REQUIRED)
find_package(metro_perception_interfaces REQUIRED)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Text of an assessment, without Qt: shared by the panel and its test.
add_library(assessment_text src/assessment_text.cpp)
target_include_directories(assessment_text PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include> $<INSTALL_INTERFACE:include>)
set_target_properties(assessment_text PROPERTIES POSITION_INDEPENDENT_CODE ON)
ament_target_dependencies(assessment_text metro_perception_interfaces)
install(TARGETS assessment_text ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)

# The dev image without RViz and the headless runtime build the text library only.
find_package(rviz_common QUIET)
if(rviz_common_FOUND)
  find_package(pluginlib REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(Qt5 REQUIRED COMPONENTS Widgets)
  set(CMAKE_AUTOMOC ON)
  add_library(assessment_panel SHARED src/assessment_panel.cpp src/assessment_panel.hpp)
  target_link_libraries(assessment_panel assessment_text Qt5::Widgets)
  ament_target_dependencies(assessment_panel metro_perception_interfaces pluginlib rclcpp
    rviz_common)
  install(TARGETS assessment_panel ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)
  pluginlib_export_plugin_description_file(rviz_common plugins_description.xml)
else()
  message(STATUS "rviz_common not found: the RViz assessment panel is not built")
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_assessment_text test/test_assessment_text.cpp)
  target_link_libraries(test_assessment_text assessment_text)
endif()

ament_package()
