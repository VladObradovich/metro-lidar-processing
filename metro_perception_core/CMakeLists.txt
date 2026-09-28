cmake_minimum_required(VERSION 3.8)
project(metro_perception_core)
find_package(ament_cmake REQUIRED)
add_library(${PROJECT_NAME} SHARED src/pipeline.cpp src/detector.cpp src/speed_tracker.cpp
  src/temporal_monitor.cpp src/labels.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_options(${PROJECT_NAME} PRIVATE -Wall -Wextra -Wpedantic)
install(TARGETS ${PROJECT_NAME} EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib LIBRARY DESTINATION lib RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pipeline test/test_pipeline.cpp)
  target_link_libraries(test_pipeline ${PROJECT_NAME})
  ament_add_gtest(test_temporal_monitor test/test_temporal_monitor.cpp)
  target_link_libraries(test_temporal_monitor ${PROJECT_NAME})
  ament_add_gtest(test_speed_tracker test/test_speed_tracker.cpp)
  target_link_libraries(test_speed_tracker ${PROJECT_NAME})
endif()
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_package()
