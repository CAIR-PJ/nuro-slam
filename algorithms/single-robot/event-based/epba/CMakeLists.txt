cmake_minimum_required(VERSION 3.16)
project(epba_ros2 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc highgui)
find_package(glog REQUIRED)
set(EPBA_SOPHUS_INCLUDE
  "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/basalt-headers/thirdparty/Sophus")

if(NOT EXISTS "${EPBA_SOPHUS_INCLUDE}/sophus/so3.hpp")
  message(FATAL_ERROR "Bundled Sophus headers missing")
endif()
set(EPBA_BASALT_DIR "${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/basalt-headers")
if(NOT EXISTS "${EPBA_BASALT_DIR}/include/basalt/spline/so3_spline.h")
  message(FATAL_ERROR "EPBA upstream Basalt headers missing: ${EPBA_BASALT_DIR}")
endif()
add_library(epba_native_core STATIC
  stage2/src/epba/model.cpp
  stage2/src/utils/trajectory.cpp
  stage2/src/utils/pose_manager.cpp
  stage2/src/utils/event_pano_warper.cpp
  stage2/src/utils/eigen_utils.cpp)
target_include_directories(epba_native_core PUBLIC
  stage2/include include ${OpenCV_INCLUDE_DIRS} ${EPBA_SOPHUS_INCLUDE}
  ${EPBA_BASALT_DIR}/include
  ${EPBA_BASALT_DIR}/thirdparty/Sophus
  ${EPBA_BASALT_DIR}/thirdparty/cereal/include)
target_link_libraries(epba_native_core PUBLIC Eigen3::Eigen ${OpenCV_LIBS} glog::glog)
if(TARGET Sophus::Sophus)
  target_link_libraries(epba_native_core PUBLIC Sophus::Sophus)
endif()
# Stage 3: native single-window Levenberg-Marquardt orchestration.
add_library(epba_native_lm STATIC stage3/src/epba/native_lm.cpp)
target_include_directories(epba_native_lm PUBLIC stage3/include stage2/include include)
target_link_libraries(epba_native_lm PUBLIC epba_native_core)
add_executable(epba_native_cli stage3/src/epba/native_cli.cpp)
target_link_libraries(epba_native_cli PRIVATE epba_native_lm)
install(TARGETS epba_native_lm epba_native_cli
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  add_test(NAME epba_native_cli_help COMMAND epba_native_cli --help)
endif()

add_executable(epba_input_probe src/native_input_probe.cpp)
target_include_directories(epba_input_probe PRIVATE include)
install(TARGETS epba_native_core epba_input_probe
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
  enable_testing()
  add_executable(epba_native_time_test test/native_time_test.cpp)
  target_include_directories(epba_native_time_test PRIVATE include)
  add_test(NAME epba_native_time_test COMMAND epba_native_time_test)
  add_executable(epba_native_camera_test test/native_camera_test.cpp)
  target_include_directories(epba_native_camera_test PRIVATE include ${OpenCV_INCLUDE_DIRS})
  target_link_libraries(epba_native_camera_test PRIVATE ${OpenCV_LIBS})
  add_test(NAME epba_native_camera_test COMMAND epba_native_camera_test)
  add_executable(epba_core_warper_test test/epba_core_warper_test.cpp)
  target_link_libraries(epba_core_warper_test PRIVATE epba_native_core)
  add_test(NAME epba_core_warper_test COMMAND epba_core_warper_test)
endif()
ament_package()


if(BUILD_TESTING)
  add_executable(epba_so3_initialization_test
    test/so3_initialization_test.cpp)
  target_include_directories(epba_so3_initialization_test PRIVATE
    stage3/include
    ${EPBA_SOPHUS_INCLUDE})
  target_link_libraries(epba_so3_initialization_test PRIVATE
    Eigen3::Eigen)
  add_test(NAME epba_so3_initialization_test
    COMMAND epba_so3_initialization_test)
endif()
