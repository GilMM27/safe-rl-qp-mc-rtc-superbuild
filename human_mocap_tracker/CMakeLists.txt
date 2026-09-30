cmake_minimum_required(VERSION 3.20)

project(HumanMocapTracker LANGUAGES CXX)

include(CTest)

if(NOT DEFINED MUJOCO_ROOT_DIR)
  message(FATAL_ERROR "MUJOCO_ROOT_DIR is required to build HumanMocapTracker")
endif()

add_library(HumanMocapTracker STATIC src/HumanMocapTracker.cpp src/QposTrajectory.cpp)
target_compile_features(HumanMocapTracker PUBLIC cxx_std_17)
target_include_directories(
  HumanMocapTracker
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
    "${MUJOCO_ROOT_DIR}/include"
)

install(
  TARGETS HumanMocapTracker
  ARCHIVE DESTINATION lib
)
install(
  DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  add_executable(QposTrajectoryTest tests/QposTrajectoryTest.cpp)
  target_link_libraries(QposTrajectoryTest PRIVATE HumanMocapTracker)
  add_test(NAME QposTrajectoryTest COMMAND QposTrajectoryTest)
endif()
