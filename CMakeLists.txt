cmake_minimum_required(VERSION 3.10)

set(CUDA_TOOLKIT_ROOT_DIR "/usr/local/cuda-9.2")
find_package(OpenCV REQUIRED)

find_package(yaml-cpp REQUIRED)

find_package(Threads REQUIRED)

add_library(common common.cpp camera.hpp calibrate.hpp)
target_link_libraries(common PUBLIC ${OpenCV_LIBRARIES})
target_include_directories(common PUBLIC ${OpenCV_INCLUDE_DIRS})
target_compile_features(common PUBLIC cxx_std_17)
target_link_libraries(common PUBLIC stdc++fs yaml-cpp)

add_executable(calibrate calibrate.cpp)
target_link_libraries(calibrate common)

add_executable(calibrate_fe calibrate_fe.cpp)
target_link_libraries(calibrate_fe common)

add_executable(take_calib_pics take_calib_pics.cpp)
target_link_libraries(take_calib_pics common)

add_executable(draw draw.cpp opengl_thread.cpp draw.hpp)
target_link_libraries(draw common Threads::Threads renderer_lib)

add_executable(undistort undistort.cpp)
target_link_libraries(undistort common)

add_subdirectory(rtk)
