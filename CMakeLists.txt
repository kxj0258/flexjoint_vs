cmake_minimum_required(VERSION 3.16)
project(flexjoint_vs CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV 4.0 REQUIRED COMPONENTS core imgproc highgui videoio imgcodecs calib3d)
find_package(Threads REQUIRED)
set(YAML_CPP_TARGET yaml-cpp)
find_package(yaml-cpp REQUIRED)
if(WIN32)
    if(TARGET yaml-cpp::yaml-cpp)
        set(YAML_CPP_TARGET yaml-cpp::yaml-cpp)
    elseif(TARGET yaml-cpp)
        set(YAML_CPP_TARGET yaml-cpp)
    endif()
endif()

set(SERIAL_PORT_SOURCE src/serial_port_posix.cpp)
if(WIN32)
    set(SERIAL_PORT_SOURCE src/serial_port_win32.cpp)
endif()

add_library(flexjoint_common
    ${SERIAL_PORT_SOURCE}
    src/app_config.cpp
    src/camera_utils.cpp
    src/feature_detection.cpp
    src/motor_client.cpp
    src/kinematics.cpp
    src/controller.cpp
    src/vision.cpp
    src/modbus_crc.cpp
)

target_include_directories(flexjoint_common PUBLIC include)

target_link_libraries(flexjoint_common PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBS}
    Threads::Threads
    ${YAML_CPP_TARGET}
)

if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 9)
    target_link_libraries(flexjoint_common PUBLIC stdc++fs)
endif()

add_executable(flexjoint_vs src/main.cpp)
target_link_libraries(flexjoint_vs PRIVATE flexjoint_common)

add_executable(camera_feature_test src/camera_feature_test.cpp)
target_link_libraries(camera_feature_test PRIVATE flexjoint_common)

add_executable(camera_calibration src/camera_calibration.cpp)
target_link_libraries(camera_calibration PRIVATE flexjoint_common)

add_executable(camera_calibration_eval src/camera_calibration_eval.cpp)
target_link_libraries(camera_calibration_eval PRIVATE flexjoint_common)

add_executable(motor_test src/motor_test.cpp)
target_link_libraries(motor_test PRIVATE flexjoint_common)

if(MSVC)
    target_compile_options(flexjoint_common PRIVATE /W4)
    target_compile_options(flexjoint_vs PRIVATE /W4)
    target_compile_options(camera_feature_test PRIVATE /W4)
    target_compile_options(camera_calibration PRIVATE /W4)
    target_compile_options(camera_calibration_eval PRIVATE /W4)
    target_compile_options(motor_test PRIVATE /W4)
else()
    target_compile_options(flexjoint_common PRIVATE -Wall -Wextra -Wno-unused-parameter)
    target_compile_options(flexjoint_vs PRIVATE -Wall -Wextra -Wno-unused-parameter)
    target_compile_options(camera_feature_test PRIVATE -Wall -Wextra -Wno-unused-parameter)
    target_compile_options(camera_calibration PRIVATE -Wall -Wextra -Wno-unused-parameter)
    target_compile_options(camera_calibration_eval PRIVATE -Wall -Wextra -Wno-unused-parameter)
    target_compile_options(motor_test PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()
