cmake_minimum_required(VERSION 3.16)
project(flexjoint_vs CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV 4.0 REQUIRED COMPONENTS core imgproc highgui videoio)
find_package(Threads REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(flexjoint_vs
    src/main.cpp
    src/serial_port.cpp
    src/kinematics.cpp
    src/controller.cpp
    src/vision.cpp
    src/modbus_crc.cpp
)

target_include_directories(flexjoint_vs PRIVATE include)

target_link_libraries(flexjoint_vs PRIVATE
    Eigen3::Eigen
    ${OpenCV_LIBS}
    Threads::Threads
    yaml-cpp
)

target_compile_options(flexjoint_vs PRIVATE -Wall -Wextra -Wno-unused-parameter)
