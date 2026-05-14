cmake_minimum_required(VERSION 3.16)
project(flexjoint_vs CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(OpenCV 4.0 REQUIRED COMPONENTS core imgproc highgui videoio)
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

add_executable(flexjoint_vs
    src/main.cpp
    ${SERIAL_PORT_SOURCE}
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
    ${YAML_CPP_TARGET}
)

if(MSVC)
    target_compile_options(flexjoint_vs PRIVATE /W4)
else()
    target_compile_options(flexjoint_vs PRIVATE -Wall -Wextra -Wno-unused-parameter)
endif()
