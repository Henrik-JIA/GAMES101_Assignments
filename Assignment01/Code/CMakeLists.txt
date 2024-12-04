cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

set(OpenCV_DIR "D:/ThirdPackage/MinGW-14.2.0/OpenCV-4.5.0-contrib")
find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories("D:/ThirdPackage/MinGW-14.2.0/Eigen-3.4.0/include/eigen3")

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})

