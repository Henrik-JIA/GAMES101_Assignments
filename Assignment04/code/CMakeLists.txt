cmake_minimum_required(VERSION 3.10)
project(Rasterizer VERSION 0.1 LANGUAGES CXX)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# OpenCV 配置
set(OpenCV_DIR D:/ThirdPackage/MinGW1120/opencv4.10/x64/mingw/lib)
find_package(OpenCV REQUIRED)

# 包含头文件目录
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories("D:/ThirdPackage/MinGW1120/eigen-3.4.0")

# 查找所有源文件
file(GLOB_RECURSE SOURCES
    ${CMAKE_SOURCE_DIR}/*.cpp
    ${CMAKE_SOURCE_DIR}/*.hpp
)
list(FILTER SOURCES EXCLUDE REGEX "/CMakeFiles/")

# 添加可执行文件
add_executable(Rasterizer ${SOURCES})
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})