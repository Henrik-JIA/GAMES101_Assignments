cmake_minimum_required(VERSION 3.5)

project(vsin01 VERSION 0.1 LANGUAGES CXX)


set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# set(OpenCV_DIR D:/ThirdPackage/MinGW1120/opencv4.10/x64/mingw/lib)#库路径你自己换成你电脑上的
# .dll 动态(大多情况是编译成功后所需要的)、.lib静态(编译过程中所需要的)
set(OpenCV_DIR D:/ThirdPackage/MinGW1120/opencv4.10/x64/mingw/lib)
find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

# 包含 Eigen 头文件目录
include_directories("D:/ThirdPackage/MinGW1120/eigen-3.4.0")


# 查找项目中的所有 .cpp 源文件，排除 CMake 生成目录
file(GLOB_RECURSE SOURCES
    ${CMAKE_SOURCE_DIR}/*.cpp
    ${CMAKE_SOURCE_DIR}/*.hpp
)

# 手动排除掉 CMake 生成的 CMakeFiles 目录
list(FILTER SOURCES EXCLUDE REGEX "/CMakeFiles/")

# 将找到的文件添加到可执行文件中
add_executable(MYCODE ${SOURCES})

target_link_libraries(MYCODE PRIVATE ${OpenCV_LIBS})
