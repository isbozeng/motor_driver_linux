cmake_minimum_required(VERSION 2.8.3) # 指定 CMake 最低版本要求

project(motor_driver) # 设置项目名称


# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 11)

include_directories(include)

link_directories(${PROJECT_SOURCE_DIR}/lib)


file(GLOB SOURCE_FILES "${PROJECT_SOURCE_DIR}/src/*.cpp")



# 添加可执行文件，指定源文件
add_executable(${PROJECT_NAME} jointTest.cpp ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME}  -lpthread  -lcontrolcan ) 