# CMakeList.txt: 顶层 CMake 项目文件，在此处执行全局配置
# 并包含子项目。
cmake_minimum_required (VERSION 3.12)

project (balance_ctrl)

## aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

set(ZMQ_LIB ${CMAKE_CURRENT_SOURCE_DIR}/cppzmq)

## 动态库生成路径
set(LIB_PATH ${PROJECT_BINARY_DIR})

set(HEAD_PATH "${CMAKE_CURRENT_SOURCE_DIR}/include")

## set lib name
set(LIB_NAME src)
set(LIB_SOURCE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/src)

## set exe name
set(EXE_NAME demo)

add_subdirectory(src)
add_subdirectory(demo)
