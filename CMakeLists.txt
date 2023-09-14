# CMakeList.txt: 顶层 CMake 项目文件，在此处执行全局配置
# 并包含子项目。

cmake_minimum_required (VERSION 3.12)

project (triplependulum)

## aris use c++ 17 standard
set(CMAKE_CXX_STANDARD 17)

## find aris
if(UNIX)
	add_definitions(-DUNIX)
    find_package(aris REQUIRED PATHS /usr/aris)
endif(UNIX)
IF(WIN32)
    find_package(aris REQUIRED PATHS C:/aris/aris-2.3.5.221111)
    find_package(sire REQUIRED PATHS C:/sire/sire-1.0)
endif(WIN32)

## find Eigen
set(TARGET_EIGEN_PATH "" CACHE PATH "Eigen3 install path")
if(EXISTS ${TARGET_EIGEN_PATH})
	message(STATUS "Directory to search Eigen3 at ${TARGET_EIGEN_PATH}")
	list(APPEND CMAKE_PREFIX_PATH ${TARGET_EIGEN_PATH})
else()
	message(WARNING "File/Directory at variable TARGET_EIGEN_PATH not exists!")
endif()
find_package(Eigen3 REQUIRED)

include_directories(${aris_INCLUDE_DIRS})
include_directories(${Eigen3_INCLUDE_DIRS})

add_executable (triplependulum  "src/main.cpp" "src/model.cpp" "src/plan.cpp" "src/zmqmsg.cpp" "src/statcal_util.cpp" "src/pid.cpp")

target_include_directories( triplependulum PUBLIC ${PROJECT_SOURCE_DIR}/lib)
target_include_directories( triplependulum PUBLIC ${PROJECT_SOURCE_DIR}/cppzmq)

## 将需要的库链接到 triplependulum 中
target_link_libraries(triplependulum ${aris_LIBRARIES})
target_link_libraries(triplependulum Eigen3::Eigen)
target_link_libraries(triplependulum ${PROJECT_SOURCE_DIR}/lib/libzmq.lib)
target_link_libraries(triplependulum ${PROJECT_SOURCE_DIR}/lib/libzmq-v142-mt-4_3_4.lib)


add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to MyTest
    COMMAND ${CMAKE_COMMAND} -E copy  # which executes "cmake - E copy_if_different..."
        "${aris_DIR}/../../../lib/${CMAKE_BUILD_TYPE}/aris_lib.dll"      # <--this is in-file
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)                 # <--this is out-file path

add_custom_command(TARGET ${PROJECT_NAME} POST_BUILD        # Adds a post-build event to MyTest
    COMMAND ${CMAKE_COMMAND} -E copy  # which executes "cmake - E copy_if_different..."
        "${PROJECT_SOURCE_DIR}/lib/libzmq-v142-mt-4_3_4.dll"    # <--this is in-file
        "${PROJECT_SOURCE_DIR}/lib/libzmq.dll" 
        $<TARGET_FILE_DIR:${PROJECT_NAME}>)                 # <--this is out-file path