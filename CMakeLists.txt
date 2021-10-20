cmake_minimum_required(VERSION 3.17)
project(StereoCalibMatch)
set(CMAKE_CXX_STANDARD 11)

# 添加opencv环境
set(OpenCV_DIR E:/software/opencv454/build)

# 中文支持
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -fexec-charset=GBK")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
add_executable(StereoCalibMatch
        main.cpp
        srcs/ChessBoard.h
        srcs/Calibrator.h
        srcs/CoreBM.h
        srcs/utils.h
        srcs/config.h
)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})