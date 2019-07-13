SET (PROJECT_NAME imuSimWithPointLine)

PROJECT(${PROJECT_NAME})
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

# SET(CMAKE_BUILD_TYPE Debug) # Release, RelWithDebInfo , Debug

SET(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
SET(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")
SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake_modules/")

find_package(Eigen3 REQUIRED)
FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)

include_directories(
${EIGEN3_INCLUDE_DIR}
${SOPHUS_INCLUDE_DIR} # for both sophus and geographiclib
${OPENCV_INCLUDE_DIR}
)

LIST(APPEND LINK_LIBS
${OpenCV_LIBS}
${Sophus_LIBRARIES}
)

ADD_EXECUTABLE(data_gen main/gener_alldata.cpp src/param.h src/param.cpp src/utilities.h src/utilities.cpp src/imu.h src/imu.cpp)
TARGET_LINK_LIBRARIES (data_gen ${LINK_LIBS})

