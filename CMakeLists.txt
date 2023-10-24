cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(extractData)

include(CheckCXXCompilerFlag)
#CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++14" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
#	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX0X)
	set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
	message(FATAL_ERROR "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Use a different C++ compiler.")
endif()

find_package(Eigen3)
IF(NOT EIGEN3_INCLUDE_DIR)
	set(EIGEN3_INCLUDE_DIR $ENV{EIGEN3_ROOT_DIR})
ENDIF()

include_directories( ./src ../izutu/utility/src ./include ~/creatFig/src ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIR} ../ssm/dssm-master/dssm-proxy/src/ ../ssm/dssm-master/PConnector/src/ ../ssm/include/)

# set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

# set the build type
set(CMAKE_BUILD_TYPE Release)
# confirmation messages
message(STATUS "CMAKE_BUILD_TYPE: ${CMAKE_BUILD_TYPE}")
message(STATUS "CMAKE_C_FLAGS: ${CMAKE_C_FLAGS}")
message(STATUS "CMAKE_C_FLAGS_RELEASE: ${CMAKE_C_FLAGS_RELEASE}")
message(STATUS "CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
message(STATUS "CMAKE_CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")


#add_executable(calcgeo ./src/calcgeo.cpp ./src/GnsstoMag.cpp)
#target_link_libraries(calcgeo m ssm)

#すべての点からoffsetX,Y and kを最小二乗法を用いて算出するプログラム
add_executable(estoff ./src/estoff.cpp ./src/GnsstoMag.cpp ~/creatFig/src/creatfig.cpp)
target_link_libraries(estoff m ssm)

add_executable(version4 ./src/version4.cpp ./src/GnsstoMag.cpp ~/creatFig/src/creatfig.cpp)
target_link_libraries(version4 m ssm)

#算出したoffsetやkを用いてoffsetZを算出するプログラム
add_executable(offsetz ./src/offsetz.cpp ./src/GnsstoMag.cpp ~/creatFig/src/creatfig.cpp)
target_link_libraries(offsetz m ssm)
add_executable(YtoZ ./src/YtoZ.cpp ./src/GnsstoMag.cpp ~/creatFig/src/creatfig.cpp)
target_link_libraries(YtoZ m ssm)
add_executable(XtoZ ./src/XtoZ.cpp ./src/GnsstoMag.cpp ~/creatFig/src/creatfig.cpp)
target_link_libraries(XtoZ m ssm)

#最小二乗法で求めた際のXYをグラフにプロットするプログラム
add_executable(xydata ./src/xydata.cpp ./src/GnsstoMag.cpp ~/creatFig/src/creatfig.cpp)
target_link_libraries(xydata m ssm)

add_executable(compyaw ./src/compyaw.cpp ~/creatFig/src/creatfig.cpp)
target_link_libraries(compyaw m ssm)

add_executable(select ./src/selectdata.cpp ~/creatFig/src/creatfig.cpp ./src/GnsstoMag.cpp)
target_link_libraries(select m ssm)

#datファイルからデータを読み込み平均を算出するプログラム
add_executable(calcave ./etc/calcave.cpp ~/creatFig/src/creatfig.cpp ./src/GnsstoMag.cpp)
target_link_libraries(calcave m ssm)

#yawdataを出力するプログラム
add_executable(yawdata ./src/yawdata.cpp)
target_link_libraries(yawdata m ssm)

#add_executable(calcgeover2 ./src/calcgeover2.cpp ./src/GnsstoMag.cpp ~/creatFig/src/creatfig.cpp)
#target_link_libraries(calcgeover2 m ssm)

#add_executable(GtoM ./src/Magcal.cpp ./src/GnsstoMag.cpp)
#target_link_libraries(GtoM m ssm)

#add_executable(eigen ./src/PracEigen.cpp ./src/GnsstoMag.cpp)
#target_link_libraries(eigen m ssm)

add_executable(callean ./etc/callean.cpp ~/creatFig/src/creatfig.cpp)

add_executable(test ./src/test.cpp ~/creatFig/src/creatfig.cpp)
target_link_libraries(test m ssm)
