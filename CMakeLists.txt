#定义需求版本和工程名称
cmake_minimum_required(VERSION 3.0.2)
project(my_slam_wiki)

#cmake 编译配置
SET(CMAKE_BUILD_TYPE "Debug")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")
add_compile_options(-std=c++14)
add_definitions(-std=c++14)

#第三方库配置
set(THIRD_PART_LIBRARIES "")
include(cmake/gtsam.cmake)

# include(cmake/pcl.cmake)
# include(cmake/geographicLib.cmake)
# include(cmake/glog.cmake)
# include(cmake/yamlcpp.cmake)
# include(cmake/g2o.cmake)



# #数据预处理节点
add_executable(main main.cpp ${ALL_SRCS} )
target_link_libraries(main  ${THIRD_PART_LIBRARIES})
