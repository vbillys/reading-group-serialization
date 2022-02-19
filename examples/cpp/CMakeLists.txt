cmake_minimum_required(VERSION 3.5)
project(ReadingGroupSerialization)

find_package(Boost REQUIRED COMPONENTS program_options)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(cereal REQUIRED)
find_package(CycloneDDS-CXX REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "RelWithDebInfo")

include_directories(${Eigen3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS}
                    ${PCL_INCLUDE_DIRS})

add_executable(receive_data_from_python ReceiveDataFromPython.cpp)
target_link_libraries(receive_data_from_python
                      ${PCL_LIBRARIES}
                      Boost::program_options
                      ${OpenCV_LIBRARIES}
                      cereal::cereal
                      CycloneDDS-CXX::ddscxx)
