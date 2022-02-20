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

add_subdirectory(cnpy)

include_directories(${Eigen3_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} libnpy/include)

add_executable(receive_data_from_python ReceiveDataFromPython.cpp)
target_link_libraries(receive_data_from_python
                      # ${PCL_LIBRARIES}
                      Boost::program_options
                      ${OpenCV_LIBRARIES}
                      cereal::cereal
                      cnpy)
# CycloneDDS-CXX::ddscxx)

add_executable(send_data SendData.cpp)

target_link_libraries(send_data ${PCL_LIBRARIES} Boost::program_options ${OpenCV_LIBRARIES} cereal::cereal cnpy)
