## This is the repo for Reading Group (Cerealization for Prototyping)

### Some knowledge assumed
 * Basic programming with cpp / python
 * Common standard Libraries: Eigen, OpenCV, PCL
 * CMake build


### Dependencies (Ubuntu 20.04)
 * `libeigen3-dev`
 * `libopencv-dev`
 * `libpcl-dev`
 * `build-essential`
 * `python3-numpy`
 * `python3-opencv`
 * `git`
 * `cmake`
 * `bison` (for *cyclonedds*)
 * `openssl` (for *cyclonedds*)

### Repos used
 * [cereal](https://github.com/USCiLab/cereal)
 * [cyclonedds](https://github.com/eclipse-cyclonedds/cyclonedds)
 * [c++ binding for cyclonedds](https://github.com/eclipse-cyclonedds/cyclonedds-cxx)
 * [python binding for cyclonedds](https://github.com/eclipse-cyclonedds/cyclonedds-python)

### Getting started
 * clone the this repo and its submodule, we assume we start at $HOME
 ```
 cd
 git clone --recurse-submodules https://github.com/vbillys/reading-group-serialization
 export REPO_FOLDER=$HOME/reading-group-serialization
 ```
 * build cyclonedds and its bindings, in `$BUILD_FOLDER`, and put it in `$DDS_INSTALL_PATH` (these locations can be customized)
 ```
 export DDS_INSTALL_PATH=$REPO_FOLDER/install
 mkdir -p $DDS_INSTALL_PATH
 export BUILD_FOLDER=$REPO_FOLDER/build
 mkdir -p $BUILD_FOLDER
 export BUILD_FOLDER_CPP=$REPO_FOLDER/buildcpp
 mkdir -p $BUILD_FOLDER_CPP

 cd $BUILD_FOLDER && cmake -DCMAKE_INSTALL_PREFIX=$DDS_INSTALL_PATH $REPO_FOLDER/cyclonedds && make -j2 && make install
 cd $BUILD_FOLDER_CPP && cmake -DCMAKE_PREFIX_PATH=$DDS_INSTALL_PATH -DCMAKE_INSTALL_PREFIX=$DDS_INSTALL_PATH $REPO_FOLDER/cyclonedds-cxx && make -j2 && make install

 # for python binding we build using pip
 CMAKE_PREFIX_PATH=$DDS_INSTALL_PATH pip3 install git+https://github.com/eclipse-cyclonedds/cyclonedds-python
 ```
 * to clean up
 ```
 rm -rf $REPO_FOLDER
 pip3 uninstall cyclonedds
 ```

### To implement

 1. `cpp` to `cpp` , Eigen, cv::Mat or image, pcl PointCloud
 2. `cpp` to `python` , Eigen/Pointcloud to npy, and cv Image (.jpg)
 3. `python` to `cpp` , if simple use npy, or use no. 5
 4. other languages, rule of thumb, use object types that can be used directly to **minimize** boilerplates, otherwise, use no. 5
 5. using `middleware` with `schema` support (for online connections)

For no.1 to 3, give example using pipes and via binary-file
