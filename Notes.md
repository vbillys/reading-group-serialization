## This is the repo for Reading Group (Cerealization for Prototyping)

### Some knowledge assumed
 * Basic programming with cpp / python
 * Common standard Libraries: Eigen, OpenCV, PCL
 * CMake build
 * Git version control
 * Linux shell (`bash`)


### Dependencies (Tested on Ubuntu 20.04)
*Note*: other Ubuntu version may work, just need to pay attention to the versions, for example `libboostX.XX-dev` and `pythonX-* `
 * `libeigen3-dev`
 * `libboost1.71-dev`
 * `libopencv-dev`
 * `libpcl-dev`
 * `build-essential`
 * `python3-numpy`
 * `python3-opencv`
 * `python3-matplotlib`
 * `git`
 * `cmake`
 * `bison` (for *cyclonedds*)
 * `openssl` (for *cyclonedds*)
 * `open3d` python package (use `pip3 install open3d` to install)
 * `transforms3d` python package (use `pip3 install transfomrs3d` to install)

For convinience, we can install all the above using `bash`-single-liner-command:
```
# please make sure this will not overwrite your current work / environment setup
sudo apt install libeigen3-dev libboost1.71-dev libopencv-dev libpcl-dev build-essential python3-numpy python3-opencv python3-matplotlib git cmake bison openssl && pip3 install open3d && pip3 install transforms3d
```

### Repos used
 * [cereal](https://github.com/USCiLab/cereal)
 * [cyclonedds](https://github.com/eclipse-cyclonedds/cyclonedds)
 * [c++ binding for cyclonedds](https://github.com/eclipse-cyclonedds/cyclonedds-cxx)
 * [python binding for cyclonedds](https://github.com/eclipse-cyclonedds/cyclonedds-python)

### Getting started
 * clone the this repo and its submodule, we assume we start at $HOME
 ```
 cd # switch to home dir
 git clone --recurse-submodules https://github.com/vbillys/reading-group-serialization
 # change REPO_FOLDER if necessary
 export REPO_FOLDER=$HOME/reading-group-serialization
 ```
 * build cyclonedds and its bindings, in `$BUILD_FOLDER`, and put it in `$DDS_INSTALL_PATH` (these locations can be customized)
 ```
 export DDS_INSTALL_PATH=$REPO_FOLDER/install && \
     export BUILD_FOLDER=$REPO_FOLDER/build && \
     export BUILD_FOLDER_CPP=$REPO_FOLDER/buildcpp && \
     export BUILD_FOLDER_CEREAL=$REPO_FOLDER/buildcereal && \
     mkdir -p $DDS_INSTALL_PATH $BUILD_FOLDER $BUILD_FOLDER_CPP $BUILD_FOLDER_CEREAL

 cd $BUILD_FOLDER_CEREAL && cmake -DCMAKE_INSTALL_PREFIX=$DDS_INSTALL_PATH -DBUILD_TESTS=OFF $REPO_FOLDER/cereal && make -j2 && make install && \
     cd $BUILD_FOLDER && cmake -DCMAKE_INSTALL_PREFIX=$DDS_INSTALL_PATH $REPO_FOLDER/cyclonedds && make -j2 && make install && \
     cd $BUILD_FOLDER_CPP && cmake -DCMAKE_PREFIX_PATH=$DDS_INSTALL_PATH -DCMAKE_INSTALL_PREFIX=$DDS_INSTALL_PATH $REPO_FOLDER/cyclonedds-cxx && make -j2 && make install

 # for python binding we build using pip
 CMAKE_PREFIX_PATH=$DDS_INSTALL_PATH pip3 install git+https://github.com/eclipse-cyclonedds/cyclonedds-python
 ```
 * to build examples (CPP)  
   Note: If encounter problem with CMake version, it may happen that the system is using an older version of CMake (i.e. older Ubuntu). You can either try to upgrade it or change the first line of `CMakeLists.txt` - `cmake_minimum_required(VERSION 3.15)` to match the available CMake version.
 ```
 export EXAMPLES=$REPO_FOLDER/examples && \
     export BUILD_EXAMPLES=$EXAMPLES/build && \
     mkdir -p $BUILD_EXAMPLES && \
     cd $BUILD_EXAMPLES && cmake -DCMAKE_PREFIX_PATH=$DDS_INSTALL_PATH $EXAMPLES/cpp && make -j2 && make install
 ```
 * to run examples  
   Note: The python scripts has `shebang` reference to be executed with `python3` interpreter (the default in Ubuntu20+). If using older interpreter, please update (the first line of the scripts), or run the script with custom interpreter e.g. `python2 <script>`
 ```
 export EXAMPLES=$REPO_FOLDER/examples
 ```
 * to auto-generate idl codes (python and cxx)  
   Note: These generated files are included in the repo
 ```
 export EXAMPLES=$REPO_FOLDER/examples
 cd $EXAMPLES/python && LD_LIBRARY_PATH=$DDS_INSTALL_PATH/lib $DDS_INSTALL_PATH/bin/idlc -l py ExampleIdlData.idl
 cd $EXAMPLES/cpp && LD_LIBRARY_PATH=$DDS_INSTALL_PATH/lib $DDS_INSTALL_PATH/bin/idlc -l cxx ExampleIdlData.idl
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
