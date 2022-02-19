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

### Repos used
 * [cereal](https://github.com/USCiLab/cereal)
 * [cyclonedds](https://github.com/eclipse-cyclonedds/cyclonedds)
 * [c++ binding for cyclonedds](https://github.com/eclipse-cyclonedds/cyclonedds-cxx)
 * [python binding for cyclonedds](https://github.com/eclipse-cyclonedds/cyclonedds-python)

### To implement

 1. `cpp` to `cpp` , Eigen, cv::Mat or image, pcl PointCloud
 2. `cpp` to `python` , Eigen/Pointcloud to npy, and cv Image (.jpg)
 3. `python` to `cpp` , if simple use npy, or use no. 5
 4. other languages, rule of thumb, use object types that can be used directly to **minimize** boilerplates, otherwise, use no. 5
 5. using `middleware` with `schema` support (for online connections)

For no.1 to 3, give example using pipes and via binary-file
