#include "npyHelper.hpp"
#include <Eigen/Dense>
#include <boost/program_options.hpp>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "ExampleIdlData.hpp"

namespace po = boost::program_options;

static std::vector<unsigned char> toVector(std::stringstream& ss)
{
    // discover size of data in stream
    ss.seekg(0, std::ios::beg);
    auto bof = ss.tellg();
    ss.seekg(0, std::ios::end);
    auto stream_size = std::size_t(ss.tellg() - bof);
    ss.seekg(0, std::ios::beg);

    // make your vector long enough
    std::vector<unsigned char> v(stream_size);

    // read directly in
    ss.read((char*)v.data(), std::streamsize(v.size()));

    return v;
}

int main(int argc, char** argv)
{
    // Program options
    po::options_description desc("Read different data and send to python\n\n"
            "Options");
    std::string pcdpath, imgpath;
    desc.add_options()("help,h", "Print this help")(
            "pcdpath,p", po::value<std::string>(&pcdpath)->required(), "Set the input path for the .pcd")(
            "imgpath,i", po::value<std::string>(&imgpath)->required(), "Set the input path for the image");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // --- Processing Normal (Numpy) Matrix

    Eigen::Matrix3d rotMat;
    Eigen::Affine3d htMat{Eigen::Affine3d::Identity()};
    rotMat = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX());
    htMat.translate(Eigen::Vector3d(20, 30, 40));
    htMat.rotate(rotMat);

    auto htMatMatrix = htMat.matrix();
    // we need the following due to Eigen stores data as column-major
    htMatMatrix.transposeInPlace();
    double* htMatData = htMatMatrix.data();

    std::stringstream eigenStream;
    std::array<long unsigned, 2> matrixShape {{4,4}};
    npy::SaveArrayToString(eigenStream, false, 2,  matrixShape.data(), htMatData);
    ExampleIdlData::BytesArray payloadEigen = toVector(eigenStream);

    std::cout << "htMat: " << std::endl << htMat.matrix() << std::endl;

    // ---


    return 0;
}