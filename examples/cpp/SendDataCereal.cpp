#include "cerealHelper.hpp"
#include <Eigen/Dense>
#include <boost/program_options.hpp>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <thread>

namespace po = boost::program_options;

struct DataCollection
{
    Eigen::Affine3d affine;
    cv::Mat image;
    pcl::PointCloud<pcl::PointXYZI> cloud;

    template<class Archive>
    void serialize(Archive& archive)
    {
        archive(affine, image, cloud);
    }
};

void colorizeCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    // pack r/g/b into rgb
    std::uint8_t r = 0, g = 255, b = 0;
    std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
    for (auto& p : *cloud) {
        p.rgb = *reinterpret_cast<float*>(&rgb);
    }
}


int main(int argc, char** argv)
{
    // Program options
    po::options_description desc("Create different data and send to cereal binary archive\n\n"
                                 "Options");
    std::string pcdpath, imgpath;
    desc.add_options()("help,h", "Print this help")(
        "pcdpath,p", po::value<std::string>(&pcdpath)->required(), "Set the input path for the .pcd")(
        "imgpath,i", po::value<std::string>(&imgpath)->required(), "Set the input path for the image");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    DataCollection dataCollection;

    // --- Processing Normal (Numpy) Matrix

    Eigen::Matrix3d rotMat;
    dataCollection.affine = Eigen::Affine3d::Identity();
    rotMat = Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitZ()) *
             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitY()) *
             Eigen::AngleAxisd(M_PI / 2, Eigen::Vector3d::UnitX());
    dataCollection.affine.translate(Eigen::Vector3d(20, 30, 40));
    dataCollection.affine.rotate(rotMat);

    std::cout << "htMat: " << std::endl << dataCollection.affine.matrix() << std::endl;

    // ---

    // --- Image (OpenCV)

    std::cout << "opening image: " << imgpath << std::endl;
    dataCollection.image = cv::imread(imgpath);
    cv::imshow("image", dataCollection.image);
    cv::waitKey(0);

    // ---

    // --- PointCloud (PCL)

    std::cout << "opening pcd: " << pcdpath << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcdpath, *cloud) == -1) {
        throw std::runtime_error("cannot load pcd");
    }
    colorizeCloud(cloud);
    pcl::copyPointCloud(*cloud, dataCollection.cloud);

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {
    }

    // ---

    // Write output

    std::ofstream outfile("cereal.bin", std::ifstream::out | std::ios::binary);
    cereal::BinaryOutputArchive outputArchive(outfile);
    outputArchive(CEREAL_NVP(dataCollection));

    return 0;
}