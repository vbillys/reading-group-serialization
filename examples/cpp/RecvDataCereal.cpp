#include "cerealHelper.hpp"
#include <Eigen/Dense>
#include <boost/program_options.hpp>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

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
    po::options_description desc("Take data from binary archive and process\n\n"
                                 "Options");
    std::string cerealpath;
    desc.add_options()("help,h", "Print this help")("filecereal,f",
                                                    po::value<std::string>(&cerealpath)->required(),
                                                    "Set the input path for the cereal binary file");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    std::ifstream dataFile(cerealpath, std::ifstream::out | std::ios::binary);
    if (dataFile.fail()) {
        throw std::runtime_error("cannot open cereal binary file!");
    }

    DataCollection dataCollection;
    cereal::BinaryInputArchive inputArchive(dataFile);
    inputArchive(CEREAL_NVP(dataCollection));

    // --- Processing Normal (Numpy) Matrix

    std::cout << "loaded Homogeneous Transformation matrix: " << std::endl
              << dataCollection.affine.matrix() << std::endl;

    // ---

    // --- Image (OpenCV)

    cv::imshow("Image", dataCollection.image);
    cv::waitKey(0);

    // ---

    // --- PointCloud (PCL)

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(dataCollection.cloud, *cloud);
    colorizeCloud(cloud);
    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {
    }

    // ---

    return 0;
}