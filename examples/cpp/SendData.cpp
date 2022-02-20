#include "cnpy/cnpy.h"
#include <boost/program_options.hpp>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

namespace po = boost::program_options;

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
    po::options_description desc("Read an image from a 2d matrix from python\n\n"
                                 "Options");
    std::string pcdpath;
    desc.add_options()("help,h", "Print this help")(
        "pcdpath,p", po::value<std::string>(&pcdpath)->required(), "Set the input path for the data");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    std::cout << "opening pcd: " << pcdpath << std::endl;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcdpath, *cloud) == -1) {
        throw std::runtime_error("cannot load pcd");
    }
    colorizeCloud(cloud);

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {
    }

    return 0;
}