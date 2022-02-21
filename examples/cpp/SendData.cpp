#include "cnpy/cnpy.h"
#include <Eigen/Dense>
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

static inline std::string cvTypeToStr(int type)
{
    std::string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth) {
        case CV_8U:
            r = "8U";
            break;
        case CV_8S:
            r = "8S";
            break;
        case CV_16U:
            r = "16U";
            break;
        case CV_16S:
            r = "16S";
            break;
        case CV_32S:
            r = "32S";
            break;
        case CV_32F:
            r = "32F";
            break;
        case CV_64F:
            r = "64F";
            break;
        default:
            r = "User";
            break;
    }

    r += "C";
    r += (chans + '0');

    return r;
}

int main(int argc, char** argv)
{
    // Program options
    po::options_description desc("Read an image from a 2d matrix from python\n\n"
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
    cnpy::npz_save("out.npz", "ht_matrix", htMatData, {4, 4}, "w");

    std::cout << "htMat: " << std::endl << htMat.matrix() << std::endl;

    // ---

    // Processing Image (OpenCV) Matrix

    auto image = cv::imread(imgpath);
    std::cout << "Image of size: " << image.rows << "x" << image.cols << " of type: " << cvTypeToStr(image.type())
              << std::endl;
    std::vector<uchar> imageArray;
    for (auto i = 0; i < image.rows; i++) {
        for (auto j = 0; j < image.cols; j++) {
            const auto& pixel = image.at<cv::Vec3b>(i, j);
            imageArray.push_back(pixel[0]);
            imageArray.push_back(pixel[1]);
            imageArray.push_back(pixel[2]);
        }
    }
    cnpy::npz_save("out.npz", "img", &imageArray[0], {image.rows, image.cols, 3}, "a");

    cv::imshow("image", image);
    cv::waitKey(0);

    // ---

    // --- Processing PointCloud (PCL) Matrix

    std::cout << "opening pcd: " << pcdpath << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcdpath, *cloud) == -1) {
        throw std::runtime_error("cannot load pcd");
    }
    colorizeCloud(cloud);

    std::vector<double> pointCloudArray;
    for (const auto& p : *cloud) {
        pointCloudArray.push_back(p.x);
        pointCloudArray.push_back(p.y);
        pointCloudArray.push_back(p.z);
    }
    cnpy::npz_save("out.npz", "pcd_numpy", &pointCloudArray[0], {cloud->size(), 3}, "a");

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {
    }

    // ---

    return 0;
}