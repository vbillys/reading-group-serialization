#include "cnpy/cnpy.h"
#include <Eigen/Dense>
#include <boost/program_options.hpp>
#include <iostream>
#include <npy.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

namespace po = boost::program_options;

int main(int argc, char** argv)
{
    // Program options
    po::options_description desc("Read an image from a 2d matrix from python\n\n"
                                 "Options");
    std::string filepath;
    desc.add_options()("help,h", "Print this help")(
        "filepath,f", po::value<std::string>(&filepath)->required(), "Set the input path for the data");
    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);
    std::cout << "opening: " << filepath << std::endl;
    cnpy::npz_t npz = cnpy::npz_load(filepath);

    // --- Processing Normal (Numpy) Matrix
    auto numpyArray = npz["ht_matrix"];
    Eigen::MatrixXd eigenMat =
        Eigen::Map<Eigen::MatrixXd>(numpyArray.data<double>(), numpyArray.shape[1], numpyArray.shape[0]);
    // need the following if the matrix is stored with C-order
    // eigenMat.transposeInPlace();
    std::cout << "Numpy matrix (as Eigen matrix): " << std::endl << eigenMat << std::endl;

    // ---

    // Processing Image (OpenCV) Matrix

    cnpy::NpyArray npzImageArray = npz["img"];
    // need to know data type before hand
    // can be fixed with modifying the cnpy library
    // the typename is actually stored in the file.
    auto* imageData = npzImageArray.data<uint8_t>();

    std::cout << "image mat shape: ";
    for (const auto& s : npzImageArray.shape) {
        std::cout << s << ",";
    }
    std::cout << std::endl;

    cv::Mat image(npzImageArray.shape[0], npzImageArray.shape[1], CV_8UC3, cv::Scalar(0, 0, 0));
    // see https://www.geeksforgeeks.org/display-numpy-array-in-fortran-order
    // for the ordering of data
    // by default numpy save in C-order (row-major)
    // Note: there may be a way to avoid copy using constructor:
    //       cv::Mat(const std::vector< _Tp > &vec, bool copyData=false)
    for (auto i = 0; i < npzImageArray.shape[0]; i++) {
        for (auto j = 0; j < npzImageArray.shape[1]; j++) {
            auto& p = image.at<cv::Vec3b>(i, j);

            p[0] = imageData[i * npzImageArray.shape[1] * 3 + j * 3];
            p[1] = imageData[i * npzImageArray.shape[1] * 3 + j * 3 + 1];
            p[2] = imageData[i * npzImageArray.shape[1] * 3 + j * 3 + 2];
        }
    }

    cv::imshow("Image", image);
    cv::waitKey(0);

    // ---

    // --- Processing PointCloud (PCL) Matrix
    cnpy::NpyArray pointCloudArray = npz["pcd_numpy"];
    auto* pointCloudData = pointCloudArray.data<double>();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // pack r/g/b into rgb
    const std::uint8_t r = 0, g = 255, b = 0;
    std::uint32_t rgb = ((std::uint32_t)r << 16 | (std::uint32_t)g << 8 | (std::uint32_t)b);
    for (auto i = 0; i < pointCloudArray.shape[0]; i++) {
        // for (auto j = 0; j < pointCloudArray.shape[1]; j++) {
            pcl::PointXYZRGB point;
            point.x = pointCloudData[i*pointCloudArray.shape[1]];
            point.y = pointCloudData[i*pointCloudArray.shape[1]+1];
            point.z = pointCloudData[i*pointCloudArray.shape[1]+2];
            point.rgb  = *reinterpret_cast<float*>(&rgb);
        // }
        cloud->push_back(point);
    }

    pcl::visualization::CloudViewer viewer("Cloud Viewer");
    viewer.showCloud(cloud);
    while (!viewer.wasStopped()) {
    }

    // ---

    return 0;
}