#include "cnpy/cnpy.h"
#include <boost/program_options.hpp>
#include <iostream>
#include <npy.hpp>
#include <opencv2/highgui/highgui.hpp>

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
    cnpy::NpyArray npzImageArray = npz["img"];
    // need to know data type before hand
    // can be fixed with modifying the cnpy library
    // the typename is actually stored in the file.
    auto* data = npzImageArray.data<uint8_t>();

    std::cout << "mat shape: ";
    for (const auto& s : npzImageArray.shape) {
        std::cout << s << ",";
    }
    std::cout << std::endl;

    cv::Mat image(npzImageArray.shape[0], npzImageArray.shape[1], CV_8UC3, cv::Scalar(0, 0, 0));
    // see https://www.geeksforgeeks.org/display-numpy-array-in-fortran-order
    // for the ordering of data
    // by default numpy save in C-order (row-major)
    for (auto i = 0; i < npzImageArray.shape[0]; i++) {
        for (auto j = 0; j < npzImageArray.shape[1]; j++) {
            auto& p = image.at<cv::Vec3b>(i, j);

            p[0] = data[i * npzImageArray.shape[1] * 3 + j * 3];
            p[1] = data[i * npzImageArray.shape[1] * 3 + j * 3 + 1];
            p[2] = data[i * npzImageArray.shape[1] * 3 + j * 3 + 2];
        }
    }

    cv::imshow("Image", image);
    cv::waitKey(0);
    return 0;
}