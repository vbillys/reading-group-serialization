#include <boost/program_options.hpp>
#include <iostream>
#include <npy.hpp>
#include <opencv2/highgui/highgui.hpp>

namespace po = boost::program_options;

int main(int argc, char **argv) {
  // Program options
  po::options_description desc("Read an image from a 2d matrix from python\n\n"
                               "Options");
  std::string filepath;
  desc.add_options()("help,h", "Print this help")(
      "filepath,f", po::value<std::string>(&filepath)->required(),
      "Set the input path for the data");
  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);
  std::cout << "opening: " << filepath << std::endl;

  std::vector<unsigned long> shape;
  std::vector<uint8_t> data;
  bool fortran_order;
  npy::LoadArrayFromNumpy(filepath, shape, fortran_order, data);
  std::cout << "mat shape: ";
  for (const auto &s : shape) {
    std::cout << s << ",";
  }
  std::cout << std::endl;

  cv::Mat image(shape[0], shape[1], CV_8UC3, cv::Scalar(0, 0, 0));
  // see https://www.geeksforgeeks.org/display-numpy-array-in-fortran-order
  // for the ordering of data
  // by default numpy save in C-order (row-major)
  for (auto i = 0; i < shape[0]; i++) {
    for (auto j = 0; j < shape[1]; j++) {
      auto &p = image.at<cv::Vec3b>(i, j);

      p[0] = data[i * shape[1] * 3 + j * 3];
      p[1] = data[i * shape[1] * 3 + j * 3 + 1];
      p[2] = data[i * shape[1] * 3 + j * 3 + 2];
    }
  }

  cv::imshow("Image", image);
  cv::waitKey(0);
  return 0;
}