#pragma once
#include <cereal/archives/binary.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/chrono.hpp>
#include <cereal/types/map.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/common/common.h>
#include <pcl/point_types.h>

namespace cereal {

template<class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline typename std::enable_if<traits::is_output_serializable<BinaryData<_Scalar>, Archive>::value, void>::type save(
    Archive& ar,
    const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix)
{
    const std::int32_t rows = static_cast<std::int32_t>(matrix.rows());
    const std::int32_t cols = static_cast<std::int32_t>(matrix.cols());
    ar(rows);
    ar(cols);
    ar(binary_data(matrix.data(), rows * cols * sizeof(_Scalar)));
}

template<class Archive, class _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
inline typename std::enable_if<traits::is_input_serializable<BinaryData<_Scalar>, Archive>::value, void>::type load(
    Archive& ar,
    Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols>& matrix)
{
    std::int32_t rows;
    std::int32_t cols;
    ar(rows);
    ar(cols);

    matrix.resize(rows, cols);

    ar(binary_data(matrix.data(),
                   static_cast<std::size_t>(static_cast<size_t>(rows) * static_cast<size_t>(cols) * sizeof(_Scalar))));
}

template<class Archive>
void save(Archive& archive, Eigen::Affine3d const& m)
{
    archive(m.matrix());
}

template<class Archive>
void load(Archive& archive, Eigen::Affine3d& m)
{
    Eigen::Matrix4d mat;
    archive(mat);
    m.matrix() = mat;
}

template<class Archive>
void save(Archive& ar, const cv::Mat& mat)
{
    int rows, cols, type;
    bool continuous;

    rows = mat.rows;
    cols = mat.cols;
    type = mat.type();
    continuous = mat.isContinuous();

    ar& rows& cols& type& continuous;

    if (continuous) {
        const int data_size = rows * cols * mat.elemSize();
        auto mat_data = cereal::binary_data(mat.ptr(), data_size);
        ar& mat_data;
    } else {
        const int row_size = cols * mat.elemSize();
        for (int i = 0; i < rows; i++) {
            auto row_data = cereal::binary_data(mat.ptr(i), row_size);
            ar& row_data;
        }
    }
}

template<class Archive>
void load(Archive& ar, cv::Mat& mat)
{
    int rows, cols, type;
    bool continuous;

    ar& rows& cols& type& continuous;

    if (continuous) {
        mat.create(rows, cols, type);
        const int data_size = rows * cols * mat.elemSize();
        auto mat_data = cereal::binary_data(mat.ptr(), data_size);
        ar& mat_data;
    } else {
        mat.create(rows, cols, type);
        const int row_size = cols * mat.elemSize();
        for (int i = 0; i < rows; i++) {
            auto row_data = cereal::binary_data(mat.ptr(i), row_size);
            ar& row_data;
        }
    }
}

template<class Archive>
void save(Archive& archive, pcl::PointCloud<pcl::PointXYZI> const& m)
{
    archive(m.size());
    for (const auto& p : m) {
        archive(p.x, p.y, p.z, p.intensity);
    }
}

template<class Archive>
void load(Archive& archive, pcl::PointCloud<pcl::PointXYZI>& m)
{
    size_t noOfPoints;
    archive(noOfPoints);
    for (size_t i = 0; i < noOfPoints; i++) {
        pcl::PointXYZI p;
        archive(p.x, p.y, p.z, p.intensity);
        m.push_back(p);
    }
}

} // namespace cereal