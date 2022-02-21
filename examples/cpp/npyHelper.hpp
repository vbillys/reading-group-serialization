#pragma once
#include "npy.hpp"

namespace npy {

template<typename Scalar>
inline void SaveArrayToString(std::stringstream& stringstream,
                              bool fortran_order,
                              unsigned int n_dims,
                              const unsigned long shape[],
                              const Scalar* data)
{
    static_assert(has_typestring<Scalar>::value, "scalar type not understood");
    dtype_t dtype = has_typestring<Scalar>::dtype;

    std::vector<ndarray_len_t> shape_v(shape, shape + n_dims);
    header_t header{dtype, fortran_order, shape_v};
    write_header(stringstream, header);

    auto size = static_cast<size_t>(comp_size(shape_v));

    stringstream.write(reinterpret_cast<const char*>(data), sizeof(Scalar) * size);
}

template<typename Scalar>
inline void LoadArrayFromString(std::stringstream& stringstream,
                                std::vector<unsigned long>& shape,
                                bool& fortran_order,
                                std::vector<Scalar>& data)
{

    std::string header_s = read_header(stringstream);

    // parse header
    header_t header = parse_header(header_s);

    // check if the typestring matches the given one
    static_assert(has_typestring<Scalar>::value, "scalar type not understood");

    if (header.dtype.tie() != has_typestring<Scalar>::dtype.tie()) {
        throw std::runtime_error("formatting error: typestrings not matching");
    }

    shape = header.shape;
    fortran_order = header.fortran_order;

    // compute the data size based on the shape
    auto size = static_cast<size_t>(comp_size(shape));
    data.resize(size);

    // read the data
    stringstream.read(reinterpret_cast<char*>(data.data()), sizeof(Scalar) * size);
}

} // namespace npy