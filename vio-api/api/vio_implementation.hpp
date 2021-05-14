#ifndef VIO_IMPLEMENTATION_HPP
#define VIO_IMPLEMENTATION_HPP

#include <string>
#include "vio.hpp"

namespace api {

std::unique_ptr<api::VioApi> buildVio(std::istream &calibrationFile, std::istream &configFile);

} // namespace api

#endif // VIO_API_HPP
