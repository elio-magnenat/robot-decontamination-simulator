#ifndef PACMAN_CPP_JSON_UTIL_H
#define PACMAN_CPP_JSON_UTIL_H

#include "json.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>

#define UNUSED_ARGUMENT(t) (void)(t)
#define UNUSED_FIELD(t) (void)(t)

#define JSON_DEFINE_FLUX_OPERATORS(Type) \
  friend std::ostream& operator<<( std::ostream& o, Type const& p) { \
    auto W = int(o.width()); \
    nlohmann::json j = p; \
    if(W > 0) return o << std::setw(W) << j;\
    else return o << j; \
  }  \
 \
  friend std::istream& operator>>( std::istream& i, Type & p) { \
     nlohmann::json j; i >> j; p = j.get<Type>(); return i;\
  }\
  \
  friend std::string to_string( Type const& p) {\
    std::stringstream s;  s << p; return s.str();\
  }

#endif //PACMAN_CPP_JSON_UTIL_H
