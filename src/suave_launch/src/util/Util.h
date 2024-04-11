#ifndef UTIL_H
#define UTIL_H
#include <iostream>

// template<typename T>
// void log(T msg);
//
// template<typename T>
// void error(T msg);
//
// template<typename T>
// void warn(T msg);

#define suave_err std::cerr << "ERROR: "
#define suave_log std::cout << "INFO: "
#define suave_warn std::cout << "WARNING: "

#endif //UTIL_H
