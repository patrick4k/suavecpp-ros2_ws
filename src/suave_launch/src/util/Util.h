#ifndef UTIL_H
#define UTIL_H
#include <ostream>

// template<typename T>
// void log(T msg);
//
// template<typename T>
// void error(T msg);
//
// template<typename T>
// void warn(T msg);

#define error std::cerr << "ERROR: "
#define log std::cout << "INFO: "
#define warn std::cout << "WARNING: "

#endif //UTIL_H
