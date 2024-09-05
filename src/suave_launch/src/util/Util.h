#ifndef UTIL_H
#define UTIL_H
#include <iostream>

#define suave_err std::cerr << "ERROR: "
#define suave_log std::cout << "INFO: "
#define suave_warn std::cout << "WARNING: "

#define await_confirmation std::cout << "Press enter to continue..." << std::endl; std::cin.ignore(std::numeric_limits<std::streamsize>::max(),'\n')

#endif //UTIL_H
