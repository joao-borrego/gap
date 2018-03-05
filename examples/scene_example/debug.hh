/*!
    \file examples/scene_example/debug.hh
    \brief Debug utilities

    \author João Borrego : jsbruglie
    \author Rui Figueiredo : ruipimentelfigueiredo
*/

#include <iostream>
#include <string.h>

/// Verbose flag. Comment to remove debug features
#define VERBOSE

/// Current filename macro
#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#ifdef VERBOSE
/// Print debug information to std::cout
#define debugPrint(x) do { std::cout << x; } while (0)
#else
#define debugPrint(x) do {} while (0)
#endif

#ifdef VERBOSE
/// Print debug information to std::cout, including current filename, line and function
#define debugPrintTrace(x) do {\
    std::cout << __FILENAME__ << ":" << __LINE__ << ":" << __func__ << ": " << \
    x << "\n";\
} while (0)
#else
#define debugPrintTrace(x) do {} while (0)
#endif