/**
 * @file debug.hh
 * @brief Custom debug utilities
 */

#include <string.h>

#define VERBOSE

#define __FILENAME__ (strrchr(__FILE__, '/') ? strrchr(__FILE__, '/') + 1 : __FILE__)

#ifdef VERBOSE
#define debugPrint(x) do { std::cout << x; } while (0)
#else
#define debugPrint(x) do {} while (0)
#endif

#ifdef VERBOSE
#define debugPrintTrace(x) do {\
    std::cout << __FILENAME__ << ":" << __LINE__ << ":" << __func__ << ": " << \
    x << "\n";\
} while (0)
#else
#define debugPrintTrace(x) do {} while (0)
#endif