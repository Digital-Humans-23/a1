#pragma once

#pragma warning(disable : 4996)

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include <Eigen/Eigen>
#include <string>
#include <vector>

#include "utils.h"

#ifdef WIN32
#include <windows.h>
#else
#include <sys/stat.h>
#endif

#define ANSI_COLOR_RED "\x1b[31m"
#define ANSI_COLOR_GREEN "\x1b[32m"
#define ANSI_COLOR_YELLOW "\x1b[33m"
#define ANSI_COLOR_BLUE "\x1b[34m"
#define ANSI_COLOR_MAGENTA "\x1b[35m"
#define ANSI_COLOR_CYAN "\x1b[36m"
#define ANSI_COLOR_RESET "\x1b[0m"

namespace crl {

// interpret the array of characters as a sequence of strings...
inline void getCharSeparatedStringList(const char *cString,
                                       std::vector<std::string> &lines,
                                       char separator = '\n');

class Logger {
public:
    struct ConsoleText {
        std::string text;
        Eigen::Vector3d color;
    };

    enum PRINT_COLOR { RED, GREEN, YELLOW, BLUE, MAGENTA, CYAN, RESET };

    static std::vector<ConsoleText> consoleOutput;
    static int maxConsoleLineCount;

    static std::string ms_strLogPath;
    static std::string ms_strPrintFileName;
    static std::string ms_strLogFileName;
    static std::string ms_strConsoleFileName;

    static void print(const char *fmt, ...);
    static void print(PRINT_COLOR color, const char *fmt, ...);
    static void logPrint(const char *fmt, ...);
    static FILE* getConsoleFilePointer();
    static void consolePrint(const char *fmt, ...);
    static void consolePrint(const Eigen::Vector3d &color, const char *fmt, ...);

    static bool createPath(const std::string &_strPath);

protected:
    Logger();
    virtual ~Logger();
};

}  // namespace crl