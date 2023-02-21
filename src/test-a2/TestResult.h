#pragma once

#include <string>
#include <iostream>
#include <ostream>
#include <cstring>

#include <Eigen/Core>

#ifdef WIN32
#define NOMINMAX
#include <Windows.h>
#endif

namespace tests {

struct TestResult {
    bool passed = true;
    std::string error = "";

    TestResult operator+(const TestResult &other) const {
        return {passed && other.passed, error + other.error};
    }

    void operator+=(const TestResult &other) {
        *this = *this + other;
    }

};



template<typename T>
double get_error(const T &a, const T &b) {
    return abs(a-b);
}
//template<typename _Scalar, int _Rows, int _Cols, int _Options, int _MaxRows, int _MaxCols>
//double get_error< Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> >(const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &a, const Eigen::Matrix<_Scalar, _Rows, _Cols, _Options, _MaxRows, _MaxCols> &b) {
//    return (a-b).norm();
//}

//template<>
//double get_error< Eigen::Vector3d >(const Eigen::Vector3d &a, const Eigen::Vector3d &b) {
//    return (a-b).norm();
//}
//template<>
//double get_error< Eigen::Matrix3d >(const Eigen::Matrix3d &a, const Eigen::Matrix3d &b) {
//    return (a-b).norm();
//}

template<class T>
struct Get {
    static double absolute_error(const T &a, const T &b){
        return abs(a-b);
    }
    static double relative_error(const T &a, const T &b){
        return abs(a-b) / abs(b);
    }
    static double norm(const T &a){
        return abs(a);
    }
    static void add_to_stream(std::stringstream &ss, const T &x) {
        ss << x;
    }
};
template<typename S, int N, int M>
struct Get<Eigen::Matrix<S,N,M>> {
    typedef  Eigen::Matrix<S,N,M> T;
    static double absolute_error(const T &a, const T &b){
        return (a-b).norm();
    }
    static double relative_error(const T &a, const T &b){
        return (a-b).norm() / b.norm();
    }
    static double norm(const T &a){
        return a.norm();
    }
    static void add_to_stream(std::stringstream &ss, const T &x) {
        ss << "\n" << x;
    }
};

typedef int64_t SameOutputFlags;
enum SameOutputFlags_ {
    SameOutputFlags_None            = 0,
    SameOutputFlags_Error           = 1 << 1,
    SameOutputFlags_ErrorWhenPassed = 1 << 2,
    SameOutputFlags_A_B             = 1 << 3,
    SameOutputFlags_Diff            = 1 << 4,
    SameOutputFlags_All             = ~0
};


typedef int64_t SameOptionFlags;
enum SameOptionFlags_ {
    SameOptionFlags_None            = 0,
    SameOptionFlags_RelativeError   = 1 << 1,
    SameOptionFlags_All             = ~0
};

template<class T>
TestResult same(const T &a, const T&b, double tol, const char *name_a, const char *name_b,  SameOutputFlags flagsOutput = SameOutputFlags_All, SameOptionFlags flagsOption = SameOptionFlags_None) {
    double error = (flagsOption & SameOptionFlags_RelativeError)
                       ? Get<T>::relative_error(a, b)
                       : Get<T>::absolute_error(a, b);
    std::stringstream ss;

    bool passed = error < tol;
    if((!passed && flagsOutput & SameOutputFlags_Error))
        ss << " (error = " << error << " > tol = " << tol << ")\n";
    if((passed && flagsOutput & SameOutputFlags_ErrorWhenPassed))
        ss << " (error = " << error << " < tol = " << tol << ")\n";
    if(error > tol) {
        ss << name_a << " != " << name_b << std::endl;
        if(flagsOutput & SameOutputFlags_A_B){
            ss << name_a << ": "; Get<T>::add_to_stream(ss, a); ss << std::endl;
            ss << name_b << ": "; Get<T>::add_to_stream(ss, b); ss << std::endl;
        }
        if(flagsOutput & SameOutputFlags_Diff){
            ss << "difference: ";
            if(flagsOption & SameOptionFlags_RelativeError)
                Get<T>::add_to_stream(ss, (a-b)/Get<T>::norm(a));
            else
                Get<T>::add_to_stream(ss, (a-b));
            ss << std::endl;
        }
        ss << "\n";
        return {false, ss.str()};
    }

    return {true, ss.str()};
}
#define SAME(a, b, tol) same(a, b, tol, #a, #b)
#define SAME_RELATIVE(a, b, tol) same(a, b, tol, #a, #b, SameOutputFlags_All, SameOptionFlags_RelativeError)
#define SAME_FLAGS(a, b, tol, flags) same(a, b, tol, #a, #b, flags)

enum Color { GREEN = 10, WHITE = 15, RED = 12, GREY = 7 };

void setOStreamColor(std::ostream &out, Color color) {
#ifdef WIN32
    HANDLE hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
    SetConsoleTextAttribute(hConsole, color);
#else
    switch (color)
    {
    case GREEN:
        out << "\033[1;32m";  break;
    case WHITE:
        out << "\033[0m"; break;
    case RED:
        out << "\033[1;31m"; break;
    case GREY:
        out << "\033[37m"; break;
    }
#endif // WIN32
}

std::ostream &operator<<(std::ostream &o, const Color &oColor) {
    setOStreamColor(o, oColor);
    return o;
}

static bool allTestsOk = true;

std::ostream& operator<<(std::ostream& out, const TestResult &tr) {
    if(tr.passed) out << GREEN << "PASSED";
    else          out << RED   << "FAILED";
    out << WHITE << "";
    out << ((tr.error.empty()) ? "\n" : tr.error);
    return out;
}

typedef TestResult (*func_t)();
inline void test_function(func_t f, const char *name) {
    TestResult tr = f();
    std::ostream &out = (!tr.passed) ? std::cerr : std::cout;

    int tabSize = 60;
    std::string whiteSpaces(std::max(0, tabSize - (int)strlen(name)), '.');
    out << name << GREY << whiteSpaces << "";
//    if(tr.passed) out << GREEN << "PASSED";
//    else          out << RED   << "FAILED";
//    out << WHITE << "" << std::endl;
//    if(!tr.passed) out << tr.error;
    out << tr;
    allTestsOk &= tr.passed;
}
#define TEST(a) test_function(a, #a)

template<int N, int M, std::size_t P>
std::ostream& operator<<(std::ostream& stream, const std::array<Eigen::Matrix<double, N, M>, P> &m) {

    int i = 0;
    for (const auto &e : m) {
        stream << i++ << ":\n" << e << std::endl;
    }
    return stream;
}

} // namespace tests
