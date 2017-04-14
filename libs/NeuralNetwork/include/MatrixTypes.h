// Copyright 2016 Carrie Rebhuhn
#ifndef MATH_MATRIXTYPES_H_
#define MATH_MATRIXTYPES_H_


//! A file for containing matrix types.

#include <vector>
#include <algorithm>
#include <functional>

typedef std::vector<double> matrix1d;
typedef std::vector<matrix1d> matrix2d;
typedef std::vector<matrix2d> matrix3d;

//! Also contains math functions for use with the matrices
namespace easymath {

template<typename T>
T sum(std::vector<T> m) {
    T s = 0;
    for (T i : m)
        s += i;
    return s;
}

template<typename T>
T sum_if_positive(std::vector<T> m) {
    T s = 0;
    for (T i : m)
        if (i > 0)
            s += i;
    return s;
}

template<typename T>
void square(std::vector<T> *m) {
    for (T &i : *m)
        i = i*i;
}

template<typename T>
T mean(std::vector<T> m) {
    return sum(m) / m.size();
}

template<typename T>
std::vector<T> mean2(std::vector<std::vector<T> > myVector) {
    std::vector<T> myMean(myVector[0].size(), 0.0);
    T s = static_cast<T>(myVector.size());
    for (size_t i = 0; i < myVector.size(); i++) {
        for (size_t j = 0; j < myVector[i].size(); j++) {
            myMean[j] += myVector[i][j] / s;
        }
    }
    return myMean;
}

template<typename T>
std::vector<T> operator+(std::vector<T> a, const std::vector<T>& b) {
    for (int i = 0; i < a.size(); i++) {
        a[i] += b[i];
    }
    return a;
}

template<typename T>
std::vector<T> operator-(const std::vector<T>& a, const std::vector<T>& b) {
    std::vector<T> result;
    result.reserve(a.size());
    std::transform(a.begin(), a.end(), b.begin(),
        std::back_inserter(result), std::minus<T>());
    return result;
}

template<typename T>
std::vector<T> operator/(std::vector<T> a, int b) {
    for (T &el : a) {
        if (b != 0)
            el = el / static_cast<double>(b);  // don't divide by 0!
    }
    return a;
}

template<typename T>
std::vector<T> set_negative_zero(const std::vector<T> &m) {
    std::vector<T> r = m;
    for (T &i : r) {
        if (i < 0) i = 0;
    }
    return r;
}

template<typename T>
size_t get_max_index(std::vector<T> v) {
    #ifndef _WIN32
    typename std::vector<T>::iterator el = std::max_element(v.begin(), v.end());
    #else
    std::vector<T>::iterator el = std::max_element(v.begin(), v.end());
    #endif
    return distance(v.begin(), el);
}

template <typename T>
std::vector<std::vector<bool> > operator<
(const std::vector<std::vector<T> >& lhs, const double rhs) {
    size_t m = lhs.size();
    size_t n = lhs[0].size();
    std::vector<std::vector<bool> > M(m, std::vector<bool>(n));
    for (size_t i = 0; i < m; i++) {
        for (size_t j = 0; j < n; j++) {
            M[i][j] = lhs[i][j] < rhs;
        }
    }
    return M;
}

double normalize(double val, double min, double max);

void sigmoid(matrix1d *myVector);
matrix1d sigmoid(matrix1d vec);
matrix1d flatten(const matrix2d &A);

void zero(matrix2d * m);
void zero(matrix1d * m);
matrix1d zeros(size_t dim1);
matrix2d zeros(size_t dim1, size_t dim2);
matrix3d zeros(size_t dim1, size_t dim2, size_t dim3);

matrix2d operator*(const matrix2d &A, const matrix2d &B);
matrix1d operator*(const matrix2d &A, const matrix1d &B);
matrix1d operator*(const matrix1d &A, const matrix2d &B);
matrix2d operator*(const matrix1d &A, const matrix1d &B);

matrix1d operator*(const double A, matrix1d B);
matrix2d operator*(const double A, matrix2d B);
matrix1d operator-(matrix1d A, const matrix1d &B);
matrix2d operator-(matrix2d A, const matrix2d &B);
matrix1d operator+(matrix1d A, const matrix1d &B);
matrix1d dot(matrix1d m1, const matrix1d &m2);
matrix1d sigmoidDerivative(matrix1d X);
matrix2d T(matrix2d m);
matrix1d pow(matrix1d A, double p);
void cmpIntFatal(size_t a, size_t b);
}  // namespace easymath
#endif  // MATH_MATRIXTYPES_H_
