// Copyright 2016 Carrie Rebhuhn
#include "MatrixTypes.h"

matrix1d easymath::pow(matrix1d A, double p) {
    for (double &a : A) {
        a = std::pow(a, p);
    }
    return A;
}

matrix1d easymath::sigmoidDerivative(matrix1d X) {
    matrix1d one = (matrix1d(X.size(), 1.0));
    return dot(sigmoid(X), one - sigmoid(X));
}

matrix2d easymath::T(matrix2d m) {
    matrix2d t(m[0].size(), matrix1d(m.size()));
    for (size_t i = 0; i < m.size(); i++) {
        for (size_t j = 0; j < m[0].size(); j++) {
            t[j][i] = m[i][j];
        }
    }
    return t;
}

double easymath::normalize(double val, double min, double max) {
    return (val - min) / (max - min);
}

void easymath::zero(matrix2d * m) {
    if (m->empty())
        return;
    else
        *m = matrix2d(m->size(), matrix1d(m[0].size(), 0.0));
}

void easymath::zero(matrix1d * m) {
    *m = matrix1d(m->size(), 0.0);
}

matrix1d easymath::zeros(size_t dim1) {
    return matrix1d(dim1, 0.0);
}

matrix2d easymath::zeros(size_t dim1, size_t dim2) {
    return matrix2d(dim1, easymath::zeros(dim2));
}

matrix3d easymath::zeros(size_t dim1, size_t dim2, size_t dim3) {
    return matrix3d(dim1, easymath::zeros(dim2, dim3));
}

matrix1d easymath::sigmoid(matrix1d vec) {
    for (auto &v : vec) {
        v = 1 / (1 + exp(-v));
    }
    return vec;
}
void easymath::sigmoid(matrix1d *myVector) {
    for (size_t i = 0; i < myVector->size(); i++) {
        myVector->at(i) = 1 / (1 + exp(-myVector->at(i)));
    }
}

void easymath::cmpIntFatal(size_t a, size_t b) {
    if (a != b) {
        printf("Sizes do not match! Pausing to debug then exiting.");
        system("pause");
        exit(1);
    }
}

matrix1d easymath::flatten(const matrix2d &A) {
    matrix1d B;
    for (auto a : A)
        B.insert(B.end(), a.begin(), a.end());
    return B;
}

matrix1d easymath::operator+(matrix1d A, const matrix1d &B) {
    for (size_t i = 0; i < A.size(); i++) {
        A[i] += B[i];
    }
    return A;
}

matrix1d easymath::operator-(matrix1d A, const matrix1d &B) {
    for (size_t i = 0; i < A.size(); i++) {
        A[i] -= B[i];
    }
    return A;
}

matrix2d easymath::operator-(matrix2d A, const matrix2d &B) {
    for (size_t i = 0; i < A.size(); i++) {
        A[i] = A[i] - B[i];
    }
    return A;
}

matrix1d easymath::dot(matrix1d m1, const matrix1d &m2) {
    for (size_t i = 0; i < m1.size(); i++) {
        m1[i] *= m2[i];
    }
    return m1;
}

matrix2d easymath::operator*(const matrix2d &A, const matrix2d &B) {
    // returns a size(A,1)xsize(B,2) matrix
    // printf("mm");
    cmpIntFatal(A[0].size(), B.size());

    matrix2d C(A.size());
    for (size_t row = 0; row < A.size(); row++) {
        C[row] = matrix1d(B[0].size(), 0.0);
        for (size_t col = 0; col < B[0].size(); col++) {
            for (size_t inner = 0; inner < B.size(); inner++) {
                C[row][col] += A[row][inner] * B[inner][col];
            }
        }
    }

    return C;
}

matrix2d easymath::operator*(const matrix1d &A, const matrix1d &B) {
    // returns a A.size()xB.size() matrix

    matrix2d C(A.size());
    for (size_t row = 0; row < A.size(); row++) {
        C[row] = matrix1d(B.size(), 0.0);
        for (size_t col = 0; col < B.size(); col++) {
            C[row][col] += A[row] * B[col];
        }
    }

    return C;
}

matrix1d easymath::operator*(const matrix2d &A, const matrix1d &B) {
    // returns a size(A,1)x1 matrix
    // assumes B is a COLUMN vector
    // printf("mm1");
    cmpIntFatal(A[0].size(), B.size());

    matrix1d C(A.size(), 0.0);
    for (size_t row = 0; row < A.size(); row++) {
        for (size_t inner = 0; inner < B.size(); inner++) {
            C[row] += A[row][inner] * B[inner];
        }
    }

    return C;
}

matrix1d easymath::operator*(const matrix1d &A, const matrix2d &B) {
    // Use this if expecting to get a vector back;
    // assumes A is a ROW vector (1xcols)
    // returns a 1xsize(B,2) matrix

    cmpIntFatal(A.size(), B.size());

    // MODIFY TO MATCH1
    matrix1d C(B[0].size(), 0.0);
    for (size_t col = 0; col < B[0].size(); col++) {
        for (size_t inner = 0; inner < B.size(); inner++) {
            C[col] += A[inner] * B[inner][col];
        }
    }

    return C;
}
