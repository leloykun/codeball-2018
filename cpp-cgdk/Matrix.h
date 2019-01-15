#if defined(_MSC_VER) && (_MSC_VER >= 1200)
#pragma once
#endif

#ifndef _MATRIX_H_
#define _MATRIX_H_

#include <vector>
#include <cassert>
#include <iostream>

struct Matrix {
  int rows, cols, cnt;
  std::vector<double> data;
  inline double& at(int i, int j) { return this->data[i*cols + j]; }
  Matrix() { }
  Matrix(int r, int c) : rows(r), cols(c), cnt(r*c) {
    this->data.assign(this->cnt, 0.0);  }
  Matrix(const std::vector<double> &other) : rows(1), cols(other.size()),
    cnt(other.size()), data(other) { }
  Matrix(const Matrix &other) : rows(other.rows), cols(other.cols),
    cnt(other.cnt), data(other.data) { }
  double& operator()(int i, int j) { return this->at(i, j); }

  Matrix& operator +=(const Matrix &other) {
    for (int i = 0; i < this->cnt; ++i)
      this->data[i] += other.data[i];
    return *this;  }
  Matrix& operator -=(const Matrix &other) {
    for (int i = 0; i < this->cnt; ++i)
      this->data[i] -= other.data[i];
    return *this;  }
  Matrix& operator *=(const double &other) {
    for (int i = 0; i < this->cnt; ++i)
      this->data[i] *= other;
    return *this;  }
  Matrix& operator *=(const Matrix &other) {
    /*
    Matrix res(this->rows, other.cols);
    for (int i = 0; i < this->rows; ++i)
      for (int k = 0; k < this->cols; ++k)
        for (int j = 0; j < other.cols; ++j)
          res(i, j) += at(i, k) * other.data[k*other.cols + j];
    for (int i = 0; i < this->cnt; ++i)
      this->data[i] = res.data[i];
    */
    // Hadamard product
    for (int i = 0; i < this->cnt; ++i)
      this->data[i] *= other.data[i];
    return *this;  }
  Matrix dot(const Matrix &other) const {
    assert(this->cols == other.rows);
    double w = 0;
    Matrix res(this->rows, other.cols);
    for (int i = 0; i < this->rows; ++i)
      for (int j = 0; j < other.cols; ++j) {
        for (int h = 0; h < this->cols; ++h)
          w += this->data[i*this->cols + h] * other.data[h*other.cols + j];
        res(i, j) = w;
        w = 0;
      }
    return res;  }

  Matrix transpose() {
    Matrix res(this->cols, this->rows);
    for (int i = 0; i < this->rows; ++i)
      for (int j = 0; j < this->cols; ++j)
        res(j, i) = at(i, j);
    return res;  }
  Matrix apply_function(double (*function)(double)) const {
    Matrix res(this->rows, this->cols);
    for (int i = 0; i < this->cnt; ++i)
      res.data[i] = (*function)(this->data[i]);
    return res;  }
  void print() {
    for (int i = 0; i < this->rows; ++i) {
      for (int j = 0; j < this->cols; ++j)
        std::cout<<this->at(i, j)<<" ";
      std::cout<<"\n";
    }
  }
};
inline Matrix operator +(Matrix lhs, Matrix &rhs) {
  lhs += rhs;  return lhs;  }
inline Matrix operator -(Matrix lhs, Matrix &rhs) {
  lhs -= rhs;  return lhs;  }
inline Matrix operator *(Matrix lhs, const double &rhs) {
  lhs *= rhs;  return lhs;  }
inline Matrix operator *(const double &lhs, Matrix rhs) {
  rhs *= lhs;  return rhs;  }
inline Matrix operator *(Matrix lhs, Matrix &rhs) {
  lhs *= rhs;  return lhs;  }

#endif
