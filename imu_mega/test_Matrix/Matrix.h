#ifndef MATRIX_H
#define MATRIX_H

#include <Arduino.h>
#include <math.h>

// Fixed-size R x C float matrix
template<uint8_t R, uint8_t C>
struct Mat {
  float m[R][C];

  void zero() {
    for (uint8_t i = 0; i < R; i++)
      for (uint8_t j = 0; j < C; j++) m[i][j] = 0.0f;
  }
  void identity() {                       // meaningful for square matrices
    zero();
    for (uint8_t i = 0; i < R && i < C; i++) m[i][i] = 1.0f;
  }
  float&       operator()(uint8_t i, uint8_t j)       { return m[i][j]; }
  const float& operator()(uint8_t i, uint8_t j) const { return m[i][j]; }
};

// out = A (R x K) * B (K x C).   out must NOT be A or B.
template<uint8_t R, uint8_t K, uint8_t C>
void matMul(const Mat<R, K>& A, const Mat<K, C>& B, Mat<R, C>& out) {
  for (uint8_t i = 0; i < R; i++)
    for (uint8_t j = 0; j < C; j++) {
      float s = 0;
      for (uint8_t k = 0; k < K; k++) s += A.m[i][k] * B.m[k][j];
      out.m[i][j] = s;
    }
}

// out = A (R x K) * B' where B is (C x K).   out must NOT alias A or B.
template<uint8_t R, uint8_t K, uint8_t C>
void matMulBt(const Mat<R, K>& A, const Mat<C, K>& B, Mat<R, C>& out) {
  for (uint8_t i = 0; i < R; i++)
    for (uint8_t j = 0; j < C; j++) {
      float s = 0;
      for (uint8_t k = 0; k < K; k++) s += A.m[i][k] * B.m[j][k];
      out.m[i][j] = s;
    }
}

// out = A'  (A is R x C, out is C x R)
template<uint8_t R, uint8_t C>
void matTranspose(const Mat<R, C>& A, Mat<C, R>& out) {
  for (uint8_t i = 0; i < R; i++)
    for (uint8_t j = 0; j < C; j++) out.m[j][i] = A.m[i][j];
}

// A += B
template<uint8_t R, uint8_t C>
void matAdd(Mat<R, C>& A, const Mat<R, C>& B) {
  for (uint8_t i = 0; i < R; i++)
    for (uint8_t j = 0; j < C; j++) A.m[i][j] += B.m[i][j];
}

// A = (A + A') / 2   (keeps covariance symmetric)
template<uint8_t N>
void matSymmetrize(Mat<N, N>& A) {
  for (uint8_t i = 0; i < N; i++)
    for (uint8_t j = i + 1; j < N; j++) {
      float v = 0.5f * (A.m[i][j] + A.m[j][i]);
      A.m[i][j] = v; A.m[j][i] = v;
    }
}

// inv = A^-1 for symmetric positive-definite A (Cholesky: A = L L').
// Returns false if A is not positive definite (the update should then be skipped).
template<uint8_t N>
bool matInvSPD(const Mat<N, N>& A, Mat<N, N>& inv) {
  Mat<N, N> L;
  L.zero();
  for (uint8_t i = 0; i < N; i++) {
    for (uint8_t j = 0; j <= i; j++) {
      float s = A.m[i][j];
      for (uint8_t k = 0; k < j; k++) s -= L.m[i][k] * L.m[j][k];
      if (i == j) {
        if (s <= 1e-12f) return false;
        L.m[i][i] = sqrtf(s);
      } else {
        L.m[i][j] = s / L.m[j][j];
      }
    }
  }
  // solve A x = e_c for each column c:  L y = e_c,  L' x = y
  for (uint8_t c = 0; c < N; c++) {
    float y[N];
    for (uint8_t i = 0; i < N; i++) {
      float s = (i == c) ? 1.0f : 0.0f;
      for (uint8_t k = 0; k < i; k++) s -= L.m[i][k] * y[k];
      y[i] = s / L.m[i][i];
    }
    for (int8_t i = N - 1; i >= 0; i--) {
      float s = y[i];
      for (uint8_t k = i + 1; k < N; k++) s -= L.m[k][i] * inv.m[k][c];
      inv.m[i][c] = s / L.m[i][i];
    }
  }
  return true;
}

#endif