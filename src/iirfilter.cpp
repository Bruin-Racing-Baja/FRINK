#include <iirfilter.h>
#include <malloc.h>
#include <stddef.h>
#include <types.h>

IIRFilter::IIRFilter(const float b_[], const float a_[], u32 M_,
                     u32 N_) {
  M = M_;
  N = N_;

  b = (float *)calloc(M, sizeof(float));
  a = (float *)calloc(N, sizeof(float));
  float scale = a_[0];
  for (size_t i = 0; i < M; i++) {
    b[i] = b_[i] / scale;
  }
  for (size_t i = 1; i < N; i++) {
    a[i] = a_[i] / scale;
  }

  y = (float *)calloc(M, sizeof(float));
  x = (float *)calloc(N, sizeof(float));
}

IIRFilter::~IIRFilter() {
  free(b);
  free(a);
}

float IIRFilter::update(float new_x) {
  for (size_t i = N - 1; i > 0; i--) {
    x[i] = x[i - 1];
  }
  x[0] = new_x;

  for (size_t i = M - 1; i > 0; i--) {
    y[i] = y[i - 1];
  }
  y[0] = 0;

  for (size_t i = 0; i < M; i++) {
    y[0] += b[i] * x[i];
  }
  for (size_t i = 1; i < N; i++) {
    y[0] -= a[i] * y[i];
  }
  return y[0];
}

float IIRFilter::get() { return y[0]; }
