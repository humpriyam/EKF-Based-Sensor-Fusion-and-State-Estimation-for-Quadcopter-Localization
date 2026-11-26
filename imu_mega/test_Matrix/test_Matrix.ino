#include "Matrix.h"

int freeRam() {
  extern int __heap_start, *__brkval;
  int v;
  return (int)&v - (__brkval == 0 ? (int)&__heap_start : (int)__brkval);
}

template<uint8_t N>
float invError(const Mat<N, N>& A) {
  Mat<N, N> inv, prod;
  if (!matInvSPD(A, inv)) return -1;
  matMul(A, inv, prod);
  float e = 0;
  for (uint8_t i = 0; i < N; i++)
    for (uint8_t j = 0; j < N; j++) {
      float t = (i == j) ? 1.0f : 0.0f;
      e = max(e, fabsf(prod(i, j) - t));
    }
  return e;
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println();
  Serial.println(F("--- Step 9 test ---"));

  // 1) 3x3 SPD inverse
  Mat<3, 3> A3;
  float a3[9] = {4, 1, 0,  1, 3, 1,  0, 1, 2};
  for (uint8_t i = 0; i < 3; i++) for (uint8_t j = 0; j < 3; j++) A3(i, j) = a3[i * 3 + j];
  Serial.print(F("3x3 max |A*inv(A) - I| = ")); Serial.println(invError(A3), 7);

  // 2) 6x6 SPD inverse: M*M' + 6I
  Mat<6, 6> M, A6;
  for (uint8_t i = 0; i < 6; i++) for (uint8_t j = 0; j < 6; j++) M(i, j) = sinf(i * 7 + j * 3 + 1);
  matMulBt(M, M, A6);
  for (uint8_t i = 0; i < 6; i++) A6(i, i) += 6.0f;
  Serial.print(F("6x6 max |A*inv(A) - I| = ")); Serial.println(invError(A6), 7);

  // 3) non-SPD must be rejected
  Mat<2, 2> bad; bad(0, 0) = 1; bad(0, 1) = 2; bad(1, 0) = 2; bad(1, 1) = 1;
  Mat<2, 2> badInv;
  Serial.print(F("non-SPD rejected: "));
  Serial.println(matInvSPD(bad, badInv) ? F("NO (bug)") : F("yes"));

  // 4) multiply check: [1 2;3 4]*[5 6;7 8] = [19 22;43 50]
  Mat<2, 2> X, Y, Z;
  X(0,0)=1; X(0,1)=2; X(1,0)=3; X(1,1)=4;
  Y(0,0)=5; Y(0,1)=6; Y(1,0)=7; Y(1,1)=8;
  matMul(X, Y, Z);
  Serial.print(F("matMul: "));
  Serial.print(Z(0,0)); Serial.print(' '); Serial.print(Z(0,1)); Serial.print(' ');
  Serial.print(Z(1,0)); Serial.print(' '); Serial.println(Z(1,1));

  // 5) timing: 7x7 multiply and 6x6 inverse
  Mat<7, 7> P, F7, R7;
  for (uint8_t i = 0; i < 7; i++)
    for (uint8_t j = 0; j < 7; j++) { P(i, j) = 0.01f * (i + j); F7(i, j) = (i == j); }
  unsigned long t0 = micros();
  for (int k = 0; k < 10; k++) matMul(F7, P, R7);
  Serial.print(F("7x7 multiply: ")); Serial.print((micros() - t0) / 10); Serial.println(F(" us"));
  Mat<6, 6> inv6;
  t0 = micros();
  for (int k = 0; k < 10; k++) matInvSPD(A6, inv6);
  Serial.print(F("6x6 inverse:  ")); Serial.print((micros() - t0) / 10); Serial.println(F(" us"));

  Serial.print(F("Free RAM: ")); Serial.print(freeRam()); Serial.println(F(" bytes"));
}

void loop() {}