//
// Trial License - for use to evaluate programs for possible purchase as
// an end-user only.
// File: fft1.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 01-Aug-2018 12:42:33
//

// Include Files
#include <string.h>
#include <cmath>
#include "rt_nonfinite.h"
#include "timeDelay.h"
#include "fft1.h"
#include "bluesteinSetup.h"

// Function Declarations
static void b_r2br_r2dit_trig(const creal_T x_data[], const int x_size[1], int
  n1_unsigned, const double costab_data[], const double sintab_data[], creal_T
  y_data[], int y_size[1]);
static void r2br_r2dit_trig_impl(const creal_T x_data[], const int x_size[1],
  int unsigned_nRows, const double costab_data[], const double sintab_data[],
  creal_T y_data[], int y_size[1]);

// Function Definitions

//
// Arguments    : const creal_T x_data[]
//                const int x_size[1]
//                int n1_unsigned
//                const double costab_data[]
//                const double sintab_data[]
//                creal_T y_data[]
//                int y_size[1]
// Return Type  : void
//
static void b_r2br_r2dit_trig(const creal_T x_data[], const int x_size[1], int
  n1_unsigned, const double costab_data[], const double sintab_data[], creal_T
  y_data[], int y_size[1])
{
  int istart;
  int nRowsD2;
  int nRowsD4;
  int ix;
  int iy;
  int ju;
  int i;
  boolean_T tst;
  double temp_re;
  double temp_im;
  int j;
  double twid_re;
  double twid_im;
  int ihi;
  istart = x_size[0];
  if (!(istart < n1_unsigned)) {
    istart = n1_unsigned;
  }

  nRowsD2 = n1_unsigned / 2;
  nRowsD4 = nRowsD2 / 2;
  y_size[0] = n1_unsigned;
  if (n1_unsigned > x_size[0]) {
    y_size[0] = n1_unsigned;
    for (iy = 0; iy < n1_unsigned; iy++) {
      y_data[iy].re = 0.0;
      y_data[iy].im = 0.0;
    }
  }

  ix = 0;
  ju = 0;
  iy = 0;
  for (i = 1; i < istart; i++) {
    y_data[iy] = x_data[ix];
    iy = n1_unsigned;
    tst = true;
    while (tst) {
      iy >>= 1;
      ju ^= iy;
      tst = ((ju & iy) == 0);
    }

    iy = ju;
    ix++;
  }

  y_data[iy] = x_data[ix];
  if (n1_unsigned > 1) {
    for (i = 0; i <= n1_unsigned - 2; i += 2) {
      temp_re = y_data[i + 1].re;
      temp_im = y_data[i + 1].im;
      y_data[i + 1].re = y_data[i].re - y_data[i + 1].re;
      y_data[i + 1].im = y_data[i].im - y_data[i + 1].im;
      y_data[i].re += temp_re;
      y_data[i].im += temp_im;
    }
  }

  iy = 2;
  ix = 4;
  ju = 1 + ((nRowsD4 - 1) << 2);
  while (nRowsD4 > 0) {
    for (i = 0; i < ju; i += ix) {
      temp_re = y_data[i + iy].re;
      temp_im = y_data[i + iy].im;
      y_data[i + iy].re = y_data[i].re - temp_re;
      y_data[i + iy].im = y_data[i].im - temp_im;
      y_data[i].re += temp_re;
      y_data[i].im += temp_im;
    }

    istart = 1;
    for (j = nRowsD4; j < nRowsD2; j += nRowsD4) {
      twid_re = costab_data[j];
      twid_im = sintab_data[j];
      i = istart;
      ihi = istart + ju;
      while (i < ihi) {
        temp_re = twid_re * y_data[i + iy].re - twid_im * y_data[i + iy].im;
        temp_im = twid_re * y_data[i + iy].im + twid_im * y_data[i + iy].re;
        y_data[i + iy].re = y_data[i].re - temp_re;
        y_data[i + iy].im = y_data[i].im - temp_im;
        y_data[i].re += temp_re;
        y_data[i].im += temp_im;
        i += ix;
      }

      istart++;
    }

    nRowsD4 /= 2;
    iy = ix;
    ix += ix;
    ju -= iy;
  }
}

//
// Arguments    : const creal_T x_data[]
//                const int x_size[1]
//                int unsigned_nRows
//                const double costab_data[]
//                const double sintab_data[]
//                creal_T y_data[]
//                int y_size[1]
// Return Type  : void
//
static void r2br_r2dit_trig_impl(const creal_T x_data[], const int x_size[1],
  int unsigned_nRows, const double costab_data[], const double sintab_data[],
  creal_T y_data[], int y_size[1])
{
  int istart;
  int nRowsD2;
  int nRowsD4;
  int ix;
  int iy;
  int ju;
  int i;
  boolean_T tst;
  double temp_re;
  double temp_im;
  int j;
  double twid_re;
  double twid_im;
  int ihi;
  istart = x_size[0];
  if (!(istart < unsigned_nRows)) {
    istart = unsigned_nRows;
  }

  nRowsD2 = unsigned_nRows / 2;
  nRowsD4 = nRowsD2 / 2;
  y_size[0] = unsigned_nRows;
  if (unsigned_nRows > x_size[0]) {
    y_size[0] = unsigned_nRows;
    for (iy = 0; iy < unsigned_nRows; iy++) {
      y_data[iy].re = 0.0;
      y_data[iy].im = 0.0;
    }
  }

  ix = 0;
  ju = 0;
  iy = 0;
  for (i = 1; i < istart; i++) {
    y_data[iy] = x_data[ix];
    iy = unsigned_nRows;
    tst = true;
    while (tst) {
      iy >>= 1;
      ju ^= iy;
      tst = ((ju & iy) == 0);
    }

    iy = ju;
    ix++;
  }

  y_data[iy] = x_data[ix];
  if (unsigned_nRows > 1) {
    for (i = 0; i <= unsigned_nRows - 2; i += 2) {
      temp_re = y_data[i + 1].re;
      temp_im = y_data[i + 1].im;
      y_data[i + 1].re = y_data[i].re - y_data[i + 1].re;
      y_data[i + 1].im = y_data[i].im - y_data[i + 1].im;
      y_data[i].re += temp_re;
      y_data[i].im += temp_im;
    }
  }

  iy = 2;
  ix = 4;
  ju = 1 + ((nRowsD4 - 1) << 2);
  while (nRowsD4 > 0) {
    for (i = 0; i < ju; i += ix) {
      temp_re = y_data[i + iy].re;
      temp_im = y_data[i + iy].im;
      y_data[i + iy].re = y_data[i].re - temp_re;
      y_data[i + iy].im = y_data[i].im - temp_im;
      y_data[i].re += temp_re;
      y_data[i].im += temp_im;
    }

    istart = 1;
    for (j = nRowsD4; j < nRowsD2; j += nRowsD4) {
      twid_re = costab_data[j];
      twid_im = sintab_data[j];
      i = istart;
      ihi = istart + ju;
      while (i < ihi) {
        temp_re = twid_re * y_data[i + iy].re - twid_im * y_data[i + iy].im;
        temp_im = twid_re * y_data[i + iy].im + twid_im * y_data[i + iy].re;
        y_data[i + iy].re = y_data[i].re - temp_re;
        y_data[i + iy].im = y_data[i].im - temp_im;
        y_data[i].re += temp_re;
        y_data[i].im += temp_im;
        i += ix;
      }

      istart++;
    }

    nRowsD4 /= 2;
    iy = ix;
    ix += ix;
    ju -= iy;
  }
}

//
// Arguments    : const creal_T x_data[]
//                const int x_size[1]
//                int N2
//                int n1
//                const double costab_data[]
//                const double sintab_data[]
//                const double sintabinv_data[]
//                creal_T y_data[]
//                int y_size[1]
// Return Type  : void
//
void b_dobluesteinfft(const creal_T x_data[], const int x_size[1], int N2, int
                      n1, const double costab_data[], const double sintab_data[],
                      const double sintabinv_data[], creal_T y_data[], int
                      y_size[1])
{
  creal_T wwc_data[255];
  int wwc_size[1];
  int minNrowsNx;
  int fv_size[1];
  int xidx;
  int loop_ub;
  creal_T fv_data[512];
  double denom_re;
  double denom_im;
  creal_T tmp_data[512];
  int tmp_size[1];
  int b_fv_size[1];
  creal_T b_fv_data[512];
  b_bluesteinSetup(n1, wwc_data, wwc_size);
  minNrowsNx = x_size[0];
  if (n1 < minNrowsNx) {
    minNrowsNx = n1;
  }

  fv_size[0] = n1;
  if (n1 > x_size[0]) {
    fv_size[0] = n1;
    for (xidx = 0; xidx < n1; xidx++) {
      fv_data[xidx].re = 0.0;
      fv_data[xidx].im = 0.0;
    }
  }

  y_size[0] = fv_size[0];
  loop_ub = fv_size[0];
  if (0 <= loop_ub - 1) {
    memcpy(&y_data[0], &fv_data[0], (unsigned int)(loop_ub * (int)sizeof(creal_T)));
  }

  xidx = 0;
  for (loop_ub = 0; loop_ub < minNrowsNx; loop_ub++) {
    denom_re = wwc_data[(n1 + loop_ub) - 1].re;
    denom_im = wwc_data[(n1 + loop_ub) - 1].im;
    y_data[loop_ub].re = denom_re * x_data[xidx].re + denom_im * x_data[xidx].im;
    y_data[loop_ub].im = denom_re * x_data[xidx].im - denom_im * x_data[xidx].re;
    xidx++;
  }

  while (minNrowsNx + 1 <= n1) {
    y_data[minNrowsNx].re = 0.0;
    y_data[minNrowsNx].im = 0.0;
    minNrowsNx++;
  }

  r2br_r2dit_trig_impl(y_data, y_size, N2, costab_data, sintab_data, fv_data,
                       fv_size);
  b_r2br_r2dit_trig(wwc_data, wwc_size, N2, costab_data, sintab_data, tmp_data,
                    tmp_size);
  b_fv_size[0] = fv_size[0];
  loop_ub = fv_size[0];
  for (xidx = 0; xidx < loop_ub; xidx++) {
    b_fv_data[xidx].re = fv_data[xidx].re * tmp_data[xidx].re - fv_data[xidx].im
      * tmp_data[xidx].im;
    b_fv_data[xidx].im = fv_data[xidx].re * tmp_data[xidx].im + fv_data[xidx].im
      * tmp_data[xidx].re;
  }

  c_r2br_r2dit_trig(b_fv_data, b_fv_size, N2, costab_data, sintabinv_data,
                    fv_data, fv_size);
  xidx = 0;
  for (loop_ub = n1 - 1; loop_ub < wwc_size[0]; loop_ub++) {
    y_data[xidx].re = wwc_data[loop_ub].re * fv_data[loop_ub].re +
      wwc_data[loop_ub].im * fv_data[loop_ub].im;
    y_data[xidx].im = wwc_data[loop_ub].re * fv_data[loop_ub].im -
      wwc_data[loop_ub].im * fv_data[loop_ub].re;
    y_data[xidx].re = wwc_data[loop_ub].re * fv_data[loop_ub].re +
      wwc_data[loop_ub].im * fv_data[loop_ub].im;
    y_data[xidx].im = wwc_data[loop_ub].re * fv_data[loop_ub].im -
      wwc_data[loop_ub].im * fv_data[loop_ub].re;
    if (y_data[xidx].im == 0.0) {
      y_data[xidx].re /= (double)n1;
      y_data[xidx].im = 0.0;
    } else if (y_data[xidx].re == 0.0) {
      y_data[xidx].re = 0.0;
      y_data[xidx].im /= (double)n1;
    } else {
      y_data[xidx].re /= (double)n1;
      y_data[xidx].im /= (double)n1;
    }

    xidx++;
  }
}

//
// Arguments    : int nRows
//                boolean_T useRadix2
//                double costab_data[]
//                int costab_size[2]
//                double sintab_data[]
//                int sintab_size[2]
//                double sintabinv_data[]
//                int sintabinv_size[2]
// Return Type  : void
//
void b_generate_twiddle_tables(int nRows, boolean_T useRadix2, double
  costab_data[], int costab_size[2], double sintab_data[], int sintab_size[2],
  double sintabinv_data[], int sintabinv_size[2])
{
  double e;
  int nRowsD4;
  double costab1q_data[129];
  int nd2;
  int k;
  e = 6.2831853071795862 / (double)nRows;
  nRowsD4 = nRows / 2 / 2;
  costab1q_data[0] = 1.0;
  nd2 = nRowsD4 / 2;
  for (k = 1; k <= nd2; k++) {
    costab1q_data[k] = std::cos(e * (double)k);
  }

  for (k = nd2 + 1; k < nRowsD4; k++) {
    costab1q_data[k] = std::sin(e * (double)(nRowsD4 - k));
  }

  costab1q_data[nRowsD4] = 0.0;
  if (!useRadix2) {
    nd2 = nRowsD4 << 1;
    costab_size[0] = 1;
    costab_size[1] = nd2 + 1;
    sintab_size[0] = 1;
    sintab_size[1] = nd2 + 1;
    costab_data[0] = 1.0;
    sintab_data[0] = 0.0;
    sintabinv_size[0] = 1;
    sintabinv_size[1] = nd2 + 1;
    for (k = 1; k <= nRowsD4; k++) {
      sintabinv_data[k] = costab1q_data[nRowsD4 - k];
    }

    for (k = nRowsD4 + 1; k <= nd2; k++) {
      sintabinv_data[k] = costab1q_data[k - nRowsD4];
    }

    for (k = 1; k <= nRowsD4; k++) {
      costab_data[k] = costab1q_data[k];
      sintab_data[k] = -costab1q_data[nRowsD4 - k];
    }

    for (k = nRowsD4 + 1; k <= nd2; k++) {
      costab_data[k] = -costab1q_data[nd2 - k];
      sintab_data[k] = -costab1q_data[k - nRowsD4];
    }
  } else {
    nd2 = nRowsD4 << 1;
    costab_size[0] = 1;
    costab_size[1] = nd2 + 1;
    sintab_size[0] = 1;
    sintab_size[1] = nd2 + 1;
    costab_data[0] = 1.0;
    sintab_data[0] = 0.0;
    for (k = 1; k <= nRowsD4; k++) {
      costab_data[k] = costab1q_data[k];
      sintab_data[k] = costab1q_data[nRowsD4 - k];
    }

    for (k = nRowsD4 + 1; k <= nd2; k++) {
      costab_data[k] = -costab1q_data[nd2 - k];
      sintab_data[k] = costab1q_data[k - nRowsD4];
    }

    sintabinv_size[0] = 1;
    sintabinv_size[1] = 0;
  }
}

//
// Arguments    : const creal_T x_data[]
//                const int x_size[1]
//                int n1_unsigned
//                const double costab_data[]
//                const double sintab_data[]
//                creal_T y_data[]
//                int y_size[1]
// Return Type  : void
//
void c_r2br_r2dit_trig(const creal_T x_data[], const int x_size[1], int
  n1_unsigned, const double costab_data[], const double sintab_data[], creal_T
  y_data[], int y_size[1])
{
  int j;
  int nRowsD2;
  int nRowsD4;
  int ix;
  int iDelta2;
  int ju;
  int iy;
  int i;
  boolean_T tst;
  double temp_re;
  double temp_im;
  double r;
  double twid_im;
  int ihi;
  j = x_size[0];
  if (!(j < n1_unsigned)) {
    j = n1_unsigned;
  }

  nRowsD2 = n1_unsigned / 2;
  nRowsD4 = nRowsD2 / 2;
  y_size[0] = n1_unsigned;
  if (n1_unsigned > x_size[0]) {
    y_size[0] = n1_unsigned;
    for (iDelta2 = 0; iDelta2 < n1_unsigned; iDelta2++) {
      y_data[iDelta2].re = 0.0;
      y_data[iDelta2].im = 0.0;
    }
  }

  ix = 0;
  ju = 0;
  iy = 0;
  for (i = 1; i < j; i++) {
    y_data[iy] = x_data[ix];
    iDelta2 = n1_unsigned;
    tst = true;
    while (tst) {
      iDelta2 >>= 1;
      ju ^= iDelta2;
      tst = ((ju & iDelta2) == 0);
    }

    iy = ju;
    ix++;
  }

  y_data[iy] = x_data[ix];
  if (n1_unsigned > 1) {
    for (i = 0; i <= n1_unsigned - 2; i += 2) {
      temp_re = y_data[i + 1].re;
      temp_im = y_data[i + 1].im;
      y_data[i + 1].re = y_data[i].re - y_data[i + 1].re;
      y_data[i + 1].im = y_data[i].im - y_data[i + 1].im;
      y_data[i].re += temp_re;
      y_data[i].im += temp_im;
    }
  }

  iy = 2;
  iDelta2 = 4;
  ix = 1 + ((nRowsD4 - 1) << 2);
  while (nRowsD4 > 0) {
    for (i = 0; i < ix; i += iDelta2) {
      temp_re = y_data[i + iy].re;
      temp_im = y_data[i + iy].im;
      y_data[i + iy].re = y_data[i].re - temp_re;
      y_data[i + iy].im = y_data[i].im - temp_im;
      y_data[i].re += temp_re;
      y_data[i].im += temp_im;
    }

    ju = 1;
    for (j = nRowsD4; j < nRowsD2; j += nRowsD4) {
      r = costab_data[j];
      twid_im = sintab_data[j];
      i = ju;
      ihi = ju + ix;
      while (i < ihi) {
        temp_re = r * y_data[i + iy].re - twid_im * y_data[i + iy].im;
        temp_im = r * y_data[i + iy].im + twid_im * y_data[i + iy].re;
        y_data[i + iy].re = y_data[i].re - temp_re;
        y_data[i + iy].im = y_data[i].im - temp_im;
        y_data[i].re += temp_re;
        y_data[i].im += temp_im;
        i += iDelta2;
      }

      ju++;
    }

    nRowsD4 /= 2;
    iy = iDelta2;
    iDelta2 += iDelta2;
    ix -= iy;
  }

  if (y_size[0] > 1) {
    r = 1.0 / (double)y_size[0];
    iy = y_size[0];
    for (iDelta2 = 0; iDelta2 < iy; iDelta2++) {
      y_data[iDelta2].re *= r;
      y_data[iDelta2].im *= r;
    }
  }
}

//
// Arguments    : const double x[50]
//                int N2
//                int n1
//                const double costab_data[]
//                const double sintab_data[]
//                const double sintabinv_data[]
//                creal_T y_data[]
//                int y_size[1]
// Return Type  : void
//
void dobluesteinfft(const double x[50], int N2, int n1, const double
                    costab_data[], const double sintab_data[], const double
                    sintabinv_data[], creal_T y_data[], int y_size[1])
{
  creal_T wwc_data[255];
  int wwc_size[1];
  int minNrowsNx;
  int xidx;
  int k;
  creal_T fv_data[512];
  int fv_size[1];
  creal_T tmp_data[512];
  int tmp_size[1];
  int b_fv_size[1];
  creal_T b_fv_data[512];
  bluesteinSetup(n1, wwc_data, wwc_size);
  if (n1 < 50) {
    minNrowsNx = n1;
  } else {
    minNrowsNx = 50;
  }

  y_size[0] = n1;
  if (n1 > 50) {
    y_size[0] = n1;
    for (k = 0; k < n1; k++) {
      y_data[k].re = 0.0;
      y_data[k].im = 0.0;
    }
  }

  xidx = 0;
  for (k = 0; k < minNrowsNx; k++) {
    y_data[k].re = wwc_data[(n1 + k) - 1].re * x[xidx];
    y_data[k].im = wwc_data[(n1 + k) - 1].im * -x[xidx];
    xidx++;
  }

  while (minNrowsNx + 1 <= n1) {
    y_data[minNrowsNx].re = 0.0;
    y_data[minNrowsNx].im = 0.0;
    minNrowsNx++;
  }

  r2br_r2dit_trig_impl(y_data, y_size, N2, costab_data, sintab_data, fv_data,
                       fv_size);
  b_r2br_r2dit_trig(wwc_data, wwc_size, N2, costab_data, sintab_data, tmp_data,
                    tmp_size);
  b_fv_size[0] = fv_size[0];
  xidx = fv_size[0];
  for (k = 0; k < xidx; k++) {
    b_fv_data[k].re = fv_data[k].re * tmp_data[k].re - fv_data[k].im *
      tmp_data[k].im;
    b_fv_data[k].im = fv_data[k].re * tmp_data[k].im + fv_data[k].im *
      tmp_data[k].re;
  }

  c_r2br_r2dit_trig(b_fv_data, b_fv_size, N2, costab_data, sintabinv_data,
                    fv_data, fv_size);
  xidx = 0;
  for (k = n1 - 1; k < wwc_size[0]; k++) {
    y_data[xidx].re = wwc_data[k].re * fv_data[k].re + wwc_data[k].im *
      fv_data[k].im;
    y_data[xidx].im = wwc_data[k].re * fv_data[k].im - wwc_data[k].im *
      fv_data[k].re;
    xidx++;
  }
}

//
// Arguments    : int nRows
//                boolean_T useRadix2
//                double costab_data[]
//                int costab_size[2]
//                double sintab_data[]
//                int sintab_size[2]
//                double sintabinv_data[]
//                int sintabinv_size[2]
// Return Type  : void
//
void generate_twiddle_tables(int nRows, boolean_T useRadix2, double costab_data[],
  int costab_size[2], double sintab_data[], int sintab_size[2], double
  sintabinv_data[], int sintabinv_size[2])
{
  double e;
  int nRowsD4;
  double costab1q_data[129];
  int nd2;
  int k;
  e = 6.2831853071795862 / (double)nRows;
  nRowsD4 = nRows / 2 / 2;
  costab1q_data[0] = 1.0;
  nd2 = nRowsD4 / 2;
  for (k = 1; k <= nd2; k++) {
    costab1q_data[k] = std::cos(e * (double)k);
  }

  for (k = nd2 + 1; k < nRowsD4; k++) {
    costab1q_data[k] = std::sin(e * (double)(nRowsD4 - k));
  }

  costab1q_data[nRowsD4] = 0.0;
  if (!useRadix2) {
    nd2 = nRowsD4 << 1;
    costab_size[0] = 1;
    costab_size[1] = nd2 + 1;
    sintab_size[0] = 1;
    sintab_size[1] = nd2 + 1;
    costab_data[0] = 1.0;
    sintab_data[0] = 0.0;
    sintabinv_size[0] = 1;
    sintabinv_size[1] = nd2 + 1;
    for (k = 1; k <= nRowsD4; k++) {
      sintabinv_data[k] = costab1q_data[nRowsD4 - k];
    }

    for (k = nRowsD4 + 1; k <= nd2; k++) {
      sintabinv_data[k] = costab1q_data[k - nRowsD4];
    }

    for (k = 1; k <= nRowsD4; k++) {
      costab_data[k] = costab1q_data[k];
      sintab_data[k] = -costab1q_data[nRowsD4 - k];
    }

    for (k = nRowsD4 + 1; k <= nd2; k++) {
      costab_data[k] = -costab1q_data[nd2 - k];
      sintab_data[k] = -costab1q_data[k - nRowsD4];
    }
  } else {
    nd2 = nRowsD4 << 1;
    costab_size[0] = 1;
    costab_size[1] = nd2 + 1;
    sintab_size[0] = 1;
    sintab_size[1] = nd2 + 1;
    costab_data[0] = 1.0;
    sintab_data[0] = 0.0;
    for (k = 1; k <= nRowsD4; k++) {
      costab_data[k] = costab1q_data[k];
      sintab_data[k] = -costab1q_data[nRowsD4 - k];
    }

    for (k = nRowsD4 + 1; k <= nd2; k++) {
      costab_data[k] = -costab1q_data[nd2 - k];
      sintab_data[k] = -costab1q_data[k - nRowsD4];
    }

    sintabinv_size[0] = 1;
    sintabinv_size[1] = 0;
  }
}

//
// Arguments    : int n1
//                boolean_T useRadix2
//                int *N2blue
//                int *nRows
// Return Type  : void
//
void get_algo_sizes(int n1, boolean_T useRadix2, int *N2blue, int *nRows)
{
  int nn1m1;
  int pmax;
  int pmin;
  boolean_T exitg1;
  int p;
  int pow2p;
  *N2blue = 1;
  if (useRadix2) {
    *nRows = n1;
  } else {
    nn1m1 = (n1 + n1) - 1;
    pmax = 31;
    if (nn1m1 <= 1) {
      pmax = 0;
    } else {
      pmin = 0;
      exitg1 = false;
      while ((!exitg1) && (pmax - pmin > 1)) {
        p = (pmin + pmax) >> 1;
        pow2p = 1 << p;
        if (pow2p == nn1m1) {
          pmax = p;
          exitg1 = true;
        } else if (pow2p > nn1m1) {
          pmax = p;
        } else {
          pmin = p;
        }
      }
    }

    *N2blue = 1 << pmax;
    *nRows = *N2blue;
  }
}

//
// Arguments    : const double x[50]
//                int n1_unsigned
//                const double costab_data[]
//                const double sintab_data[]
//                creal_T y_data[]
//                int y_size[1]
// Return Type  : void
//
void r2br_r2dit_trig(const double x[50], int n1_unsigned, const double
                     costab_data[], const double sintab_data[], creal_T y_data[],
                     int y_size[1])
{
  int istart;
  int nRowsD2;
  int nRowsD4;
  int ix;
  int iy;
  int ju;
  int i;
  boolean_T tst;
  double temp_re;
  double temp_im;
  int j;
  double twid_re;
  double twid_im;
  int ihi;
  if (50 < n1_unsigned) {
    istart = 50;
  } else {
    istart = n1_unsigned;
  }

  nRowsD2 = n1_unsigned / 2;
  nRowsD4 = nRowsD2 / 2;
  y_size[0] = n1_unsigned;
  if (n1_unsigned > 50) {
    y_size[0] = n1_unsigned;
    for (iy = 0; iy < n1_unsigned; iy++) {
      y_data[iy].re = 0.0;
      y_data[iy].im = 0.0;
    }
  }

  ix = 0;
  ju = 0;
  iy = 0;
  for (i = 1; i < istart; i++) {
    y_data[iy].re = x[ix];
    y_data[iy].im = 0.0;
    iy = n1_unsigned;
    tst = true;
    while (tst) {
      iy >>= 1;
      ju ^= iy;
      tst = ((ju & iy) == 0);
    }

    iy = ju;
    ix++;
  }

  y_data[iy].re = x[ix];
  y_data[iy].im = 0.0;
  if (n1_unsigned > 1) {
    for (i = 0; i <= n1_unsigned - 2; i += 2) {
      temp_re = y_data[i + 1].re;
      temp_im = y_data[i + 1].im;
      y_data[i + 1].re = y_data[i].re - y_data[i + 1].re;
      y_data[i + 1].im = y_data[i].im - y_data[i + 1].im;
      y_data[i].re += temp_re;
      y_data[i].im += temp_im;
    }
  }

  iy = 2;
  ix = 4;
  ju = 1 + ((nRowsD4 - 1) << 2);
  while (nRowsD4 > 0) {
    for (i = 0; i < ju; i += ix) {
      temp_re = y_data[i + iy].re;
      temp_im = y_data[i + iy].im;
      y_data[i + iy].re = y_data[i].re - temp_re;
      y_data[i + iy].im = y_data[i].im - temp_im;
      y_data[i].re += temp_re;
      y_data[i].im += temp_im;
    }

    istart = 1;
    for (j = nRowsD4; j < nRowsD2; j += nRowsD4) {
      twid_re = costab_data[j];
      twid_im = sintab_data[j];
      i = istart;
      ihi = istart + ju;
      while (i < ihi) {
        temp_re = twid_re * y_data[i + iy].re - twid_im * y_data[i + iy].im;
        temp_im = twid_re * y_data[i + iy].im + twid_im * y_data[i + iy].re;
        y_data[i + iy].re = y_data[i].re - temp_re;
        y_data[i + iy].im = y_data[i].im - temp_im;
        y_data[i].re += temp_re;
        y_data[i].im += temp_im;
        i += ix;
      }

      istart++;
    }

    nRowsD4 /= 2;
    iy = ix;
    ix += ix;
    ju -= iy;
  }
}

//
// File trailer for fft1.cpp
//
// [EOF]
//
