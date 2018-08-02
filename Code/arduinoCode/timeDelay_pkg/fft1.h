//
// Trial License - for use to evaluate programs for possible purchase as
// an end-user only.
// File: fft1.h
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 01-Aug-2018 12:42:33
//
#ifndef FFT1_H
#define FFT1_H

// Include Files
#include <stddef.h>
#include <stdlib.h>
#include "rtwtypes.h"
#include "timeDelay_types.h"

// Function Declarations
extern void b_dobluesteinfft(const creal_T x_data[], const int x_size[1], int N2,
  int n1, const double costab_data[], const double sintab_data[], const double
  sintabinv_data[], creal_T y_data[], int y_size[1]);
extern void b_generate_twiddle_tables(int nRows, boolean_T useRadix2, double
  costab_data[], int costab_size[2], double sintab_data[], int sintab_size[2],
  double sintabinv_data[], int sintabinv_size[2]);
extern void c_r2br_r2dit_trig(const creal_T x_data[], const int x_size[1], int
  n1_unsigned, const double costab_data[], const double sintab_data[], creal_T
  y_data[], int y_size[1]);
extern void dobluesteinfft(const double x[50], int N2, int n1, const double
  costab_data[], const double sintab_data[], const double sintabinv_data[],
  creal_T y_data[], int y_size[1]);
extern void generate_twiddle_tables(int nRows, boolean_T useRadix2, double
  costab_data[], int costab_size[2], double sintab_data[], int sintab_size[2],
  double sintabinv_data[], int sintabinv_size[2]);
extern void get_algo_sizes(int n1, boolean_T useRadix2, int *N2blue, int *nRows);
extern void r2br_r2dit_trig(const double x[50], int n1_unsigned, const double
  costab_data[], const double sintab_data[], creal_T y_data[], int y_size[1]);

#endif

//
// File trailer for fft1.h
//
// [EOF]
//
