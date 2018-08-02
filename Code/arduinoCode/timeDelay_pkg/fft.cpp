//
// Trial License - for use to evaluate programs for possible purchase as
// an end-user only.
// File: fft.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 01-Aug-2018 12:42:33
//

// Include Files
#include "rt_nonfinite.h"
#include "timeDelay.h"
#include "fft.h"
#include "fft1.h"

// Function Definitions

//
// Arguments    : const double x[50]
//                double varargin_1
//                creal_T y_data[]
//                int y_size[1]
// Return Type  : void
//
void fft(const double x[50], double varargin_1, creal_T y_data[], int y_size[1])
{
  boolean_T useRadix2;
  int N2blue;
  int nRows;
  double costab_data[257];
  int costab_size[2];
  double sintab_data[257];
  int sintab_size[2];
  double sintabinv_data[257];
  int sintabinv_size[2];
  if ((int)varargin_1 == 0) {
    y_size[0] = 0;
  } else {
    useRadix2 = (((int)varargin_1 & ((int)varargin_1 - 1)) == 0);
    get_algo_sizes((int)varargin_1, useRadix2, &N2blue, &nRows);
    generate_twiddle_tables(nRows, useRadix2, costab_data, costab_size,
      sintab_data, sintab_size, sintabinv_data, sintabinv_size);
    if (useRadix2) {
      r2br_r2dit_trig(x, (int)varargin_1, costab_data, sintab_data, y_data,
                      y_size);
    } else {
      dobluesteinfft(x, N2blue, (int)varargin_1, costab_data, sintab_data,
                     sintabinv_data, y_data, y_size);
    }
  }
}

//
// File trailer for fft.cpp
//
// [EOF]
//
