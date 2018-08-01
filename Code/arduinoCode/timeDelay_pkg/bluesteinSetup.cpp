//
// Trial License - for use to evaluate programs for possible purchase as
// an end-user only.
// File: bluesteinSetup.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 01-Aug-2018 12:42:33
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "timeDelay.h"
#include "bluesteinSetup.h"

// Function Definitions

//
// Arguments    : int nRows
//                creal_T wwc_data[]
//                int wwc_size[1]
// Return Type  : void
//
void b_bluesteinSetup(int nRows, creal_T wwc_data[], int wwc_size[1])
{
  int nInt2m1;
  int idx;
  int rt;
  int nInt2;
  int k;
  int y;
  double nt_im;
  double nt_re;
  nInt2m1 = (nRows + nRows) - 1;
  wwc_size[0] = nInt2m1;
  idx = nRows;
  rt = 0;
  wwc_data[nRows - 1].re = 1.0;
  wwc_data[nRows - 1].im = 0.0;
  nInt2 = nRows << 1;
  for (k = 1; k < nRows; k++) {
    y = (k << 1) - 1;
    if (nInt2 - rt <= y) {
      rt += y - nInt2;
    } else {
      rt += y;
    }

    nt_im = 3.1415926535897931 * (double)rt / (double)nRows;
    if (nt_im == 0.0) {
      nt_re = 1.0;
      nt_im = 0.0;
    } else {
      nt_re = std::cos(nt_im);
      nt_im = std::sin(nt_im);
    }

    wwc_data[idx - 2].re = nt_re;
    wwc_data[idx - 2].im = -nt_im;
    idx--;
  }

  idx = 0;
  for (k = nInt2m1 - 1; k >= nRows; k--) {
    wwc_data[k] = wwc_data[idx];
    idx++;
  }
}

//
// Arguments    : int nRows
//                creal_T wwc_data[]
//                int wwc_size[1]
// Return Type  : void
//
void bluesteinSetup(int nRows, creal_T wwc_data[], int wwc_size[1])
{
  int nInt2m1;
  int idx;
  int rt;
  int nInt2;
  int k;
  int y;
  double nt_im;
  double nt_re;
  nInt2m1 = (nRows + nRows) - 1;
  wwc_size[0] = nInt2m1;
  idx = nRows;
  rt = 0;
  wwc_data[nRows - 1].re = 1.0;
  wwc_data[nRows - 1].im = 0.0;
  nInt2 = nRows << 1;
  for (k = 1; k < nRows; k++) {
    y = (k << 1) - 1;
    if (nInt2 - rt <= y) {
      rt += y - nInt2;
    } else {
      rt += y;
    }

    nt_im = -3.1415926535897931 * (double)rt / (double)nRows;
    if (nt_im == 0.0) {
      nt_re = 1.0;
      nt_im = 0.0;
    } else {
      nt_re = std::cos(nt_im);
      nt_im = std::sin(nt_im);
    }

    wwc_data[idx - 2].re = nt_re;
    wwc_data[idx - 2].im = -nt_im;
    idx--;
  }

  idx = 0;
  for (k = nInt2m1 - 1; k >= nRows; k--) {
    wwc_data[k] = wwc_data[idx];
    idx++;
  }
}

//
// File trailer for bluesteinSetup.cpp
//
// [EOF]
//
