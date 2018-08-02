//
// Trial License - for use to evaluate programs for possible purchase as
// an end-user only.
// File: timeDelay.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 01-Aug-2018 12:42:33
//

// Include Files
#include <cmath>
#include <string.h>
#include "rt_nonfinite.h"
#include "timeDelay.h"
#include "ifft.h"
#include "exp.h"
#include "ifftshift.h"
#include "fft.h"
#include "mpower.h"
#include "nextpow2.h"
#include "round.h"

// Function Definitions

//
// Arguments    : const double bandInput[200]
//                const double weightTableTimeDelay[4]
//                double Fs
//                double bandOutput[200]
// Return Type  : void
//
void timeDelay(const double bandInput[200], const double weightTableTimeDelay[4],
               double Fs, double bandOutput[200])
{
  int i0;
  double delayInt[4];
  double orgStart;
  int idx;
  double dtIn[4];
  int k;
  boolean_T exitg1;
  double maxLengthLimit;
  int loop_ub;
  double output_data[400];
  int colI;
  double d0;
  double newStart;
  double y_data[128];
  creal_T tmpxd_data[128];
  int tmpxd_size[1];
  double y_re;
  double y_im;
  int tmp_size[1];
  double tmp_data[128];
  int b_tmp_size[1];
  creal_T b_tmp_data[128];
  creal_T b_tmpxd_data[128];
  for (i0 = 0; i0 < 4; i0++) {
    orgStart = weightTableTimeDelay[i0] * Fs;
    delayInt[i0] = orgStart;
    dtIn[i0] = orgStart;
  }

  b_round(delayInt);
  if (!rtIsNaN(delayInt[0])) {
    idx = 1;
  } else {
    idx = 0;
    k = 2;
    exitg1 = false;
    while ((!exitg1) && (k < 5)) {
      if (!rtIsNaN(delayInt[k - 1])) {
        idx = k;
        exitg1 = true;
      } else {
        k++;
      }
    }
  }

  if (idx == 0) {
    orgStart = delayInt[0];
  } else {
    orgStart = delayInt[idx - 1];
    while (idx + 1 < 5) {
      if (orgStart < delayInt[idx]) {
        orgStart = delayInt[idx];
      }

      idx++;
    }
  }

  if ((0.0 > orgStart) || rtIsNaN(orgStart)) {
    orgStart = 0.0;
  }

  if (50.0 + orgStart > 100.0) {
    maxLengthLimit = 100.0;
  } else {
    maxLengthLimit = 50.0 + orgStart;
  }

  loop_ub = (int)maxLengthLimit << 2;
  if (0 <= loop_ub - 1) {
    memset(&output_data[0], 0, (unsigned int)(loop_ub * (int)sizeof(double)));
  }

  for (colI = 0; colI < 4; colI++) {
    if ((50.0 + delayInt[colI] <= 0.0) || (50.0 + delayInt[colI] > 100.0)) {
    } else if (dtIn[colI] - delayInt[colI] != 0.0) {
      if ((0.0 > delayInt[colI]) || rtIsNaN(delayInt[colI])) {
        d0 = 0.0;
      } else {
        d0 = delayInt[colI];
      }

      orgStart = mpower(nextpow2(50.0 + d0));
      newStart = std::floor(orgStart / 2.0);
      if (rtIsNaN(orgStart - 1.0)) {
        idx = 1;
        y_data[0] = rtNaN;
      } else if (orgStart - 1.0 < 0.0) {
        idx = 0;
      } else if (rtIsInf(orgStart - 1.0) && (0.0 == orgStart - 1.0)) {
        idx = 1;
        y_data[0] = rtNaN;
      } else {
        idx = (int)std::floor(orgStart - 1.0) + 1;
        loop_ub = (int)std::floor(orgStart - 1.0);
        for (i0 = 0; i0 <= loop_ub; i0++) {
          y_data[i0] = i0;
        }
      }

      fft(*(double (*)[50])&bandInput[50 * colI], orgStart, tmpxd_data,
          tmpxd_size);
      y_re = dtIn[colI] * 0.0;
      y_im = -dtIn[colI];
      tmp_size[0] = idx;
      for (i0 = 0; i0 < idx; i0++) {
        tmp_data[i0] = y_data[i0] - newStart;
      }

      ifftshift(tmp_data, tmp_size);
      b_tmp_size[0] = tmp_size[0];
      loop_ub = tmp_size[0];
      for (i0 = 0; i0 < loop_ub; i0++) {
        b_tmp_data[i0].re = 6.2831853071795862 * tmp_data[i0] / orgStart * y_re;
        b_tmp_data[i0].im = 6.2831853071795862 * tmp_data[i0] / orgStart * y_im;
      }

      b_exp(b_tmp_data, b_tmp_size);
      tmp_size[0] = tmpxd_size[0];
      loop_ub = tmpxd_size[0];
      for (i0 = 0; i0 < loop_ub; i0++) {
        b_tmpxd_data[i0].re = tmpxd_data[i0].re * b_tmp_data[i0].re -
          tmpxd_data[i0].im * b_tmp_data[i0].im;
        b_tmpxd_data[i0].im = tmpxd_data[i0].re * b_tmp_data[i0].im +
          tmpxd_data[i0].im * b_tmp_data[i0].re;
      }

      ifft(b_tmpxd_data, tmp_size, tmpxd_data, tmpxd_size);
      if (delayInt[colI] >= 0.0) {
        orgStart = delayInt[colI] + 1.0;
        newStart = delayInt[colI] + 1.0;
      } else {
        orgStart = 1.0;
        newStart = 1.0;
      }

      if (orgStart > 50.0 + delayInt[colI]) {
        i0 = 1;
        idx = 0;
      } else {
        i0 = (int)orgStart;
        idx = (int)(50.0 + delayInt[colI]);
      }

      if (newStart > 50.0 + delayInt[colI]) {
        k = 0;
      } else {
        k = (int)newStart - 1;
      }

      loop_ub = idx - i0;
      for (idx = 0; idx <= loop_ub; idx++) {
        output_data[(k + idx) + (int)maxLengthLimit * colI] = tmpxd_data[(i0 +
          idx) - 1].re;
      }
    } else {
      if (delayInt[colI] >= 0.0) {
        orgStart = 1.0;
        newStart = delayInt[colI] + 1.0;
      } else {
        orgStart = 1.0 - delayInt[colI];
        newStart = 1.0;
      }

      if (orgStart > 50.0) {
        i0 = 1;
        idx = 0;
      } else {
        i0 = (int)orgStart;
        idx = 50;
      }

      if (newStart > 50.0 + delayInt[colI]) {
        k = 0;
      } else {
        k = (int)newStart - 1;
      }

      loop_ub = idx - i0;
      for (idx = 0; idx <= loop_ub; idx++) {
        output_data[(k + idx) + (int)maxLengthLimit * colI] = bandInput[((i0 +
          idx) + 50 * colI) - 1];
      }
    }
  }

  for (i0 = 0; i0 < 4; i0++) {
    for (idx = 0; idx < 50; idx++) {
      bandOutput[idx + 50 * i0] = output_data[idx + (int)maxLengthLimit * i0];
    }
  }
}

//
// File trailer for timeDelay.cpp
//
// [EOF]
//
