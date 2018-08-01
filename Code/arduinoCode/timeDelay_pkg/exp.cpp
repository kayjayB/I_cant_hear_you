//
// Trial License - for use to evaluate programs for possible purchase as
// an end-user only.
// File: exp.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 01-Aug-2018 12:42:33
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "timeDelay.h"
#include "exp.h"

// Function Definitions

//
// Arguments    : creal_T x_data[]
//                int x_size[1]
// Return Type  : void
//
void b_exp(creal_T x_data[], int x_size[1])
{
  int nx;
  int k;
  double x_re;
  double r;
  nx = x_size[0];
  for (k = 0; k < nx; k++) {
    if (x_data[k].im == 0.0) {
      x_re = std::exp(x_data[k].re);
      r = 0.0;
    } else if (rtIsInf(x_data[k].im) && rtIsInf(x_data[k].re) && (x_data[k].re <
                0.0)) {
      x_re = 0.0;
      r = 0.0;
    } else {
      r = std::exp(x_data[k].re / 2.0);
      x_re = r * (r * std::cos(x_data[k].im));
      r *= r * std::sin(x_data[k].im);
    }

    x_data[k].re = x_re;
    x_data[k].im = r;
  }
}

//
// File trailer for exp.cpp
//
// [EOF]
//
