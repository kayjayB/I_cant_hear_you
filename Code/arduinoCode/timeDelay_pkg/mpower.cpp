//
// Trial License - for use to evaluate programs for possible purchase as
// an end-user only.
// File: mpower.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 01-Aug-2018 12:42:33
//

// Include Files
#include <cmath>
#include <math.h>
#include "rt_nonfinite.h"
#include "timeDelay.h"
#include "mpower.h"

// Function Declarations
static double rt_powd_snf(double u0, double u1);

// Function Definitions

//
// Arguments    : double u0
//                double u1
// Return Type  : double
//
static double rt_powd_snf(double u0, double u1)
{
  double y;
  double d1;
  double d2;
  if (rtIsNaN(u0) || rtIsNaN(u1)) {
    y = rtNaN;
  } else {
    d1 = std::abs(u0);
    d2 = std::abs(u1);
    if (rtIsInf(u1)) {
      if (d1 == 1.0) {
        y = 1.0;
      } else if (d1 > 1.0) {
        if (u1 > 0.0) {
          y = rtInf;
        } else {
          y = 0.0;
        }
      } else if (u1 > 0.0) {
        y = 0.0;
      } else {
        y = rtInf;
      }
    } else if (d2 == 0.0) {
      y = 1.0;
    } else if (d2 == 1.0) {
      if (u1 > 0.0) {
        y = u0;
      } else {
        y = 1.0 / u0;
      }
    } else if (u1 == 2.0) {
      y = u0 * u0;
    } else if ((u1 == 0.5) && (u0 >= 0.0)) {
      y = std::sqrt(u0);
    } else if ((u0 < 0.0) && (u1 > std::floor(u1))) {
      y = rtNaN;
    } else {
      y = pow(u0, u1);
    }
  }

  return y;
}

//
// Arguments    : double b
// Return Type  : double
//
double mpower(double b)
{
  return rt_powd_snf(2.0, b);
}

//
// File trailer for mpower.cpp
//
// [EOF]
//
