//
// Trial License - for use to evaluate programs for possible purchase as
// an end-user only.
// File: round.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 01-Aug-2018 12:42:33
//

// Include Files
#include <cmath>
#include "rt_nonfinite.h"
#include "timeDelay.h"
#include "round.h"

// Function Declarations
static double rt_roundd_snf(double u);

// Function Definitions

//
// Arguments    : double u
// Return Type  : double
//
static double rt_roundd_snf(double u)
{
  double y;
  if (std::abs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = std::floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = std::ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

//
// Arguments    : double x[4]
// Return Type  : void
//
void b_round(double x[4])
{
  int k;
  for (k = 0; k < 4; k++) {
    x[k] = rt_roundd_snf(x[k]);
  }
}

//
// File trailer for round.cpp
//
// [EOF]
//
