//
// Trial License - for use to evaluate programs for possible purchase as
// an end-user only.
// File: nextpow2.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 01-Aug-2018 12:42:33
//

// Include Files
#include <math.h>
#include "rt_nonfinite.h"
#include "timeDelay.h"
#include "nextpow2.h"

// Function Definitions

//
// Arguments    : double n
// Return Type  : double
//
double nextpow2(double n)
{
  double p;
  double f;
  int eint;
  p = n;
  if ((!rtIsInf(n)) && (!rtIsNaN(n))) {
    f = frexp(n, &eint);
    p = eint;
    if (f == 0.5) {
      p = (double)eint - 1.0;
    }
  }

  return p;
}

//
// File trailer for nextpow2.cpp
//
// [EOF]
//
