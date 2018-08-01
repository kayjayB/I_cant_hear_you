//
// Trial License - for use to evaluate programs for possible purchase as
// an end-user only.
// File: ifftshift.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 01-Aug-2018 12:42:33
//

// Include Files
#include "rt_nonfinite.h"
#include "timeDelay.h"
#include "ifftshift.h"

// Function Definitions

//
// Arguments    : double x_data[]
//                int x_size[1]
// Return Type  : void
//
void ifftshift(double x_data[], int x_size[1])
{
  int dim;
  int i2;
  int vlend2;
  int vstride;
  int k;
  int midoffset;
  int i1;
  int j;
  int ia;
  int ib;
  double xtmp;
  int ic;
  for (dim = 0; dim < 2; dim++) {
    if (dim + 1 <= 1) {
      i2 = x_size[0];
    } else {
      i2 = 1;
    }

    if (!(i2 <= 1)) {
      vlend2 = i2 / 2;
      if (vlend2 << 1 == i2) {
        if (dim + 1 <= 1) {
          i2 = x_size[0];
        } else {
          i2 = 1;
        }

        if (!(i2 <= 1)) {
          vlend2 = i2 / 2;
          vstride = 1;
          k = 1;
          while (k <= dim) {
            vstride *= x_size[0];
            k = 2;
          }

          midoffset = vlend2 * vstride;
          if (vlend2 << 1 == i2) {
            i1 = -1;
            for (j = 1; j <= vstride; j++) {
              i1++;
              ia = i1;
              ib = i1 + midoffset;
              for (k = 1; k <= vlend2; k++) {
                xtmp = x_data[ia];
                x_data[ia] = x_data[ib];
                x_data[ib] = xtmp;
                ia += vstride;
                ib += vstride;
              }
            }
          } else {
            i1 = -1;
            for (j = 1; j <= vstride; j++) {
              i1++;
              ia = i1;
              ib = (i1 + midoffset) + 1;
              xtmp = x_data[ib - 1];
              for (k = 1; k <= vlend2; k++) {
                ic = ib + vstride;
                x_data[ib - 1] = x_data[ia];
                x_data[ia] = x_data[ic - 1];
                ia += vstride;
                ib = ic;
              }

              x_data[ib - 1] = xtmp;
            }
          }
        }
      } else {
        vstride = 1;
        k = 1;
        while (k <= dim) {
          vstride *= x_size[0];
          k = 2;
        }

        midoffset = vlend2 * vstride;
        i1 = -1;
        i2 = (i2 - 1) * vstride - 1;
        for (j = 1; j <= vstride; j++) {
          i1++;
          i2++;
          ia = i1 + midoffset;
          ib = i2;
          xtmp = x_data[i2];
          for (k = 1; k <= vlend2; k++) {
            ia -= vstride;
            ic = ib;
            ib -= vstride;
            x_data[ic] = x_data[ia];
            x_data[ia] = x_data[ib];
          }

          x_data[ib] = xtmp;
        }
      }
    }
  }
}

//
// File trailer for ifftshift.cpp
//
// [EOF]
//
