//
// Trial License - for use to evaluate programs for possible purchase as
// an end-user only.
// File: ifft.cpp
//
// MATLAB Coder version            : 4.0
// C/C++ source code generated on  : 01-Aug-2018 12:42:33
//

// Include Files
#include <string.h>
#include "rt_nonfinite.h"
#include "timeDelay.h"
#include "ifft.h"
#include "fft1.h"

// Function Definitions

//
// Arguments    : const creal_T x_data[]
//                const int x_size[1]
//                creal_T y_data[]
//                int y_size[1]
// Return Type  : void
//
void ifft(const creal_T x_data[], const int x_size[1], creal_T y_data[], int
          y_size[1])
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
  creal_T b_y_data[512];
  int b_y_size[1];
  if (x_size[0] == 0) {
    y_size[0] = 0;
  } else {
    useRadix2 = ((x_size[0] & (x_size[0] - 1)) == 0);
    get_algo_sizes(x_size[0], useRadix2, &N2blue, &nRows);
    b_generate_twiddle_tables(nRows, useRadix2, costab_data, costab_size,
      sintab_data, sintab_size, sintabinv_data, sintabinv_size);
    if (useRadix2) {
      c_r2br_r2dit_trig(x_data, x_size, x_size[0], costab_data, sintab_data,
                        b_y_data, b_y_size);
      y_size[0] = b_y_size[0];
      N2blue = b_y_size[0];
      if (0 <= N2blue - 1) {
        memcpy(&y_data[0], &b_y_data[0], (unsigned int)(N2blue * (int)sizeof
                (creal_T)));
      }
    } else {
      b_dobluesteinfft(x_data, x_size, N2blue, x_size[0], costab_data,
                       sintab_data, sintabinv_data, y_data, y_size);
    }
  }
}

//
// File trailer for ifft.cpp
//
// [EOF]
//
