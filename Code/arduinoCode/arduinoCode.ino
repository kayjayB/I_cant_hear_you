#include <bluesteinSetup.h>
#include <exp.h>
#include <fft.h>
#include <fft1.h>
#include <ifft.h>
#include <ifftshift.h>
#include <mpower.h>
#include <nextpow2.h>
#include <round.h>
#include <rt_nonfinite.h>
#include <rtGetInf.h>
#include <rtGetNaN.h>
#include <rtwtypes.h>
#include <timeDelay.h>
#include <timeDelay_initialize.h>
#include <timeDelay_terminate.h>
#include <timeDelay_types.h>
#include <tmwtypes.h>


#include <stddef.h>
#include <stdlib.h>
#include <math.h>

//static void argInit_1x4_real_T(double result[4]);
static void argInit_50x4_real_T(double result[200]);

double Fs = 40000;
double input[200];
double bandOutput12[200];
double weightings[4] = { -4.3732 * pow(10, -4), -2.9155 * pow(10, -4), -1.4577 * pow(10, -4), 0};
float ADC_value1[5];
float ADC_value2[5];
float outputFilter12[5];
float outputFilter15[5];
//const float audiogram[16]={15, 13.7, 12, 10, 10, 10, 10, 11.25, 13, 15, 13.75, 12.125, 10, 7.5, 7.25, 20};
const float audiogram[16] = {5.623, 4.842, 3.981, 3.162, 3.162, 3.162, 3.162, 3.652, 4.467, 5.623, 4.870, 4.039, 3.162, 2.371, 2.304, 10.0};
unsigned int j;
const int analogInputA0 = A0;
const float gain12 = audiogram[11];
const float gain15 = audiogram[14]; // filt 12 and filt 15
float sum;

static void argInit_50x4_real_T(double result[200])
{
  int idx0;
  int idx1;

  // Loop over the array to initialize each element.
  // 4 rows and 50 columns
  for (idx0 = 0; idx0 < 50; idx0++) {
    for (idx1 = 0; idx1 < 4; idx1++) {
      result[idx0 + 50*idx1] = 0.0;
    }
  }
}

void setup() {
  Serial.begin(115200); // Begin Serial port
  ADC->ADC_MR |= 0x80;  //set free running mode on ADC
  ADC->ADC_CHER = 0xFF; //enable ADC on pin A0-A7
  analogWriteResolution(12);

  // DAC Setup
  analogWrite(DAC1, 0);
  analogWriteResolution(12);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);


  argInit_50x4_real_T(bandOutput12);
}

void loop() {
  for (int idx0 = 0; idx0 < 50; idx0++) {
    for (int idx1 = 0; idx1 < 4; idx1++) {
      while ((ADC->ADC_ISR & 0x80) == 0);
      input[idx0 + 50 * idx1] = ADC->ADC_CDR[7];
    }
  }

  //  for (j = 0; j < 512; j++) {
  //    while((ADC->ADC_ISR & 0x80)==0);
  //    input[j]
  //    ADC_value1[j]= ADC->ADC_CDR[7];
  //    ADC_value2[j]= ADC->ADC_CDR[6];
  //  }
 // Serial.println("Before timedelay");
  timeDelay(input, weightings, Fs, bandOutput12);
 // Serial.println("After time delay");
  
//  for (int idx0 = 0; idx0 < 4; idx0++) {
//    for (int idx1 = 0; idx1 < 50; idx1++) {
//      Serial.println(bandOutput12[idx1 + 50 * idx0]);
//    }
//  }
    for (int idx0 = 0; idx0 < 200; idx0= idx0+4) {
      Serial.println(bandOutput12[idx0]);
    }

  
  //    for(j=0;j<512;j++) {
  //      outputFilter12[j] = ADC_value1[j]*gain12;
  //      outputFilter15[j] = ADC_value2[j]*gain15;
  ////outputFilter12[j] = outputFilter12[j]*(3.3 / 4095.0);
  ////      Serial.print(outputFilter12[j]);
  ////      Serial.print(" ");
  ////      Serial.println(outputFilter15[j]);
  //
  //
  //      sum = outputFilter15[j] + outputFilter12[j];
  //      dacc_write_conversion_data(DACC_INTERFACE, sum);
  //    }
  delay(2);
}
