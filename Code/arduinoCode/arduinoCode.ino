A#include <FIR.h>

#include "src/filterLibrary/filterBand39Function.h"
#include "src/filterLibrary/filterBand39Function_types.h"
#include "src/filterLibrary/rt_nonfinite.h"
#include "src/filterLibrary/rtGetInf.h"
#include "src/filterLibrary/rtGetNaN.h"
#include <stddef.h>
#include <stdlib.h>

static void argInit_1x1025_real32_T(float result[1025]);

static void argInit_1x1025_real32_T(float result[1025])
{
  int idx1;
  for (idx1 = 0; idx1 < 1025; idx1++) {
    result[idx1] = 0.0F;
  }
}

float ADC_value[1025];
float output[1025];
const float audiogram[16]={15, 13.7, 12, 10, 10, 10, 10, 11.25, 13 , 15, 13.75, 12.125, 10, 7.5, 7.25, 20};
unsigned int j;
const int analogInputA0 = A0;

void setup() {
  Serial.begin(230400); // Begin Serial port
  //pinMode(1, INPUT);
  //ADCSRA &= ~(1 << ADPS2) | (1 << ADPS1) | (1 <<
  //ADPS0); // ADC settings
  //ADCSRA |= 1 << ADPS2; // set 1 MHz frequency
  filterBand39Function_initialize();
  argInit_1x1025_real32_T(output);
  
}

void loop() {
  
  for (j = 0; j < 1024; j++) {
    ADC_value[j] = analogRead(analogInputA0);
  }
  filterFrames(ADC_value, audiogram, output);
  //  for(j=0;j<100;j++) {
  //    Serial.println(ADC_value [j]);
  //    }
  //delay(10);
}
