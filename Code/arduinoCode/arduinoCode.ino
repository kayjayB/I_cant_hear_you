#include <bluesteinSetup.h>
#include <compressor.h>
#include <exp.h>
#include <fft.h>
#include <fft1.h>
#include <ifft.h>
#include <ifftshift.h>
#include <mpower.h>
#include <nextpow2.h>
#include <rangeCompression.h>
#include <rangeCompression_initialize.h>
#include <rangeCompression_rtwutil.h>
#include <rangeCompression_terminate.h>
#include <rangeCompression_types.h>
#include <round.h>
#include <rt_nonfinite.h>
#include <rtGetInf.h>
#include <rtGetNaN.h>
#include <rtwtypes.h>
#include <timeDelay.h>
#include <tmwtypes.h>

#include <stddef.h>
#include <stdlib.h>
#include <math.h>

//static void argInit_1x4_real_T(double result[4]);
static void argInit_50x4_real_T(double result[200]);

double Fs = 20000;
double input[400];
double directionalOutput[400];
double outputAmplification[400];
double result[50];
double compressedOutput[50];
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
int ADC_counter = 7;
double weightings[8];

const double weightTable[19][8] = { { -0.000437317784256560, -0.000437317784256560, -0.000291545189504373, -0.000291545189504373, -0.000145772594752187, -0.000145772594752187, 0, 0},
  { -0.000430673944465980, -0.000430673944465980, -0.000287115962977320, -0.000287115962977320, -0.000143557981488660, -0.000143557981488660, 0, 0},
  { -0.000410944294804333, -0.000410944294804333, -0.000273962863202889, -0.000273962863202889, -0.000136981431601444, -0.000136981431601444, 0, 0},
  { -0.000378728310692903, -0.000378728310692903, -0.000252485540461936, -0.000252485540461936, -0.000126242770230968, -0.000126242770230968, 0, 0},
  { -0.000335004858506842, -0.000335004858506842, -0.000223336572337895, -0.000223336572337895, -0.000111668286168947, -0.000111668286168947, 0, 0},
  { -0.000281102453215688, -0.000281102453215688, -0.000187401635477125, -0.000187401635477125, -9.37008177385626e-05, -9.37008177385626e-05, 0, 0},
  { -0.000218658892128280, -0.000218658892128280, -0.000145772594752187, -0.000145772594752187, -7.28862973760933e-05, -7.28862973760933e-05, 0, 0},
  { -0.000149571491250293, -0.000149571491250293, -9.97143275001950e-05, -9.97143275001950e-05, -4.98571637500975e-05, -4.98571637500975e-05, 0, 0},
  { -7.59394362974914e-05, -7.59394362974914e-05, -5.06262908649943e-05, -5.06262908649943e-05, -2.53131454324971e-05, -2.53131454324971e-05, 0, 0},
  { -2.67779912350004e-20, -2.67779912350004e-20, -1.78519941566670e-20, -1.78519941566670e-20, -8.92599707833348e-21, -8.92599707833348e-21, 0, 0},
  {7.59394362974914e-05, 7.59394362974914e-05, 5.06262908649943e-05, 5.06262908649943e-05, 2.53131454324971e-05, 2.53131454324971e-05, 0, 0},
  {0.000149571491250292, 0.000149571491250292, 9.97143275001950e-05, 9.97143275001950e-05, 4.98571637500975e-05, 4.98571637500975e-05, 0, 0},
  {0.000218658892128280, 0.000218658892128280, 0.000145772594752187, 0.000145772594752187, 7.28862973760933e-05, 7.28862973760933e-05, 0, 0},
  {0.000281102453215688, 0.000281102453215688, 0.000187401635477125, 0.000187401635477125, 9.37008177385626e-05, 9.37008177385626e-05, 0, 0},
  {0.000335004858506842, 0.000335004858506842, 0.000223336572337894, 0.000223336572337894, 0.000111668286168947, 0.000111668286168947, 0, 0},
  {0.000378728310692903, 0.000378728310692903, 0.000252485540461936, 0.000252485540461936, 0.000126242770230968, 0.000126242770230968, 0, 0},
  {0.000410944294804333, 0.000410944294804333, 0.000273962863202889, 0.000273962863202889, 0.000136981431601444, 0.000136981431601444, 0, 0},
  {0.000430673944465980, 0.000430673944465980, 0.000287115962977320, 0.000287115962977320, 0.000143557981488660, 0.000143557981488660, 0, 0},
  {0.000437317784256560, 0.000437317784256560, 0.000291545189504373, 0.000291545189504373, 0.000145772594752187, 0.000145772594752187, 0, 0 }
};

static void argInit_50x4_real_T(double result[400])
{
  int idx0;
  int idx1;

  // Loop over the array to initialize each element.
  // 4 rows and 50 columns
  for (idx0 = 0; idx0 < 50; idx0++) {
    for (idx1 = 0; idx1 < 8; idx1++) {
      result[idx0 + 50 * idx1] = 0.0;
    }
  }
}

void setup() {
  Serial.begin(115200); // Begin Serial port

  pmc_enable_periph_clk (ID_ADC);

  adc_init (ADC, SystemCoreClock, ADC_FREQ_MIN, ADC_STARTUP_FAST);

  ADC->ADC_CHER = 0xFF; //enable ADC on pin A0-A7 // for A0 to A8 use 0x4FF

  ADC->ADC_WPMR = 0x00;//Disables the write protect key, WPEN
  ADC->ADC_MR = 0x00000000;//clear all the before setted characteristics of ADC 

  PIOA->PIO_PDR |= PIO_PDR_P16; //Disable PIO Controller
  ADC->ADC_MR = ADC_MR_PRESCAL(2); // set ADC prescale to 2
  ADC->ADC_MR |= 0x80;  //set free running mode on ADC. Don't wait for triggers
  ADC->ADC_CHER = 0xFF; //enable ADC on pin A0-A7 // for A0 to A8 use 0x4FF

  ADC->ADC_MR |= ADC_MR_TRACKTIM(3); 
  ADC->ADC_MR |= ADC_MR_STARTUP_SUT8; 
  ADC->ADC_EMR = 0;
  REG_ADC_MR = (REG_ADC_MR & 0xFFF0FFFF) | 0x00020000;
  ADC->ADC_MR |= 0x40; // Set fast wakeup mode
  ADC->ADC_MR |= ADC_MR_LOWRES_BITS_12;

  //Interupt Setup
  NVIC_SetPriority(ADC_IRQn, 3);    
  adc_disable_interrupt(ADC, 0xFFFFFFFF);
  adc_enable_interrupt(ADC,ADC_IER_EOC7);
  NVIC_EnableIRQ(ADC_IRQn);  


  // DAC Setup
  analogWrite(DAC1, 0);
  analogWriteResolution(12);
  pinMode(10, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(12, OUTPUT);

  argInit_50x4_real_T(directionalOutput);
  argInit_50x4_real_T(outputAmplification);
}

void loop() {

  long t0, t;

  t0 = micros();
  // Correct way to loop
  for (int idx1 = 0; idx1 < 8; idx1++) {
    for (int idx0 = 0; idx0 < 50; idx0++) {
      while ((ADC->ADC_ISR & 0xFF) != 0xFF);
      input[idx0 + 50 * idx1] = ADC->ADC_CDR[ADC_counter];
      input[idx0 + 50 * idx1] = (input[idx0 + 50 * idx1]*0.00080586) - 1.5875; //3.3/4095
      ADC_counter--;
      if (ADC_counter == 8)
      {
        ADC_counter = 7;
      }
    }
  }

  t = micros()-t0;  // calculate elapsed time

  // Serial.print("Time per sample: ");
  // Serial.println((float)t/400);
  // Serial.print("Frequency: ");
  // Serial.println((float)400*1000000/t);
  // Serial.println();
  // delay(1000);

  // PDC_ADC->PERIPH_RPR = (uint32_t) buf; // address of buffer
  // PDC_ADC->PERIPH_RCR = BUFFER_SIZE; 
  // PDC_ADC->PERIPH_PTCR = PERIPH_PTCR_RXTEN; // enable receive

  memcpy(weightings, weightTable[0], 8 * sizeof(double) );
  timeDelay(input, weightings, Fs, directionalOutput);

 for (int idx0 = 0; idx0 < 400; idx0 = idx0 + 8) {
   Serial.println(input[idx0]);
 }

  // Apply gains to the first filter signals
  for (int j = 0; j < 400; j = j + 2) {
    outputAmplification[j] = directionalOutput[j]*gain12;
  }
  
  // Apply gains to the second filter signals
  for (int j = 1; j < 400; j = j + 2) {
    outputAmplification[j] = directionalOutput[j]*gain15;
  }

  // Adding the microphone signals together
  for (int idx1 = 0; idx1 < 50; idx1++) {
    double temp = 0;
    for (int idx0 = 0; idx0 < 8; idx0++) {
      temp += outputAmplification[idx0 + 8 * idx1] ;
    }
    result[idx1] = temp/4; // Divide by 4 to create the correct amplitude
  }

  rangeCompression(result, Fs, compressedOutput);

    for (int idx0 = 0; idx0 < 50; idx0++) {
      compressedOutput[idx0] = ((result[idx0]+ 1.5875)*4095/3.3);
      // Serial.print(result[idx0]);
      // Serial.print(",");
     // Serial.println(compressedOutput[idx0]);
  }
  
  //      dacc_write_conversion_data(DACC_INTERFACE, sum);
  
  delay(10);
}

// ADC Interrupt handler.
// void ADC_Handler(void)
// { 

 
//   if ((adc_get_status(ADC) & ADC_IER_EOC7) == ADC_IER_EOC7) 
//   { 
//     adc_disable_interrupt(ADC, ADC_IER_EOC7);
//     adcResult = ADC->ADC_CDR[7]; //clears the EOC7 bit
      
//     if (averagedEnvironment == 1) {                                    
//       fadcResult = speechFilter.processReading(adcResult);
//       } else if (averagedEnvironment == 2){                             
//        fadcResult = musicFilter.processReading(adcResult); 
//         } else if (averagedEnvironment == 3){                           
//           fadcResult = noiseFilter.processReading(adcResult/4);
//         }

//     dacc_write_conversion_data(DACC_INTERFACE, fadcResult);

//    if (alternate && adcCounter < sampleCount){
//     interuptVector[adcCounter] = (adcResult - 1551.5);
//     adcCounter++;
//     alternate = 0;
//       } else if(adcCounter < sampleCount){
//        alternate = 1;
//        }  
//   }
//   adc_enable_interrupt(ADC,ADC_IER_EOC7);
// }