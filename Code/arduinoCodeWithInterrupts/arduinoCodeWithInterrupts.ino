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

static void argInit_50x4_real_T(double result[200]);
static void argInit_100_real_T(unsigned int result[100]);
int directionalityAngle(volatile int x);

const int sampleCount = 400; //50*8
double Fs = 18000;

volatile double input[sampleCount];
double inputVector[sampleCount];
volatile int sample_counter = 0;
volatile int number_of_interrupts = 0;
volatile int potentiometerValue = 0;
volatile bool alternate = 1;
unsigned int adcResult0 = 0;
unsigned int adcResult1 = 0;
unsigned int adcResult2 = 0;
unsigned int adcResult3 = 0;
unsigned int adcResult4 = 0;
unsigned int adcResult5 = 0;
unsigned int adcResult6 = 0;
unsigned int adcResult7 = 0;
unsigned int adcResult8 = 0;
unsigned int adcResult9 = 0;

double directionalOutput[400];
double outputAmplification[400];
double result[50];
double compressedOutput[50];
unsigned int calibration[100];
float ADC_value1[5];
float ADC_value2[5];
float outputFilter12[5];
float outputFilter15[5];
//const float audiogram[16]={15, 13.7, 12, 10, 10, 10, 10, 11.25, 13, 15, 13.75, 12.125, 10, 7.5, 7.25, 20};
const float audiogram[16] = {5.623, 4.842, 3.981, 3.162, 3.162, 3.162, 3.162, 3.652, 4.467, 5.623, 4.870, 4.039, 3.162, 2.371, 2.304, 10.0};
unsigned int j;
int angle = 0;
const int analogInputA0 = A0;
const float gain12 = audiogram[11];
const float gain15 = audiogram[14]; // filt 12 and filt 15
float sum;
int ADC_counter = 7;
double weightings[8];
long t0, t;
double offset = 0;

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

static void argInit_100_real_T(unsigned int result[100])
{
  int idx0;

  // Loop over the array to initialize each element.
  // 4 rows and 50 columns
  for (idx0 = 0; idx0 < 100; idx0++) {
      result[idx0] = 0;
  }
}

void setup() {
  Serial.begin(115200); // Begin Serial port

  argInit_100_real_T(calibration);

  analogReadResolution(12); 
  pinMode(A11, INPUT);

  for (int i=0; i<100;i++)
  {
    //while ((ADC->ADC_ISR & 0xFF) != 0xFF);
    calibration[i] = analogRead(A11);
   // Serial.println(calibration[i]);
    delay(5);
  }
  for (int i=0; i<100; i++) 
  {
    offset += calibration[i];
  }
  offset = offset/100; // average
  // t0 = micros();
  offset = offset*0.00080586; 
  //Serial.println(offset, 8);

  pmc_enable_periph_clk(ID_ADC);

  adc_init (ADC, SystemCoreClock, ADC_FREQ_MAX*2, 3);

  ADC->ADC_WPMR = 0x00;//Disables the write protect key, WPEN
  ADC->ADC_MR = 0x00000000;//clear all the before setted characteristics of ADC 

  PIOA->PIO_PDR |= PIO_PDR_P16; //Disable PIO Controller
  ADC->ADC_MR = ADC_MR_PRESCAL(2); // set ADC prescale to 2
  ADC->ADC_MR |= 0x80;  //set free running mode on ADC. Don't wait for triggers
  //ADC->ADC_CHER = 0xF0; //enable ADC on pin A0-A3 // for A0 to A8 use 0x4FF
  //ADC->ADC_CHER = 0xFF; //enable ADC on pin A0-A7 // for A0 to A8 use 0x4FF
  ADC->ADC_CHER = 0x4FF; //enable ADC on pin A0-A8
  //ADC->ADC_CHER = 0xF8; //enable ADC on pin A0-A4 // for A0 to A8 use 0x4FF

  ADC->ADC_MR |= ADC_MR_TRACKTIM(3); 
  ADC->ADC_MR |= ADC_MR_STARTUP_SUT8; 
  ADC->ADC_EMR = 0;
  REG_ADC_MR = (REG_ADC_MR & 0xFFF0FFFF) | 0x00020000;
  ADC->ADC_MR |= 0x40; // Set fast wakeup mode
  ADC->ADC_MR |= ADC_MR_LOWRES_BITS_12;
  adc_disable_ts(ADC);

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
  //t0 = micros();

  // Serial.print("Time per sample: ");
  // Serial.println((float)t/400);
  // Serial.print("Frequency: ");
  // Serial.println((float)400*1000000/t);
  // Serial.println();
  // delay(1000);
  angle = directionalityAngle(potentiometerValue);
  memcpy(weightings, weightTable[18], 8 * sizeof(double));

  for (int j = 0; j < sampleCount; j++) inputVector[j] = input[j];

  // for (int idx0 = 0; idx0 < 400; idx0 = idx0 + 8) {
  //  Serial.println(inputVector[idx0]);
  // }

  // for (int idx0 = 0; idx0 < 8; idx0++) {
  //   Serial.print("Row in weight table: ");
  //   Serial.print(angle);
  //   Serial.print(" Weightings: ");
  //   Serial.println(weightings[idx0],10);
  // }


  timeDelay(inputVector, weightings, Fs, directionalOutput);

  // Apply gains to the first filter signals
  for (int j = 0; j < 400; j = j + 2) {
    //outputAmplification[j] = directionalOutput[j]*gain12;
    outputAmplification[j] = directionalOutput[j]*1;
  }
  
  // Apply gains to the second filter signals
  for (int j = 1; j < 400; j = j + 2) {
    //outputAmplification[j] = directionalOutput[j]*gain15;
    outputAmplification[j] = directionalOutput[j]*1;
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
    compressedOutput[idx0] = ((compressedOutput[idx0]+ offset)*4095/3.3);
    Serial.print(angle);
    // Serial.print(result[idx0]);
    Serial.print(",");
    Serial.println(compressedOutput[idx0]);
}
  
  //      dacc_write_conversion_data(DACC_INTERFACE, sum);
  sample_counter = 0;
  

  // t = micros()-t0;  // calculate elapsed time
  // Serial.print("Frequency: ");
  // Serial.println((float)9*number_of_interrupts*1000000/t);
  // Serial.println();

  number_of_interrupts = 0;
}

// ADC Interrupt handler.
void ADC_Handler(void)
{ 
  if ((adc_get_status(ADC) & ADC_IER_EOC7) == ADC_IER_EOC7) 
  { 
    adc_disable_interrupt(ADC, ADC_IER_EOC7);
    number_of_interrupts++ ;
    // Read in all the ADC values - 8 channels
    adcResult0 = ADC->ADC_CDR[7];
    adcResult1 = ADC->ADC_CDR[6];
    adcResult2 = ADC->ADC_CDR[5];
    adcResult3 = ADC->ADC_CDR[4];
    adcResult4 = ADC->ADC_CDR[3];
    adcResult5 = ADC->ADC_CDR[2];
    adcResult6 = ADC->ADC_CDR[1];
    adcResult7 = ADC->ADC_CDR[0];
    adcResult9 = ADC->ADC_CDR[10]; //pot

    // Read in all the ADC values - 4 channels
    // adcResult0 = ADC->ADC_CDR[7];
    // adcResult1 = ADC->ADC_CDR[6];
    // adcResult2 = ADC->ADC_CDR[5];
    // adcResult3 = ADC->ADC_CDR[4];

    // Read in all the ADC values - 4 channels + pot on channel A4
    // adcResult0 = ADC->ADC_CDR[7];
    // adcResult1 = ADC->ADC_CDR[6];
    // adcResult2 = ADC->ADC_CDR[5];
    // adcResult3 = ADC->ADC_CDR[4];
    // adcResult4 = ADC->ADC_CDR[3];

    if (alternate && sample_counter < sampleCount)
    {
      // 8 channels 
      // to voltage
      input[0 + sample_counter] = adcResult0*0.00080586 - offset;
      input[1 + sample_counter] = adcResult1*0.00080586 - offset;
      input[2 + sample_counter] = adcResult2*0.00080586 - offset;
      input[3 + sample_counter] = adcResult3*0.00080586 - offset;
      input[4 + sample_counter] = adcResult4*0.00080586 - offset;
      input[5 + sample_counter] = adcResult5*0.00080586 - offset;
      input[6 + sample_counter] = adcResult6*0.00080586 - offset;
      input[7 + sample_counter] = adcResult7*0.00080586 - offset;

      // 4 channels
      // input[0 + sample_counter] = adcResult0*0.00080586 - 1.5875;
      // input[1 + sample_counter] = adcResult1*0.00080586 - 1.5875;
      // input[2 + sample_counter] = adcResult2*0.00080586 - 1.5875;
      // input[3 + sample_counter] = adcResult3*0.00080586 - 1.5875;
      // input[4 + sample_counter] = adcResult0*0.00080586 - 1.5875;
      // input[5 + sample_counter] = adcResult1*0.00080586 - 1.5875;
      // input[6 + sample_counter] = adcResult2*0.00080586 - 1.5875;
      // input[7 + sample_counter] = adcResult3*0.00080586 - 1.5875;

      // pot 
      potentiometerValue = adcResult9;

      sample_counter+=8;
      alternate = 0;

    } 
    else if(sample_counter < sampleCount)
    {
      alternate = 1;
    }
    // if (sample_counter == sampleCount)
    // {
    //   t = micros()-t0;  // calculate elapsed time
    //   Serial.print("Time per sample: ");
    //   Serial.println((float)t/400);
    //   Serial.print("Frequency: ");
    //   Serial.println((float)400*1000000/t);
    //   Serial.println();
    //   t0 = micros();

    // }
  }
  adc_enable_interrupt(ADC,ADC_IER_EOC7);
}

int directionalityAngle(volatile int x)
{
  if (x >= 0 && x <= 106 ) return 0;
  else if (x >= 107 && x <= 213 ) return 1;
  else if (x >= 214 && x <= 320 ) return 2;
  else if (x >= 321 && x <= 427 ) return 3;
  else if (x >= 428 && x <= 534 ) return 4;
  else if (x >= 535 && x <= 641 ) return 5;
  else if (x >= 642 && x <= 748 ) return 6;
  else if (x >= 749 && x <= 855 ) return 7;
  else if (x >= 856 && x <= 962 ) return 8;
  else if (x >= 963 && x <= 1069 ) return 9;
  else if (x >= 1070 && x <= 1176 ) return 10;
  else if (x >= 1177 && x <= 1283 ) return 11;
  else if (x >= 1284 && x <= 1390 ) return 12;
  else if (x >= 1391 && x <= 1497 ) return 13;
  else if (x >= 1498 && x <= 1604 ) return 14;
  else if (x >= 1605 && x <= 1711 ) return 15;
  else if (x >= 1712 && x <= 1818 ) return 16;
  else if (x >= 1819 && x <= 1925 ) return 17;
  else return 18;

}