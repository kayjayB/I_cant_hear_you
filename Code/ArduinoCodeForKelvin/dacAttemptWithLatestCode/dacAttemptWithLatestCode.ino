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

const int outputSize = 100;
const int numberOfInputs = 8;
const int sampleCount = outputSize * numberOfInputs; //50*8
double Fs = 44117;

static void argInit_50x4_real_T(double result[sampleCount]);
static void argInit_100_real_T(double result[100]);
static void argInit_50_real_T(double compressedOutput[outputSize]);
int directionalityAngle(volatile int x);

volatile double input[sampleCount];
double inputVector[sampleCount];
volatile int sample_counter = 0;
volatile int dac_counter = 0;
volatile int potentiometerValue = 0;
volatile bool alternate = 1;
volatile bool alternateDAC = 1;
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

double directionalOutput[sampleCount];
double outputAmplification[sampleCount];
double result[50];
double compressedOutput[outputSize];
double tempOutput[outputSize];
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
double weightings[8];
long t0, t;
double offset = 0;
double temp = 0;
int dimInput[2] = {outputSize,numberOfInputs};
int dimWeighting[2] = {1,numberOfInputs};
int dimOutput[2] = {outputSize,numberOfInputs};

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

static void argInit_50x4_real_T(double result[sampleCount])
{
  int idx0;
  int idx1;

  // Loop over the array to initialize each element.
  // 4 rows and 50 columns
  for (idx0 = 0; idx0 < outputSize; idx0++) {
    for (idx1 = 0; idx1 < numberOfInputs; idx1++) {
      result[idx0 + outputSize * idx1] = 0.0;
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

static void argInit_50_real_T(double compressedOutput[outputSize])
{
  int idx0;

  // Loop over the array to initialize each element.
  // 4 rows and 50 columns
  for (idx0 = 0; idx0 < outputSize; idx0++) {
      compressedOutput[idx0] = 0;
  }
}

void setup()
{
 Serial.begin(115200);

  argInit_100_real_T(calibration);

  analogReadResolution(12); 
  pinMode(A11, INPUT);

  for (int i=0; i<100;i++)
  {
    calibration[i] = analogRead(A11);
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
  
 adc_setup () ;         // setup ADC
 
 pmc_enable_periph_clk (TC_INTERFACE_ID + 0*3+0) ;  // clock the TC0 channel 0

 TcChannel * t = &(TC0->TC_CHANNEL)[0] ;    // pointer to TC0 registers for its channel 0
 t->TC_CCR = TC_CCR_CLKDIS ;  // disable internal clocking while setup regs
 t->TC_IDR = 0xFFFFFFFF ;     // disable interrupts
 t->TC_SR ;                   // read int status reg to clear pending
 t->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |   // use TCLK1 (prescale by 2, = 42MHz)
             TC_CMR_WAVE |                  // waveform mode
             TC_CMR_WAVSEL_UP_RC |          // count-up PWM using RC as threshold
             TC_CMR_EEVT_XC0 |     // Set external events from XC0 (this setup TIOB as output)
             TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
             TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR ;

 //t->TC_RC =  437 ;     // counter resets on RC, so sets period in terms of 42MHz clock - 96kHz
 //t->TC_RA =  218 ;     // Duty cycle
//t->TC_RC =  875 ;     // counter resets on RC, so sets period in terms of 42MHz clock - 48kHz
//t->TC_RA =  440 ; 
t->TC_RC =  952 ;     // counter resets on RC, so sets period in terms of 42MHz clock - 44.1kHz
t->TC_RA =  476 ; 
 t->TC_CMR = (t->TC_CMR & 0xFFF0FFFF) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET ;  // set clear and set from RA and RC compares
 
 t->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG ;  // re-enable local clocking and switch to hardware trigger source.

 setup_pio_TIOA0() ;  // drive Arduino pin 2 at 48kHz to bring clock out

 dac_setup();

 pinMode(3, OUTPUT);

}

void setup_pio_TIOA0 ()  // Configure Ard pin 2 as output from TC0 channel A (copy of trigger event)
{
  PIOB->PIO_PDR = PIO_PB25B_TIOA0 ;  // disable PIO control
  PIOB->PIO_IDR = PIO_PB25B_TIOA0 ;   // disable PIO interrupts
  PIOB->PIO_ABSR |= PIO_PB25B_TIOA0 ;  // switch to B peripheral
}

void adc_setup ()
{
 NVIC_EnableIRQ (ADC_IRQn) ;   // enable ADC interrupt vector
 ADC->ADC_IDR = 0xFFFFFFFF ;   // disable interrupts
 ADC->ADC_IER = 0x80 ;         // enable AD7 End-Of-Conv interrupt (Arduino pin A0)
 ADC->ADC_CHDR = 0xFFFF ;      // disable all channels
 ADC->ADC_CHER = 0x4FF ;        // ch7:A0 ch6:A1 ch5:A2 ch4:A3 ch3:A4 ch2:A5 ch1:A6 ch0:A7
 ADC->ADC_CGR = 0x15555555 ;   // All gains set to x1
 ADC->ADC_COR = 0x00000000 ;   // All offsets off
 
 ADC->ADC_MR = (ADC->ADC_MR & 0xFFFFFFF0) | (1 << 1) | ADC_MR_TRGEN ;  // 1 = trig source TIO from TC0
}

void dac_setup()
{

  pmc_enable_periph_clk (DACC_INTERFACE_ID) ; // start clocking DAC
  DACC->DACC_CR = DACC_CR_SWRST ;  // reset DAC

  DACC->DACC_MR = 
    DACC_MR_TRGEN_EN | DACC_MR_TRGSEL (1) |  // trigger 1 = TIO output of TC0
    (1 << DACC_MR_USER_SEL_Pos) |  // select channel 0
    DACC_MR_REFRESH (0x0F) |       // bit of a guess... I'm assuming refresh not needed at 48kHz
    (24 << DACC_MR_STARTUP_Pos) ;  // 24 = 1536 cycles which I think is in range 23..45us since DAC clock = 42MHz

  DACC->DACC_IDR = 0xFFFFFFFF ; // no interrupts
  DACC->DACC_CHER = DACC_CHER_CH1 << 0 ; // enable chan0

  argInit_50x4_real_T(directionalOutput);
  argInit_50x4_real_T(outputAmplification);
  argInit_50_real_T(compressedOutput);
  argInit_50_real_T(tempOutput);
  
}

void loop()
{
 

  if (sample_counter == sampleCount)
  {
      angle = directionalityAngle(potentiometerValue);
      memcpy(weightings, weightTable[9], 8 * sizeof(double));
      
      for (int j = 0; j < sampleCount; j++) inputVector[j] = input[j];
    
      timeDelay(inputVector, dimInput, weightings, dimWeighting, Fs, directionalOutput, dimOutput);
    
      // Apply gains to the first filter signals
      for (int j = 0; j < sampleCount-1; j = j + 2) {
        //outputAmplification[j] = directionalOutput[j]*gain12;
        outputAmplification[j] = directionalOutput[j]*1;
        outputAmplification[j] = directionalOutput[j+1]*1;
      }
      
      // Apply gains to the second filter signals
    //  for (int j = 1; j < sampleCount; j = j + 2) {
    //    //outputAmplification[j] = directionalOutput[j]*gain15;
    //    outputAmplification[j] = directionalOutput[j]*1;
    //  }
    
      // Adding the microphone signals together
      for (int idx1 = 0; idx1 < outputSize; idx1++) {
        temp = 0;
        for (int idx0 = 0; idx0 < numberOfInputs; idx0++) {
          temp += outputAmplification[idx0 + numberOfInputs * idx1] ;
        }
        result[idx1] = temp/4; // Divide by 4 to create the correct amplitude
      }
    
      rangeCompression(result, Fs, compressedOutput);
    
//      for (int idx0 = 0; idx0 < outputSize; idx0++) {
//        compressedOutput[idx0] = ((result[idx0]+ offset)*4095/3.3);
//      }
    
      int i=0; // needs to stay
      for (int idx0 = 0; idx0 < sampleCount; idx0 = idx0 + 8) {
        
        tempOutput[i]=(inputVector[idx0] + offset)*1240.909091;
        i=i+1;
    }
      
      sample_counter = 0;
      dac_counter = 0;
  }
}

void ADC_Handler (void)
{
  digitalWrite(3, LOW);
 //wait untill all 8 ADCs have finished thier converstion.
 while(!((ADC->ADC_ISR & ADC_ISR_EOC7) && (ADC->ADC_ISR & ADC_ISR_EOC6) && (ADC->ADC_ISR & ADC_ISR_EOC5) && (ADC->ADC_ISR & ADC_ISR_EOC4)
  && (ADC->ADC_ISR & ADC_ISR_EOC3) && (ADC->ADC_ISR & ADC_ISR_EOC2) && (ADC->ADC_ISR & ADC_ISR_EOC1) && (ADC->ADC_ISR & ADC_ISR_EOC0)));

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

    if (sample_counter < sampleCount && alternate)
    {
      // 8 channels 
      // to voltage
      input[0 + sample_counter] = adcResult0 *0.00080586 - offset;
      input[1 + sample_counter] = adcResult1 *0.00080586 - offset;
      input[2 + sample_counter] = adcResult2 *0.00080586 - offset;
      input[3 + sample_counter] = adcResult3 *0.00080586 - offset;
      input[4 + sample_counter] = adcResult4 *0.00080586 - offset;
      input[5 + sample_counter] = adcResult5 *0.00080586 - offset;
      input[6 + sample_counter] = adcResult6 *0.00080586 - offset;
      input[7 + sample_counter] = adcResult7 *0.00080586 - offset;

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

      alternate = 0;

//      dacc_set_channel_selection(DACC_INTERFACE, 1);       //select DAC channel 1
//      dacc_write_conversion_data(DACC_INTERFACE, tempOutput[dac_counter]);//write on DAC
//      dac_counter++;

      sample_counter+=numberOfInputs;
     
    } 
    else if(sample_counter < sampleCount)
   {
       alternate = 1;
    }

    if (dac_counter < outputSize && alternateDAC)
    {

        dacc_set_channel_selection(DACC_INTERFACE, 1);       //select DAC channel 1
        dacc_write_conversion_data(DACC_INTERFACE, tempOutput[dac_counter]);//write on DAC
        dac_counter++;
        alternateDAC = 0;
    }
    else if (dac_counter < outputSize)
    {
      alternateDAC = 1;
    }
        
    if (dac_counter >= outputSize)
    {
      dac_counter = 0;
    }
    
  digitalWrite(3, HIGH);
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

