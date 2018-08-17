#include <abs.h>
#include <bluesteinSetup.h>
#include <compressor.h>
#include <CompressorBase.h>
#include <exp.h>
#include <fft.h>
#include <fft1.h>
#include <ifft.h>
#include <ifftshift.h>
#include <mpower.h>
#include <nextpow2.h>
#include <power.h>
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

#include <FIR.h>


#include <stddef.h>
#include <stdlib.h>
#include <math.h>

const int outputSize = 100;
const int numberOfInputs = 8;
const int sampleCount = outputSize * numberOfInputs; //50*8
double Fs = 22059;

static void argInit_50x4_real_T(double result[sampleCount]);
static void argInit_100_real_T(unsigned int result[100]);
int directionalityAngle(volatile int x);
static void argInit_50_real_T(double compressedOutput[outputSize]);
static void argInit_50x4_volatile(volatile double input[sampleCount]);
void directionality180(double result[outputSize], double inputVector[sampleCount]);
void directionality0(double result[outputSize], double inputVector[sampleCount]);
void directionality90(double result[outputSize], double inputVector[sampleCount]);

volatile double input[sampleCount];
double inputVector[sampleCount];
double inputVectorOmni[outputSize * 2];
volatile int sample_counter = 0;
volatile int dac_counter = 0;
volatile int number_of_interrupts = 0;
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
double outputAmplificationOmni[outputSize * 2];
double result[outputSize];
double filteredResult[outputSize];
double compressedOutput[outputSize];
double tempOutput[outputSize];
unsigned int calibration[100];
const int modeSwitchPin = 5;

const float audiogram[16] = {5.623, 4.842, 3.981, 3.162, 3.162, 3.162, 3.162, 3.652, 4.467, 5.623, 4.870, 4.039, 3.162, 2.371, 2.304, 10.0};
unsigned int j;
int angle = 0;
const int analogInputA0 = A0;
const float gain12 = audiogram[11];
const float gain15 = audiogram[14]; // filt 12 and filt 15
long t0, t;
double offset = 0;
double temp = 0;

bool mode = 1;

// Stuff for new directionality -- for Fs of 44kHz
const double shift0Deg[4] = {0, 3, 6, 9};
const double shift180Deg[4] = {9, 6, 3, 0};
const double shift60Deg[4] = {0, 2, 4, 6};
const double shift120Deg[4] = {6, 4, 2, 0};

//// for Fs of 48kHz
//const double shift0Deg[4] = {0, 2, 4, 6};
//const double shift180Deg[4] = {6, 4, 2, 0};
//const double shift60Deg[4] = {0, 2, 4, 6};
//const double shift120Deg[4] = {6, 4, 2, 0};

double mic1[outputSize];
double mic2[outputSize];
double mic3[outputSize];
double mic4[outputSize];

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

static void argInit_50x4_volatile(volatile double input[sampleCount])
{
  int idx0;
  int idx1;

  // Loop over the array to initialize each element.
  // 4 rows and 50 columns
  for (idx0 = 0; idx0 < outputSize; idx0++) {
    for (idx1 = 0; idx1 < numberOfInputs; idx1++) {
      input[idx0 + outputSize * idx1] = 0;
    }
  }
}

static void argInit_100_real_T(unsigned int result[100])
{ // Initialize the calibration
  int idx0;
  for (idx0 = 0; idx0 < 100; idx0++) {
    result[idx0] = 0;
  }
}

static void argInit_50_real_T(double compressedOutput[outputSize])
{
  int idx0;

  for (idx0 = 0; idx0 < outputSize; idx0++) {
    compressedOutput[idx0] = 0;
  }
}

void setup()
{
  Serial.begin(115200);

  calibration_setup();

  timer_setup();

  adc_setup ();         // setup ADC

  setup_pio_TIOA0();  // drive Arduino pin 2 at 48kHz to bring clock out

  variableInit();

  dac_setup();

  pinMode(3, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(modeSwitchPin, INPUT);
}

void calibration_setup()
{
  argInit_100_real_T(calibration);

  analogReadResolution(12);
  pinMode(A11, INPUT);

  for (int i = 0; i < 100; i++)
  {
    calibration[i] = analogRead(A11);
    // Serial.println(calibration[i]);
    delay(5);
  }

  for (int i = 0; i < 100; i++)
  {
    offset += calibration[i];
  }
  offset = offset / 100; // average

  offset = offset * 0.00080586;
}

void timer_setup()
{
  pmc_enable_periph_clk (TC_INTERFACE_ID + 0 * 3 + 0) ; // clock the TC0 channel 0

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

//t->TC_RC =  875 ;     // counter resets on RC, so sets period in terms of 42MHz clock - 48kHz
//t->TC_RA =  440 ; 

  t->TC_RC =  952 ;     // counter resets on RC, so sets period in terms of 42MHz clock - 44.1kHz
  t->TC_RA =  476 ;
  t->TC_CMR = (t->TC_CMR & 0xFFF0FFFF) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET ;  // set clear and set from RA and RC compares

  t->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG ;  // re-enable local clocking and switch to hardware trigger source.
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
  ADC->ADC_CHER = 0x4FF ;        // ch7:A0 ch6:A1 ch5:A2 ch4:A3 ch3:A4 ch2:A5 ch1:A6 ch0:A7 ch10:A8
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
    (1 << DACC_MR_USER_SEL_Pos) |  // select channel 1
    DACC_MR_REFRESH (0x0F) |       // bit of a guess... I'm assuming refresh not needed at 48kHz
    (24 << DACC_MR_STARTUP_Pos) ;  // 24 = 1536 cycles which I think is in range 23..45us since DAC clock = 42MHz

  DACC->DACC_IDR = 0xFFFFFFFF ; // no interrupts
  DACC->DACC_CHER = DACC_CHER_CH1 << 0 ; // enable chan1

}

void variableInit()
{
  argInit_50x4_real_T(directionalOutput);
  argInit_50x4_real_T(outputAmplification);
  argInit_50_real_T(compressedOutput);
  argInit_50x4_volatile(input);
  argInit_50_real_T(result);
}

void loop()
{
  mode = digitalRead(modeSwitchPin);
  mode = 1;
  if (mode == 1) // if directional mode is selected
  {
    if (sample_counter == sampleCount)
    {

      for (int j = 0; j < sampleCount; j++) inputVector[j] = input[j];

      angle = directionalityAngle(potentiometerValue);
      angle = 90;
//      // Apply gains to the signals
//      for (int j = 0; j < sampleCount - 1; j = j + 2) {
//        outputAmplification[j] = directionalOutput[j] * 1;
//        outputAmplification[j + 1] = directionalOutput[j + 1] * 1;
//      }

      if (angle == 0)
      {
        directionality0(result, inputVector);
      }
      else if (angle == 90)
      {
        directionality90(result, inputVector);
      }
      else if (angle == 180)
      {
        directionality180(result, inputVector);
      }
      else if (angle == 60)
      {
        directionality60(result, inputVector);
      }
      else if (angle == 120)
      {
        directionality120(result, inputVector);
      }

      //rangeCompression(result, Fs, compressedOutput);

      for (int idx0 = 0; idx0 < outputSize; idx0++) {
        compressedOutput[idx0] = ((result[idx0] + offset) * 1240.909091);
      }

      sample_counter = 0;
      __asm__("nop\n\t"); //nop
      // dac_counter = 0;
    }
  }
  else if (mode == 0){
    if (sample_counter == sampleCount)
    {
      for (int j = 0; j < sampleCount; j++) inputVector[j] = input[j];
      int index = 0;
      for (int j = 6; j < sampleCount; j = j + 8) // For directional mode, only use mic 2 (A2 and A3)
      {
        inputVectorOmni[index] = input[j];
        inputVectorOmni[index + 1] = input[j + 1];
        index = index + 2;
      }

      // Apply gains to the signals
      for (int j = 0; j < outputSize * 2 - 1; j = j + 2)
      {
        outputAmplificationOmni[j] = inputVectorOmni[j] * gain12;
        outputAmplificationOmni[j + 1] = inputVectorOmni[j + 1] * gain15;
      }

      // Adding the microphone signals together
      for (int idx1 = 0; idx1 < outputSize; idx1++)
      {
        double temp = 0;
        for (int idx0 = 0; idx0 < 2; idx0++)
        {
          temp += outputAmplificationOmni[idx0 + 2 * idx1] ;
        }
        result[idx1] = temp; // Divide by 4 to create the correct amplitude
      }

      //  rangeCompression(result, Fs, compressedOutput);

      for (int idx0 = 0; idx0 < outputSize; idx0++)
      {
        compressedOutput[idx0] = ((result[idx0] + offset) * 1240.909091);
      }

      sample_counter = 0;
      __asm__("nop\n\t"); //nop
      // dac_counter = 0;
    }
  }
}

void ADC_Handler (void)
{
  digitalWrite(3, LOW);
  digitalWrite(4, LOW);
  // delay(0.004);
  //wait untill all 8 ADCs have finished thier converstion.
  while (!((ADC->ADC_ISR & ADC_ISR_EOC7) && (ADC->ADC_ISR & ADC_ISR_EOC6) && (ADC->ADC_ISR & ADC_ISR_EOC5) && (ADC->ADC_ISR & ADC_ISR_EOC4)
           && (ADC->ADC_ISR & ADC_ISR_EOC3) && (ADC->ADC_ISR & ADC_ISR_EOC2) && (ADC->ADC_ISR & ADC_ISR_EOC1) && (ADC->ADC_ISR & ADC_ISR_EOC0)));

  adcResult0 = ADC->ADC_CDR[7];
  adcResult1 = ADC->ADC_CDR[6];
  adcResult2 = ADC->ADC_CDR[5];
  adcResult3 = ADC->ADC_CDR[4];
  adcResult4 = ADC->ADC_CDR[3];
  adcResult5 = ADC->ADC_CDR[2];
  adcResult6 = ADC->ADC_CDR[1];
  adcResult7 = ADC->ADC_CDR[0];

  while (!(ADC->ADC_ISR & ADC_ISR_EOC10));
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
    input[0 + sample_counter] = adcResult0 * 0.00080586 - offset;
    input[1 + sample_counter] = adcResult1 * 0.00080586 - offset;
    input[2 + sample_counter] = adcResult2 * 0.00080586 - offset;
    input[3 + sample_counter] = adcResult3 * 0.00080586 - offset;
    input[4 + sample_counter] = adcResult4 * 0.00080586 - offset;
    input[5 + sample_counter] = adcResult5 * 0.00080586 - offset;
    input[6 + sample_counter] = adcResult6 * 0.00080586 - offset;
    input[7 + sample_counter] = adcResult7 * 0.00080586 - offset;

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

    sample_counter += numberOfInputs;

  }
  else if (sample_counter < sampleCount)
  {
    alternate = 1;
  }


  if (dac_counter < outputSize && alternateDAC)
  {

    dacc_set_channel_selection(DACC_INTERFACE, 1);       //select DAC channel 1
    dacc_write_conversion_data(DACC_INTERFACE, compressedOutput[dac_counter]);//write on DAC
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
    //alternateDAC = 1;
  }

  digitalWrite(3, HIGH);
  digitalWrite(4, HIGH);
  //delay(0.003);
}

int directionalityAngle(volatile int x)
{
  //  if (x >= 0 && x <= 106 ) return 0;
  //  else if (x >= 107 && x <= 213 ) return 1;
  //  else if (x >= 214 && x <= 320 ) return 2;
  //  else if (x >= 321 && x <= 427 ) return 3;
  //  else if (x >= 428 && x <= 534 ) return 4;
  //  else if (x >= 535 && x <= 641 ) return 5;
  //  else if (x >= 642 && x <= 748 ) return 6;
  //  else if (x >= 749 && x <= 855 ) return 7;
  //  else if (x >= 856 && x <= 962 ) return 8;
  //  else if (x >= 963 && x <= 1069 ) return 9;
  //  else if (x >= 1070 && x <= 1176 ) return 10;
  //  else if (x >= 1177 && x <= 1283 ) return 11;
  //  else if (x >= 1284 && x <= 1390 ) return 12;
  //  else if (x >= 1391 && x <= 1497 ) return 13;
  //  else if (x >= 1498 && x <= 1604 ) return 14;
  //  else if (x >= 1605 && x <= 1711 ) return 15;
  //  else if (x >= 1712 && x <= 1818 ) return 16;
  //  else if (x >= 1819 && x <= 1925 ) return 17;
  //  else return 18;

//  if (x >= 0 && x < 505) return 0;
//  else if (x >= 505 && x < 1515) return 90;
//  else return 180;

  if (x>=0 && x< 336) return 0;
  else if (x>=336 && x< 840) return 60;
  else if (x>=840 && x< 1176) return 90;
  else if (x>=1176 && x< 1680) return 120;
  else return 180;

}

void directionality90(double result[outputSize], double inputVector[sampleCount])
{
  // Adding the microphone signals together
  for (int idx1 = 0; idx1 < outputSize; idx1++) {
    double temp = 0;
    for (int idx0 = 0; idx0 < numberOfInputs; idx0++) {
      temp += inputVector[idx0 + numberOfInputs * idx1];
    }
    result[idx1] = temp / 4; // Divide by 4 to create the correct amplitude
  }
}

void directionality0(double result[outputSize], double inputVector[sampleCount])
{
  for (int i = 0; i < outputSize; i++)
  {
    mic1[i] = 0;
    mic2[i] = 0;
    mic3[i] = 0;
    mic4[i] = 0;
  }
  for (int i = 0; i < outputSize; i++)
  {
    mic1[i] = inputVector[i * 8] + inputVector[i * 8 + 1];

    if (i >= shift0Deg[1])
    {
      mic2[i] = inputVector[i * 8 + 2] + inputVector[i * 8 + 3];
    }
    else if (i >= shift0Deg[2])
    {
      mic2[i] = inputVector[i * 8 + 2] + inputVector[i * 8 + 3];
      mic3[i] = inputVector[i * 8 + 4] + inputVector[i * 8 + 5];
    }
    else if (i >= shift0Deg[3])
    {
      mic2[i] = inputVector[i * 8 + 2] + inputVector[i * 8 + 3];
      mic3[i] = inputVector[i * 8 + 4] + inputVector[i * 8 + 5];
      mic4[i] = inputVector[i * 8 + 6] + inputVector[i * 8 + 7];
    }
    result[i] = mic1[i] + mic2[i] + mic3[i] + mic4[i];
    result[i] = result[i] / 4;
  }
}

void directionality180(double result[outputSize], double inputVector[sampleCount])
{
  for (int i = 0; i < outputSize; i++)
  {
    mic1[i] = 0;
    mic2[i] = 0;
    mic3[i] = 0;
    mic4[i] = 0;
  }
  for (int i = 0; i < outputSize; i++)
  {
    mic4[i] = inputVector[i * 8 + 6] + inputVector[i * 8 + 7];

    if (i >= shift180Deg[1])
    {
      mic3[i] = inputVector[i * 8 + 4] + inputVector[i * 8 + 5];

    }
    else if (i >= shift180Deg[2])
    {
      mic3[i] = inputVector[i * 8 + 4] + inputVector[i * 8 + 5];
      mic2[i] = inputVector[i * 8 + 2] + inputVector[i * 8 + 3];
    }
    else if (i >= shift180Deg[3])
    {
      mic3[i] = inputVector[i * 8 + 4] + inputVector[i * 8 + 5];
      mic2[i] = inputVector[i * 8 + 2] + inputVector[i * 8 + 3];
      mic1[i] = inputVector[i * 8] + inputVector[i * 8 + 1];
    }
    result[i] = mic1[i] + mic2[i] + mic3[i] + mic4[i];
    result[i] = result[i] / 4;
  }
}

void directionality60(double result[outputSize], double inputVector[sampleCount])
{
  for (int i = 0; i < outputSize; i++)
  {
    mic1[i] = 0;
    mic2[i] = 0;
    mic3[i] = 0;
    mic4[i] = 0;
  }
  for (int i = 0; i < outputSize; i++)
  {
    mic1[i] = inputVector[i * 8] + inputVector[i * 8 + 1];

    if (i >= shift60Deg[1])
    {
      mic2[i] = inputVector[i * 8 + 2] + inputVector[i * 8 + 3];
    }
    else if (i >= shift60Deg[2])
    {
      mic2[i] = inputVector[i * 8 + 2] + inputVector[i * 8 + 3];
      mic3[i] = inputVector[i * 8 + 4] + inputVector[i * 8 + 5];
    }
    else if (i >= shift60Deg[3])
    {
      mic2[i] = inputVector[i * 8 + 2] + inputVector[i * 8 + 3];
      mic3[i] = inputVector[i * 8 + 4] + inputVector[i * 8 + 5];
      mic4[i] = inputVector[i * 8 + 6] + inputVector[i * 8 + 7];
    }
    result[i] = mic1[i] + mic2[i] + mic3[i] + mic4[i];
    result[i] = result[i] / 4;
  }
}

void directionality120(double result[outputSize], double inputVector[sampleCount])
{
  for (int i = 0; i < outputSize; i++)
  {
    mic1[i] = 0;
    mic2[i] = 0;
    mic3[i] = 0;
    mic4[i] = 0;
  }
  for (int i = 0; i < outputSize; i++)
  {
    mic4[i] = inputVector[i * 8 + 6] + inputVector[i * 8 + 7];

    if (i >= shift120Deg[1])
    {
      mic3[i] = inputVector[i * 8 + 4] + inputVector[i * 8 + 5];

    }
    else if (i >= shift120Deg[2])
    {
      mic3[i] = inputVector[i * 8 + 4] + inputVector[i * 8 + 5];
      mic2[i] = inputVector[i * 8 + 2] + inputVector[i * 8 + 3];
    }
    else if (i >= shift120Deg[3])
    {
      mic3[i] = inputVector[i * 8 + 4] + inputVector[i * 8 + 5];
      mic2[i] = inputVector[i * 8 + 2] + inputVector[i * 8 + 3];
      mic1[i] = inputVector[i * 8] + inputVector[i * 8 + 1];
    }
    result[i] = mic1[i] + mic2[i] + mic3[i] + mic4[i];
    result[i] = result[i] / 4;
  }
}

