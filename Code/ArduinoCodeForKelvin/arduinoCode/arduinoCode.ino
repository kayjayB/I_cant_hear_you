//const int analogInputA0 = A0;
//char mode;
//
//void setup() {
//   Serial.begin(115200); // Set up serial communciation at a baud rate of 115200 bits/sec
//  //ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2)); // clear all prescaler bits
// 
//  Serial.println('a') ;
//  char a = 'b';
//  while (a !='a')
//  { 
//    a=Serial.read()  ; 
//  }
//  //ADCSRA |= bit (ADPS2) ; // set a prescaler of 16. This sets the ADC clock frequency to 1000kHz
//  REG_ADC_MR = (REG_ADC_MR & 0xFFF0FFFF) | 0x00020000; //http://frenki.net/2013/10/fast-analogread-with-arduino-due/
//  REG_ADC_MR = (REG_ADC_MR & 0xFFFFFF0F) | 0x00000080; //enable FREERUN mode
//  
//  analogReadResolution(10);
//  analogWriteResolution(12);
//}
//
//void loop() {
//  // put your main code here, to run repeatedly:
//  // read the input on analog pin 0:
//  if (Serial.available()>0 ) // make sure that there is data being serially transmitted
//  {
//    mode=Serial.read(); //read the byte
//    if (mode == 'R')    
//    {
//      unsigned int micInput = analogRead(analogInputA0);
//      float micVoltage = micInput * (3.3 / 4095.0);
//      // print out the value you read:
//      Serial.println(micInput);
//    }
//    //delay(1) ;
// //analogWrite(DAC0,micInput);
//  }
//}
//********************************************************************
#include <SD.h>
#include <SPI.h>
#include <Audio.h>

unsigned long start_time;
unsigned long stop_time;
unsigned int values[50];
int sizeOfBuffer = sizeof(values);

void setup() {        
  Serial.begin(9600);  
  ADC->ADC_MR |= 0x80;  //set free running mode on ADC
  ADC->ADC_CHER = 0x80; //enable ADC on pin A0

//  pmc_enable_periph_clk (ID_ADC);
//
//  adc_init (ADC, SystemCoreClock, ADC_FREQ_MIN, ADC_STARTUP_FAST);
//
//  ADC->ADC_CHER = 0xFF; //enable ADC on pin A0-A7 // for A0 to A8 use 0x4FF
//
//  ADC->ADC_WPMR = 0x00;//Disables the write protect key, WPEN
//  ADC->ADC_MR = 0x00000000;//clear all the before setted characteristics of ADC 
//
//  PIOA->PIO_PDR |= PIO_PDR_P16; //Disable PIO Controller
//  ADC->ADC_MR = ADC_MR_PRESCAL(2); // set ADC prescale to 2
//  ADC->ADC_MR |= 0x80;  //set free running mode on ADC. Don't wait for triggers
//  ADC->ADC_CHER = 0xFF; //enable ADC on pin A0-A7 // for A0 to A8 use 0x4FF
//
//  ADC->ADC_MR |= ADC_MR_TRACKTIM(3); 
//  ADC->ADC_MR |= ADC_MR_STARTUP_SUT8; 
//  ADC->ADC_EMR = 0;
  
  analogWriteResolution(12);
  //analogWrite(DAC0,0);  // Enables DAC0
  analogWrite(DAC1,0);  // Enables DAC0
   DACC->DACC_MR = DACC_MR_TRGEN_DIS                    // Free running mode
                  | DACC_MR_USER_SEL_CHANNEL1          // select channel 1
                  //| DACC_MR_REFRESH (25)
                  //| DACC_MR_STARTUP_8
                  | DACC_MR_MAXS;
}

void loop() {
  unsigned int i;
    
  start_time = micros();
  for(i=0;i<sizeOfBuffer;i++){
    while((ADC->ADC_ISR & 0x80)==0); // wait for conversion
    values[i]=ADC->ADC_CDR[7]; //get values
//    analogWrite(DAC0,values[i]);
  }
  stop_time = micros();

//  Serial.print("Total time: ");
//  Serial.println(stop_time-start_time); 
//  Serial.print("Average time per conversion: ");
//  Serial.println((float)(stop_time-start_time)/1000);

//  Serial.println("Values: ");
//  for(i=0;i<50;i++) {
//    Serial.println(values[i]);
//  }


  
for(i=0;i<sizeOfBuffer;i++){
//  float temp=0;
//  for (int j=0; j<5; j++) {
//  //analogWriteResolution(12);
//  //analogWrite(DAC0,values[i]);
//    temp += values[i*5+j];
//  }
//  temp = temp/5;
  dacc_set_channel_selection(DACC_INTERFACE, 1);       //select DAC channel 0
  dacc_write_conversion_data(DACC_INTERFACE, values[i]);//write on DAC
  //analogWrite(DAC1,values[i]);
  //delayMicroseconds(1);
}
}

