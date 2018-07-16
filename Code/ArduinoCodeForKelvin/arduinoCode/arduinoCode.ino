const int analogInputA0 = A0;
char mode;

void setup() {
   Serial.begin(115200); // Set up serial communciation at a baud rate of 115200 bits/sec
  //ADCSRA &= ~(bit (ADPS0) | bit (ADPS1) | bit (ADPS2)); // clear all prescaler bits
 
  Serial.println('a') ;
  char a = 'b';
  while (a !='a')
  { 
    a=Serial.read()  ; 
  }
  //ADCSRA |= bit (ADPS2) ; // set a prescaler of 16. This sets the ADC clock frequency to 1000kHz
  REG_ADC_MR = (REG_ADC_MR & 0xFFF0FFFF) | 0x00020000; //http://frenki.net/2013/10/fast-analogread-with-arduino-due/
  REG_ADC_MR = (REG_ADC_MR & 0xFFFFFF0F) | 0x00000080; //enable FREERUN mode
  
  analogReadResolution(12);
  analogWriteResolution(12);
}

void loop() {
  // put your main code here, to run repeatedly:
  // read the input on analog pin 0:
  if (Serial.available()>0 ) // make sure that there is data being serially transmitted
  {
    mode=Serial.read(); //read the byte
    if (mode == 'R')    
    {
      unsigned int micInput = analogRead(analogInputA0);
      float micVoltage = micInput * (3.3 / 4095.0);
      // print out the value you read:
      Serial.println(micInput);
    }
    //delay(1) ;
 //analogWrite(DAC0,micInput);
  }
}
