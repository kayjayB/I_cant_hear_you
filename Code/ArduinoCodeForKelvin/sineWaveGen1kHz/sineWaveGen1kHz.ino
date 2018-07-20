int led = 31;

//1kHz
int sine[] = {2048,2305,2557,2802,3034,3251,3449,3625,3777,3901,3995,4059,4092,4092,4060,3996,3902,3778,3628,3452,3254,3037,2805,2560,2308,2051,1795,1542,1297,1065,847,649,473,321,197,102,37,4,4,35,99,193,316,466,642,839,1056,1288,1533,1785,2041};

// the setup routine runs once when you press reset:
void setup() {                
  // initialize the digital pin as an output.
  pinMode(led, OUTPUT);   
  //pinMode(DAC0, OUTPUT);   
}

// the loop routine runs over and over again forever:
void loop() {
  for(int i = 0; i<50;i++){
      if(sine[i]>4095) {
         sine[i]=4095;
         digitalWrite(led, LOW);
     }
    analogWriteResolution(12);
    analogWrite(DAC1, sine[i]);
    delayMicroseconds(14);
  }
}
