void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  analogReadResolution(12);
  analogWriteResolution(12);
}

void loop() {
  // put your main code here, to run repeatedly:
  // read the input on analog pin 0:
  int micInput = analogRead(A0);
  float micVoltage = micInput * (3.3 / 4095.0);
  // print out the value you read:
  Serial.println(micVoltage);

  analogWrite(DAC0,micInput);
}
