#include<PID_v1.h>

int sensorPinH = A9;
int sensorPinL = A0;
  // Pin 13: Arduino has an LED connected on pin 13
  // Pin 11: Teensy 2.0 has the LED on pin 11
  // Pin  6: Teensy++ 2.0 has the LED on pin 6
  // Pin 13: Teensy 3.0 has the LED on pin 13
int sensorValueH = 0;  // variable to store the value coming from the sensor
int sensorValueL = 0;  // variable to store the value coming from the sensor
float runningAvg = 0;
float Volt = 0;
float V0 = 1645; // in Millivolts
float Res1 = 99.8;  // (top right) resistance of upper resistor in other leg resistor set
float Res2 = 100.3; // (bottom right) resistance of lower resistor in other leg resistor set
float Res3 = 100.2; // (top left) resistance of resistor in series with probe
float ResP = 0; // (bottom left) resistance of the probe
double temp = 0;
double coeffA = .00385;
double coeffB= -0.0000005775;
float Res0 = 100;


void setup() {
  Serial.begin(38400);
  analogReference(INTERNAL);
  analogReadResolution(14);
  analogReadAveraging(32);
}

void loop() {
  // put your main code here, to run repeatedly:
  // read the value from the sensor:
  sensorValueH = analogRead(sensorPinH);    
  sensorValueL = analogRead(sensorPinL);    
  runningAvg = (10.0*runningAvg + (sensorValueH-sensorValueL)/1.0)/11.0;
  Volt = (runningAvg/16383.0)*1200;
  ResP = (Res3*(Res1*(V0-Volt)-Res2*Volt))/(Res1*Volt+Res2*(Volt+V0));
  temp = (-coeffA*sqrt(Res0)+sqrt(coeffA*coeffA*Res0-4*coeffB*Res0+4*coeffB*ResP))/(2*coeffB*sqrt(Res0));
  delay(100);
  Serial.print(' ');
  Serial.print((int)sensorValueH);
  Serial.print(' ');
  Serial.print((int)sensorValueL);
  Serial.print(' ');
  Serial.print(Volt);
  Serial.print(' ');
  Serial.print(ResP);
  Serial.print(' ');
  Serial.print(temp);
  Serial.print(' ');
  Serial.println(runningAvg);
}
