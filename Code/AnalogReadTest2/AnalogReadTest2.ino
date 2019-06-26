
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

IntervalTimer outputDriveTimer;
IntervalTimer tempMeasTimer;

const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;
int sensorPinH = A9;
int sensorPinL = A0;
int RelayPin = A2;

double Setpoint = 82.2;
double Input;
double Output;
 
volatile long onTime = 0;
 
// pid tuning parameters
double Kp = 70;
double Ki = 17.2;
double Kd = 0;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 500; 
long windowStartTime;
// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;
 
double aTuneStep=400;
double aTuneNoise=1.5;
unsigned int aTuneLookBack=150;
// 61.10, 119.83, 0.08 
 
boolean tuning = false;
boolean autotune = 0;
 
PID_ATune aTune(&Input, &Output);

void TempToInput(){
  float temp = readTemp(sensorPinH,sensorPinL);
  noInterrupts();
  Input = temp;
  interrupts();
}

void DoControl()
{
  // Read the input:
  if (tuning) // run the auto-tuner
  {
     if (aTune.Runtime()) // returns 'true' when done
     {
       tuning = false;

       // Extract the auto-tune calculated parameters
       Kp = aTune.GetKp();
       Ki = aTune.GetKi();
       Kd = aTune.GetKd();

       // Re-tune the PID and revert to normal control mode
       myPID.SetTunings(Kp,Ki,Kd);
       myPID.SetMode(ATuneModeRemember);
     }
  }
  else // Execute control algorithm
  {
     myPID.Compute();
  }
  
  // Time Proportional relay state is updated regularly via timer interrupt.
  noInterrupts();
  onTime = Output; 
  interrupts();
}
void Run()
{


   while(true)
   {

      if (autotune && (abs(Input - Setpoint) < 0.5))  // Should be at steady-state
      {
        autotune = false;
        StartAutoTune();
      }

      DoControl();

      // periodically log to serial port in csv format
      if (millis() - lastLogTime > logInterval)  
      {
        Serial.print("0,");
        Serial.print(Setpoint);
        Serial.print(",");
        Serial.print(Input);
        Serial.print(",");
        Serial.print(Kp);
        Serial.print(",");
        Serial.print(Ki);
        Serial.print(",");
        Serial.print(Kd);
        Serial.print(",");
        Serial.print(tuning);
        Serial.print(",");
        Serial.print(Output);
        Serial.println(", 0");
        lastLogTime = millis();
      }

      delay(100);
   }
}
void DriveOutput()
{  
  long now = millis();
  // Set the output
  // "on time" is proportional to the PID output
  if(now - windowStartTime>WindowSize)
  { //time to shift the Relay Window
     windowStartTime += WindowSize;
  }
  if(onTime > (now - windowStartTime))
  {
     digitalWrite(RelayPin,HIGH);
  }
  else
  {
     digitalWrite(RelayPin,LOW);
  }
}
void StartAutoTune()
{
   // REmember the mode we were in
   ATuneModeRemember = myPID.GetMode();

   // set up the auto-tune parameters
   aTune.SetControlType(0);
   aTune.SetNoiseBand(aTuneNoise);
   aTune.SetOutputStep(aTuneStep);
   aTune.SetLookbackSec((int)aTuneLookBack);
   tuning = true;
}

float readTemp( int pinHigh, int pinLow){
  float temp = 0;
  float V0 = 1645; // in Millivolts
  float Res1 = 99.8;  // (top right) resistance of upper resistor in other leg resistor set
  float Res2 = 100.3; // (bottom right) resistance of lower resistor in other leg resistor set
  float Res3 = 100.2; // (top left) resistance of resistor in series with probe
  double coeffA = .00385;
  double coeffB= -0.0000005775;
  float Res0 = 100;
  
  int sensorValueH = analogRead(pinHigh);    
  int sensorValueL = analogRead(pinLow); 
  float Volt = ((sensorValueH-sensorValueL)/16383.0)*1200;
  float ResP = (Res3*(Res1*(V0-Volt)-Res2*Volt))/(Res1*Volt+Res2*(Volt+V0));
  temp = (-coeffA*sqrt(Res0)+sqrt(coeffA*coeffA*Res0-4*coeffB*Res0+4*coeffB*ResP))/(2*coeffB*sqrt(Res0));
  return temp;
}

void setup() {
  Serial.begin(9600);
  
  
  analogReference(INTERNAL);
  analogReadResolution(14);
  analogReadAveraging(32);
  pinMode(RelayPin, OUTPUT);
  digitalWrite(RelayPin, LOW);
  myPID.SetTunings(Kp,Ki,Kd); 
  myPID.SetSampleTime(1000);
  myPID.SetOutputLimits(0, WindowSize);
  if (!autotune){
    myPID.SetMode(AUTOMATIC);
  }  
  outputDriveTimer.priority(100);
  outputDriveTimer.begin(DriveOutput, 15000);
  tempMeasTimer.begin( TempToInput, 14001 );
  delay(3000);
  Serial.print("0,");
  Serial.print("Setpoint");
  Serial.print(",");
  Serial.print("Input");
  Serial.print(",");
  Serial.print("Kp");
  Serial.print(",");
  Serial.print("Ki");
  Serial.print(",");
  Serial.print("Kd");
  Serial.print(",");
  Serial.print("tuning");
  Serial.print(",");
  Serial.print("Output");
  Serial.println(", 0");
}

void loop() {
  // put your main code here, to run repeatedly:
  // read the value from the sensor:
  Run(); 

  //delay(100);
  //Serial.print(' ');
  //Serial.print((int)sensorValueH);
  //Serial.print(' ');
  //Serial.print((int)sensorValueL);
  //Serial.print(' ');
  //Serial.print(Volt);
  //Serial.print(' ');
  //Serial.print(ResP);
  //Serial.print(' ');
  //Serial.println(temp);
}
