#include <PID_v1.h>
#include <PID_AutoTune_v0.h>

IntervalTimer outputDriveTimer;
IntervalTimer tempMeasTimer;
IntervalTimer logTimer;

const int logInterval = 10000; // log every 10 seconds
long lastLogTime = 0;
int sensorPinH = A9;
int sensorPinL = A0;
int RelayPin = A2;

double Setpoint = 49;
double Input;
double Output = 400; //400/500 80% ontime for relay
volatile long onTime = 0;
volatile long heatTime = 60*1000*200; // how long to heat
volatile long coolTime = 60*1000*500; // 1000ms/s*60s/min*100 min
 
// pid tuning parameters NOT NEEDED
double Kp = 0;
double Ki = 0;
double Kd = 0;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
int WindowSize = 500; 
long windowStartTime;
// ************************************************
// Auto Tune Variables and constants
// ************************************************
byte ATuneModeRemember=2;
 
double aTuneStep=200;
double aTuneNoise=1;
unsigned int aTuneLookBack=1000;
 
boolean tuning = false;
boolean autotune = true;
 
PID_ATune aTune(&Input, &Output);

void TempToInput(){
  float temp = readTemp(sensorPinH,sensorPinL);
  noInterrupts();
  Input = temp;
  interrupts();
}

void DoControl()
{
  //lolololol nothing to control
}
void Run()
{


   while(true)
   {
      Serial.println("#S|LOGFILE1|[heating,0,0,0,0,0,0]#");
      noInterrupts();
      onTime = Output;
      interrupts();
      delay(heatTime);
      Serial.println("#S|LOGFILE1|[cooling,0,0,0,0,0,0]#");
      noInterrupts();
      onTime = 0;
      interrupts();
      delay(coolTime);

   }
}

void logtoserial()
{
        // periodically log to serial port in csv format
      if (millis() - lastLogTime > logInterval)  
      {
        Serial.print("#S|LOGFILE1|[");
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
        Serial.print(onTime);
        Serial.println("]#");
        lastLogTime = millis();
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
  logTimer.begin(logtoserial,10000);
  delay(3000);
  Serial.print("#S|LOGFILE1|[");
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
  Serial.println("]#");
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
