#include <XPT2046_Touchscreen.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <font_Arial.h>
#include <font_ArialBold.h>
#include <ILI9341_t3.h>
#include "SPI.h"

//*************************************************************************************
// screen usage variables
//*************************************************************************************
int Y_min = 0;
int Y_max = 4096;
int X_min = 0;
int X_max = 4096; 
elapsedMillis screenUpdate;
elapsedMillis buttonmillis;
int button_timeout = 50; //millis
#define TFT_DC  9
#define TFT_CS 10
#define CS_PIN  8
// MOSI=11, MISO=12, SCK=13
//XPT2046_Touchscreen ts(CS_PIN);
#define TIRQ_PIN  2
//XPT2046_Touchscreen ts(CS_PIN);  // Param 2 - NULL - No interrupts
//XPT2046_Touchscreen ts(CS_PIN, 255);  // Param 2 - 255 - No interrupts
XPT2046_Touchscreen ts(CS_PIN, TIRQ_PIN);  // Param 2 - Touch IRQ Pin - interrupt enabled polling
// Use hardware SPI (on Uno, #13, #12, #11) and the above for CS/DC
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);

//*************************************************************************************
// PID control and temp sense variable
//*************************************************************************************
IntervalTimer outputDriveTimer;
IntervalTimer tempMeasTimer;
const int logInterval = 10000; // log every 10 seconds
elapsedMillis lastLogTime;
elapsedMillis DoControltimer;
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


//*************************************************************************************
// screen usage Functions
//*************************************************************************************
float CtoF (float C_temp)
{
  return C_temp*9/5+32.0;
}

int MapX( int touch_input)
{
  int result = touch_input*320/(X_max-X_min);
  return result;
}

int MapY( int touch_input)
{
  int result = touch_input*240/(Y_max-Y_min);
  return result;
}
void drawText()
{
  tft.setFont(Arial_24);
  tft.setCursor(0,58);
  tft.println("Current:");

  tft.setCursor(210,58);
  tft.println("Set to:");
}
void drawButtons()
{
  
  tft.fillRoundRect(120,100,80,40,15,ILI9341_BLACK);  // middle rect
  tft.drawRoundRect(120,100,80,40,15,ILI9341_WHITE);  // middle rect

  tft.fillRoundRect(120,50,80,40,15,ILI9341_BLACK);
  tft.drawRoundRect(120,50,80,40,15,ILI9341_WHITE);   // top rect
  tft.drawLine(145,80,160,60,ILI9341_WHITE);            // top triangle
  tft.drawLine(160,60,175,80,ILI9341_WHITE);

  tft.fillRoundRect(120,150,80,40,15,ILI9341_BLACK);
  tft.drawRoundRect(120,150,80,40,15,ILI9341_WHITE);  // bottom rect
  tft.drawLine(145,160,160,180,ILI9341_WHITE);            // bottom triangle
  tft.drawLine(160,180,175,160,ILI9341_WHITE);
}
void drawBaseScreen()
{
  tft.fillScreen(ILI9341_BLACK);
  drawButtons();
  drawText();
}
void updateTemps(float curr, float setp)
{
  tft.fillRect(10,108,100,20, ILI9341_BLACK);
  tft.setFont(Arial_20);
  tft.setCursor(10,108);
  tft.print(curr,1);
  tft.setFont(Arial_8);
  tft.print(" o");
  tft.setFont(Arial_20);
  tft.print("C");

  tft.fillRect(10,158,100,20, ILI9341_BLACK);
  tft.setFont(Arial_20);
  tft.setCursor(10,158);
  tft.print(CtoF(curr),1);
  tft.setFont(Arial_8);
  tft.print(" o");
  tft.setFont(Arial_20);
  tft.print("F");

  tft.fillRect(210,108,100,20, ILI9341_BLACK);
  tft.setFont(Arial_20);
  tft.setCursor(210,108);
  tft.print(setp,1);
  tft.setFont(Arial_8);
  tft.print(" o");
  tft.setFont(Arial_20);
  tft.print("C");

  tft.fillRect(210,158,100,20, ILI9341_BLACK);
  tft.setFont(Arial_20);
  tft.setCursor(210,158);
  tft.print(CtoF(setp),1);
  tft.setFont(Arial_8);
  tft.print(" o");
  tft.setFont(Arial_20);
  tft.print("F");
}

//*************************************************************************************
//  PID control and temp sense Functions
//*************************************************************************************

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
  if (autotune && (abs(Input - Setpoint) < 0.5))  // Should be at steady-state
  {
    autotune = false;
    StartAutoTune();
  }
  if (DoControltimer>100){
    DoControl();  
    DoControltimer = 0;
  }
  
  // periodically log to serial port in csv format
  if (lastLogTime > logInterval)  
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
    lastLogTime = 0;
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
 //*****************************************************************************************************
 //  setup time
 //*****************************************************************************************************

void setup() {
  //setup screen variables**************************************//
  tft.begin();                                                  //
  ts.begin();                                                   //
  tft.setRotation(1);                                           //
  tft.fillScreen(ILI9341_BLACK);                                //
  tft.setTextColor(ILI9341_WHITE);                              //
  tft.setFont(Arial_24);                                        //
  tft.setCursor(0,0);                                           //
  tft.println("Waiting for Arduino Serial Monitor...");         //
  tft.fillRect(0,0,24,24, ILI9341_BLACK);                       //
  //************************************************************//

  //************************************************************//
  analogReference(INTERNAL);                                    //
  analogReadResolution(14);                                     //
  analogReadAveraging(32);                                      //
  pinMode(RelayPin, OUTPUT);                                    //
  digitalWrite(RelayPin, LOW);                                  //
  myPID.SetTunings(Kp,Ki,Kd);                                   //
  myPID.SetSampleTime(1000);                                    //
  myPID.SetOutputLimits(0, WindowSize);                         //
  if (!autotune){                                               //
    myPID.SetMode(AUTOMATIC);                                   //
  }                                                             //
  outputDriveTimer.priority(100);                               //
  outputDriveTimer.begin(DriveOutput, 15000);                   //
  tempMeasTimer.begin( TempToInput, 14001 );                    //
  //************************************************************//
  
  Serial.begin(38400);
  //while (!Serial | lastLogTime>8000) ; // wait for Arduino Serial Monitor
  Serial.println("ILI9341 Test!"); 

  // read diagnostics (optional but can help debug problems)
  uint8_t x = tft.readcommand8(ILI9341_RDMODE);
  Serial.print("Display Power Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDMADCTL);
  Serial.print("MADCTL Mode: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDPIXFMT);
  Serial.print("Pixel Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDIMGFMT);
  Serial.print("Image Format: 0x"); Serial.println(x, HEX);
  x = tft.readcommand8(ILI9341_RDSELFDIAG);
  Serial.print("Self Diagnostic: 0x"); Serial.println(x, HEX); 
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

  drawBaseScreen();
  updateTemps(Input,Setpoint);


  
}


void loop() {
  // put your main code here, to run repeatedly:
  Run(); 
  if (ts.touched()) {
    TS_Point p = ts.getPoint();
    Serial.print("Pressure = ");
    Serial.print(p.z);
    Serial.print(", x = ");
    Serial.print(p.x);
    Serial.print(", y = ");
    Serial.print(p.y);
    delay(10);
    Serial.println();
    //tft.fillScreen(ILI9341_BLACK);
    //tft.fillRect(MapX(p.x),MapY(p.y),5,5,ILI9341_WHITE);
    float pointX = MapX(p.x);
    float pointY = MapY(p.y);
    if ( pointX < 200 && pointX >120 && buttonmillis>button_timeout)
    {
      if(pointY <90 ){
        //top button touched
        tft.fillRoundRect(125,55,70,30,15,ILI9341_BLUE);   // top rect
        Setpoint = Setpoint+0.1; //increment setpoint
        
      }
      else if (pointY > 150){
        //bottom button touched
        tft.fillRoundRect(125,155,70,30,15,ILI9341_BLUE); 
        Setpoint = Setpoint-0.1; //decrement setpoint

      }
      else{
        //middlebutton
        tft.fillRoundRect(125,105,70,30,15,ILI9341_BLUE);
      }
      buttonmillis = 0;      
    }
    drawButtons();
    updateTemps(Input,Setpoint);
  }
  if (screenUpdate > (1000)){
    updateTemps(Input,Setpoint);
    screenUpdate = 0;
  }
  
}
