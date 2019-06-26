#include <XPT2046_Touchscreen.h>
#include <PID_v1.h>
#include <PID_AutoTune_v0.h>
#include <font_Arial.h>
#include <font_ArialBold.h>
#include <ILI9341_t3.h>
#include "SPI.h"
#include <Adafruit_MAX31865.h>

//*************************************************************************************
// screen usage variables
//*************************************************************************************
#define TFT_DC  9
#define TFT_CS 10
#define CS_PIN  8
#define TIRQ_PIN  2
XPT2046_Touchscreen ts(CS_PIN, TIRQ_PIN);  
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);
int Y_min = 0;
int Y_max = 4096;
int X_min = 0;
int X_max = 4096; 
elapsedMillis TFTupdateTimer;
int TFTupdatePeriod = 1077;    //millis
elapsedMillis buttonmillis;                 // touchscreen debouncing
int button_timeout = 30;       //millis     // touchscreen debouncing
int totalTime = 0;

//*************************************************************************************
// PID control and temp sense variable
//*************************************************************************************
Adafruit_MAX31865 maxi = Adafruit_MAX31865(19);
#define RREF      430.0
#define RNOMINAL  100.0
elapsedMillis TempUpdateTimer;
int TempUpdatePeriod = 1013;     //millis

double Setpoint = 60.0;
double Input = 21;
double Output = 0;
// pid tuning parameters
double Kp = 10;   //proportionality terms
double Ki = 0.002; //.05   ; //1.7
double Kd = 0;
double Ks = 0; //0.5;
double Gp = 0;    // gain terms
double Gi = 0;
double Gd = 0; 
double Gs = 0;
double Err_n = 0; // "new" error term
double Err_l = 0; // previous error terms
elapsedMillis PIDUpdateTimer;
int PIDUpdatePeriod = 1053;        //millis
// 61.10, 119.83, 0.08 

int RelayPin = A6;
elapsedMillis OutputUpdateTimer;
int OutputUpdatePeriod = 1000;     //millis


elapsedMillis SerialUpdateTimer;
int SerialUpdatePeriod = 10000;     //millis

//*************************************************************************************
// screen usage Functions
//*************************************************************************************
float CtoF (float C_temp)
{return C_temp*9/5+32.0;
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
void drawTFT_Text()
{
  tft.setFont(Arial_24);
  tft.setCursor(0,58);
  tft.println("Current:");
  tft.setCursor(210,58);
  tft.println("Set to:");
}
void drawTFT_Buttons()
{
  
  tft.fillRoundRect(120,100,80,40,15,ILI9341_BLACK);   // middle rect
  tft.drawRoundRect(120,100,80,40,15,ILI9341_WHITE);   // middle rect

  tft.fillRoundRect(120,50,80,40,15,ILI9341_BLACK);
  tft.drawRoundRect(120,50,80,40,15,ILI9341_WHITE);    // top rect
  tft.drawLine(145,80,160,60,ILI9341_WHITE);           // top triangle
  tft.drawLine(160,60,175,80,ILI9341_WHITE);

  tft.fillRoundRect(120,150,80,40,15,ILI9341_BLACK);
  tft.drawRoundRect(120,150,80,40,15,ILI9341_WHITE);   // bottom rect
  tft.drawLine(145,160,160,180,ILI9341_WHITE);         // bottom triangle
  tft.drawLine(160,180,175,160,ILI9341_WHITE);
}
void drawTFT_Base()
{
  tft.fillScreen(ILI9341_BLACK);
  drawTFT_Buttons();
  drawTFT_Text();
}

void drawTFT_Temps(float curr, float setp)
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
  uint16_t rtd = maxi.readRTD();
  Input = maxi.temperature(RNOMINAL, RREF);
}

void UpdatePID(){
  Err_n = Setpoint - Input;
  Gs = Ks*(Setpoint-25);
  Gp = min(Err_n*Kp,255);
  Gi += Err_n*Ki*PIDUpdatePeriod/1000;  
  Gi = min(Gi, 255-Gp-Gs);
  Gi = max(Gi, -255+Gp);
  Gd = (Err_n-Err_l)/(PIDUpdatePeriod/1000)*Kd;
  
  Output = (11.0/16)*Output + (5.0/16)*(Gp+Gi+Gd+Gs); // (Gp+Gi+Gd+Gs);
  Err_l = Err_n;
}

void UpdateOutput(){
  // this might become much more complicated, thats why it's its own function/loop
  // TODO: what is A6 write resolution? 255?
  analogWrite(RelayPin, min(Output, 255));
}

void UpdateSerial(){
  totalTime += SerialUpdateTimer/1000; 
  Serial.print(totalTime);
  Serial.print(",");
  Serial.print(Setpoint);
  Serial.print(",");
  Serial.print(Input);
  Serial.print(",");
  Serial.print(Err_n);
  Serial.print(",");
  Serial.print(Err_l);
  Serial.print(",");
  Serial.print(Gp);
  Serial.print(",");
  Serial.print(Gi);
  Serial.print(",");
  Serial.print(Gd);
  Serial.print(",");
  Serial.print(Gs);
  Serial.print(",");
  Serial.print(Output);
  Serial.println(", 0");
}


 //*****************************************************************************************************
 //  setup time
 //*****************************************************************************************************

void setup() {
  // setup serial stuff
  Serial.begin(115200);

  // setup temp measuring stuff
  maxi.begin(MAX31865_3WIRE);  // set to 2WIRE or 4WIRE as necessary
  
  //setup touchscreen variables*********************************//
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

  //setup driving stuff
  pinMode(RelayPin, OUTPUT);                                    //
  
  //tft.println("WALUIGI WAAAAAAAA");                             //
  //if (!autotune){                                               //
  //  myPID.SetMode(AUTOMATIC);                                   //
  //}                                                             //
  //outputDriveTimer.priority(100);                               //
  //outputDriveTimer.begin(DriveOutput, 15000);                   //
  //tempMeasTimer.begin( TempToInput, 14001 );                    //
  //************************************************************//
  
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
  Serial.print("Err_n");
  Serial.print(",");
  Serial.print("Err_l");
  Serial.print(",");
  Serial.print("Kp");
  Serial.print(",");
  Serial.print("Ki");
  Serial.print(",");
  Serial.print("Kd");
  Serial.print(",");
  Serial.print("Ks");
  Serial.print(",");
  Serial.print("Output");
  Serial.println(", 0"); 

  
  drawTFT_Base();
  drawTFT_Temps(Input,Setpoint);  
}


void loop() {

  // update the TFT
  if (TFTupdateTimer > TFTupdatePeriod){
    //drawTFT_Base();
    drawTFT_Temps(Input,Setpoint);
    TFTupdateTimer = 0;
  }

  // update the touchscreen
  if (ts.touched() && (buttonmillis>button_timeout)) {
    TS_Point p = ts.getPoint();
    float pointX = MapX(p.x);
    float pointY = MapY(p.y);
    if (pointX < 200 && pointX >120)
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
    drawTFT_Buttons();
    drawTFT_Temps(Input,Setpoint);
  }

  // update the Temp data
  if (TempUpdateTimer > TempUpdatePeriod){
    TempToInput();
    TempUpdateTimer = 0;
  }

  // Update the PID stuff
    if (PIDUpdateTimer > PIDUpdatePeriod){
    UpdatePID();
    PIDUpdateTimer = 0;
  }

  // Update the output (likely trivial)
    if (OutputUpdateTimer > OutputUpdatePeriod){
    UpdateOutput();
    OutputUpdateTimer = 0;
  }
  
  // update Serial Stuff
    if (SerialUpdateTimer > SerialUpdatePeriod){
    UpdateSerial();
    SerialUpdateTimer = 0;
  }
  
}
