#include <XPT2046_Touchscreen.h>

#include <font_Arial.h>
#include <font_ArialBold.h>
#include <ILI9341_t3.h>
#include "SPI.h"

int Y_min = 0;
int Y_max = 4096;
int X_min = 0;
int X_max = 4096; 

float var_1 = 0;
float var_2 = 0;

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

float CtoF (float C_temp)
{
  return C_temp*9/5+32.0;
}

void setup() {
  tft.begin();
  ts.begin();
  tft.setRotation(1);
  tft.fillScreen(ILI9341_BLACK);
  tft.setTextColor(ILI9341_WHITE);
  tft.setFont(Arial_24);
  tft.setCursor(0,0);
  tft.println("Waiting for Arduino Serial Monitor...");
  tft.fillRect(0,0,24,24, ILI9341_BLACK);

  Serial.begin(38400);
  while (!Serial) ; // wait for Arduino Serial Monitor
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

  drawBaseScreen();
  updateTemps(var_1,var_2);
  delay(1000);
  //updateTemps(17.7,12.25);

  
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

void loop() {
  // put your main code here, to run repeatedly:
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
        var_2 = var_2+0.1; //increment setpoint
        
      }
      else if (pointY > 150){
        //bottom button touched
        tft.fillRoundRect(125,155,70,30,15,ILI9341_BLUE); 
        var_2 = var_2-0.1; //decrement setpoint

      }
      else{
        //middlebutton
        tft.fillRoundRect(125,105,70,30,15,ILI9341_BLUE);
      }
      buttonmillis = 0;      
    }
    drawButtons();
    updateTemps(var_1,var_2);
  }
  if (screenUpdate > (1000)){
    updateTemps(var_1,var_2);
    screenUpdate = 0;
  }
  
}
