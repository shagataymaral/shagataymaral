//Board: Arduino nano ble
//Sensor: GY-AS7263
//QR: GM65-s
//SD: Waveshare micro sd
#include "AP_29demo.h"

#include <ArduinoBLE.h>

#include <Wire.h>
#include "Adafruit_AS726x.h"
#include <Arduino.h>                              
#include <wiring_private.h>
#include <SPI.h>
#include <SD.h>

//Red and green led
const int Red_pin = 2;
const int Green_pin = 3;

//create the object
#define TCAADDR 0x70
Adafruit_AS726x ams1;
Adafruit_AS726x ams2;
const int tca_ams1 = 0;
const int tca_ams2 = 1;

//buffer to hold raw values
uint16_t sensorValues[AS726x_NUM_CHANNELS];
uint16_t sensorValuesSpecimen[AS726x_NUM_CHANNELS];
uint16_t dry_sensorValues[AS726x_NUM_CHANNELS];
uint16_t dry_sensorValuesSpecimen[AS726x_NUM_CHANNELS];
uint16_t cf_v, cf_b, sf_v, sf_b;

String result;
int result_switch = 0;
const int buttonPin = 4; //D4
double margint;
uint8_t board_temp1;
uint8_t board_temp2;

//Settings
float stain_margin_percent=4;
float max_control_change_percent=4;
int16_t max_negative_stain_margin_percent=20;
uint16_t min_reading_threshold=40;

//Eink
/*
 * Busy 7
 * Rst 8
 * DC 9
 * CS 10
 * DIN 11
 * CLK 13
 */
//IO settings
const int BUSY_Pin = 7; 
const int RES_Pin = 8; 
const int DC_Pin = 9; 
//const int CS_Pin = 10; 
const int CS_Pin = 5; //D5
const int SCK_Pin = 13; 
const int SDI_Pin = 11; 

#define EPD_W21_MOSI_0  digitalWrite(SDI_Pin,LOW)
#define EPD_W21_MOSI_1  digitalWrite(SDI_Pin,HIGH) 

#define EPD_W21_CLK_0 digitalWrite(SCK_Pin,LOW)
#define EPD_W21_CLK_1 digitalWrite(SCK_Pin,HIGH)

#define EPD_W21_CS_0 digitalWrite(CS_Pin,LOW)
#define EPD_W21_CS_1 digitalWrite(CS_Pin,HIGH)

#define EPD_W21_DC_0  digitalWrite(DC_Pin,LOW)
#define EPD_W21_DC_1  digitalWrite(DC_Pin,HIGH)

#define EPD_W21_RST_0 digitalWrite(RES_Pin,LOW)
#define EPD_W21_RST_1 digitalWrite(RES_Pin,HIGH)

#define isEPD_W21_BUSY digitalRead(BUSY_Pin)

//SIZE: 250*122
#define MONOMSB_MODE 1
#define MONOLSB_MODE 2 
#define RED_MODE     3

#define MAX_LINE_BYTES 25 // =200/8
#define MAX_COLUMN_BYTES  200

#define ALLSCREEN_GRAGHBYTES 5000

//Function
void driver_delay_us(unsigned int xus);
void driver_delay_xms(unsigned long xms);
void DELAY_S(unsigned int delaytime);     
void SPI_Delay(unsigned char xrate);
void SPI_Write(unsigned char value);
void Epaper_Write_Command(unsigned char command);
void Epaper_Write_Data(unsigned char command);

//EPD
//void Epaper_READBUSY(void);
//void SPI_Write(u8 TxData);
//void Epaper_Write_Command(u8 cmd);
//void Epaper_Write_Data(u8 data);

void EPD_HW_Init(void); //Electronic paper initialization
void EPD_Part_Init(void); //Local refresh initialization

void EPD_Part_Update(void); 
void EPD_Update(void);

void EPD_WhiteScreen_Black(void);
void EPD_WhiteScreen_White(void);
void EPD_DeepSleep(void);

//Display 
void EPD_WhiteScreen_ALL(const unsigned char * datas);
void EPD_SetRAMValue_BaseMap(const unsigned char * datas);
void EPD_Dis_Part(unsigned int x_start,unsigned int y_start,const unsigned char * datas,unsigned int PART_COLUMN,unsigned int PART_LINE);

//QR reader
/*
 * Tx black p2
 * Rx yellow p1
 */
int incomingByte = 0; // for incoming serial data
String readString = "";
String decodeString = "";
#define timer 0.1 // 0.2 = 12s , running time of the while loop in minutes
boolean QR_received = false;
//boolean stringComplete = false;

//SD card
/*
 * Miso p12
 * Mosi p11
 * Clk p13
 * Cs p10

File myFile;
const int SdSelect = 10;
String filename = "test.txt";
 */
void tcaselect(uint8_t i) {
  if (i > 7) return;
 
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
  pinMode(BUSY_Pin, INPUT); 
  pinMode(RES_Pin, OUTPUT);
  pinMode(DC_Pin, OUTPUT);        
  pinMode(SCK_Pin, OUTPUT);    
  pinMode(SDI_Pin, OUTPUT);

  pinMode(CS_Pin, OUTPUT);
  //pinMode(SdSelect, OUTPUT);
  
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(Red_pin, OUTPUT);
  pinMode(Green_pin, OUTPUT);
  
  Serial.begin(9600);
  while(!Serial); //REMOVE LATER

  /Multiplexer breakout
  Wire.begin();
  Serial.println("\nTCAScanner ready!");
  
  for (uint8_t t=0; t<8; t++) {
    tcaselect(t);
    Serial.print("TCA Port #"); Serial.println(t);

    for (uint8_t addr = 0; addr<=127; addr++) {
      if (addr == TCAADDR) continue;

      Wire.beginTransmission(addr);
      if (!Wire.endTransmission()) {
        Serial.print("Found I2C 0x");  Serial.println(addr,HEX);
      }
    }
  }
  
  //NIR 
  tcaselect(tca_ams1);
  if(!ams1.begin()){
    Serial.println("could not connect to sensor! Please check your wiring.");
    while(1);
  }
  delay(100);

  tcaselect(tca_ams2);
  if(!ams2.begin()){
    Serial.println("could not connect to sensor! Please check your wiring.");
    while(1);
  }
  delay(100);
  
  //QR serial
  Serial1.begin(9600);
}

void loop() {
  //unsigned char fen_L,fen_H,miao_L,miao_H;
  //----------------Initialise
  Serial.println("Initialisation");
  digitalWrite(Red_pin, LOW);
  digitalWrite(Green_pin, LOW);

  tcaselect(tca_ams1);
  board_temp1 = ams1.readTemperature();

  tcaselect(tca_ams2);
  board_temp2 = ams2.readTemperature();
  Serial.print("Temperature: S1 "); Serial.print(board_temp1);  Serial.print(" S2 "); Serial.println(board_temp2);

  //MOVE TO SETUP. REPEATS AFTER LOOP
  //-------------Start test
  Serial.println("NEUOME");
  EPD_HW_Init(); //Electronic paper initialization
  EPD_WhiteScreen_ALL(gImage_main); //Refresh the picture in full screen
  delay(100);
  waitForButtonPress();

  //------------Get QR code
  EPD_HW_Init(); //Electronic paper initialization
  EPD_WhiteScreen_ALL(gImage_qr); //Refresh the picture in full screen
  delay(100);
  
  while(!QR_received){
    byte message[] = {0x7E, 0x00, 0x08, 0x01, 0x00, 0x02, 0x01, 0xAB, 0xCD};
    Serial1.write(message, sizeof(message));
    QR_decoder();

    Serial.println("QR not received");
  }
  Serial.println(decodeString);

  //------------DRY CONTROL----------------------
  Serial.println("Dry control and press button");
  EPD_HW_Init(); //Electronic paper initialization
  EPD_WhiteScreen_ALL(gImage_step1); //Refresh the picture in full screen
  delay(100);
  waitForButtonPress();

  tcaselect(tca_ams1);
  readSpectrometer(1, dry_sensorValues);
  tcaselect(tca_ams2);
  readSpectrometer(2, dry_sensorValuesSpecimen);
  
  if(dry_sensorValuesSpecimen[AS726x_VIOLET] < 100){
    EPD_HW_Init(); //Electronic paper initialization
    EPD_WhiteScreen_ALL(gImage_nostrip); //Refresh the picture in full screen
    delay(100);
    
    Serial.println("Error no strip");
    return;
  }
  else if (dry_sensorValues[AS726x_VIOLET] < 100){
    EPD_HW_Init(); //Electronic paper initialization
    EPD_WhiteScreen_ALL(gImage_nostrip); //Refresh the picture in full screen
    delay(100);
    
    Serial.println("Error no strip");
    return;
  }
  
  //cooloff();

  //------------SAMPLED CONTROL----------------------
  Serial.println("Sampled control and press button");
  EPD_HW_Init(); //Electronic paper initialization
  EPD_WhiteScreen_ALL(gImage_step3); //Refresh the picture in full screen
  delay(100);
  waitForButtonPress();

  tcaselect(tca_ams1);
  readSpectrometer(1, sensorValues);
  tcaselect(tca_ams2);
  readSpectrometer(2, sensorValuesSpecimen);

  if(sensorValues[AS726x_VIOLET] < 100){
    EPD_HW_Init(); //Electronic paper initialization
    EPD_WhiteScreen_ALL(gImage_nostrip); //Refresh the picture in full screen
    delay(100);
    
    Serial.println("Error no strip");
    return;
  }
  else if(sensorValuesSpecimen[AS726x_VIOLET] < 100){
    EPD_HW_Init(); //Electronic paper initialization
    EPD_WhiteScreen_ALL(gImage_nostrip); //Refresh the picture in full screen
    delay(1000);
    
    Serial.println("Error no strip");
    return;
  }
  //cooloff();

//----------RESULT test and restart
  //delay(100);
  computeTest();
  Serial.print("YOUR TEST RESULT: ");
  Serial.print(result);
  Serial.println("");

  switch(result_switch){
    case 0:
      break;
    case 1:
      EPD_HW_Init(); //Electronic paper initialization
      EPD_WhiteScreen_ALL(gImage_error); //Refresh the picture in full screen
      delay(100);
      break;
    case 2:
      EPD_HW_Init(); //Electronic paper initialization
      EPD_WhiteScreen_ALL(gImage_pos); //Refresh the picture in full screen
      delay(100);
      break;
    case 3:
      EPD_HW_Init(); //Electronic paper initialization
      EPD_WhiteScreen_ALL(gImage_neg); //Refresh the picture in full screen
      delay(100);
      break;
    case 4:
      EPD_HW_Init(); //Electronic paper initialization
      EPD_WhiteScreen_ALL(gImage_amb); //Refresh the picture in full screen
      delay(100);
      break;
  }
  waitForButtonPress();
/*
  //REPLACE WITH END OF TEST
  EPD_HW_Init(); //Electronic paper initialization
  EPD_WhiteScreen_ALL(gImage_main); //Refresh the picture in full screen
  delay(100);*/
  
  Serial.println("done...waiting for button press");
  QR_received = false;
  waitForButtonPress();
}

void readSpectrometer(int n_ams, uint16_t ref[]){
  if (n_ams == 1){
    ams1.drvOn();
    ams1.startMeasurement();
    while(!ams1.dataReady()){
      delay(5);
    }
    ams1.readRawValues(ref);
    ams1.drvOff();
    Serial.print("reading=");
    Serial.println(ref[AS726x_VIOLET]);
  }
  else if (n_ams == 2){
    ams2.drvOn();
    ams2.startMeasurement();
    while(!ams2.dataReady()){
      delay(5);
    }
    ams2.readRawValues(ref);
    ams2.drvOff();
    Serial.print("reading=");
    Serial.println(ref[AS726x_VIOLET]);
  }
}

void computeTest(){
  int16_t specimen_diff = sensorValuesSpecimen[AS726x_VIOLET]-dry_sensorValuesSpecimen[AS726x_VIOLET];
  int16_t test_variance = (specimen_diff*1000.0)/(dry_sensorValuesSpecimen[AS726x_VIOLET]*1.0);
  int16_t dry_diff = dry_sensorValues[AS726x_VIOLET] - sensorValues[AS726x_VIOLET];
  int16_t control_variance =  (dry_diff*1000.0)/dry_sensorValues[AS726x_VIOLET]*1.0;
  delay(100);
  margint = test_variance;
  
  Serial.println(dry_sensorValues[AS726x_VIOLET]);
  Serial.println(dry_sensorValuesSpecimen[AS726x_VIOLET]);
  Serial.println(sensorValues[AS726x_VIOLET]);
  Serial.println(sensorValuesSpecimen[AS726x_VIOLET]);
  Serial.print("control_variance");
  Serial.println(control_variance);
  Serial.print("test_variance");
  Serial.println(test_variance);   

  //result switch: 1 repeat test, 2 positive, 3 negative, 4 ambigious
  // in case control increases and is more than 1 percent
  if(control_variance < -10){
    result = String("REPEAT TEST (pnb)");
    result_switch = 1;
    return;
  }

  if(abs(control_variance) > max_control_change_percent*10){
    // Serial.println("repeat");
    result = String("REPEAT TEST (nb)");
    result_switch = 1;
    return;
  }


  if(sensorValuesSpecimen[AS726x_VIOLET] < dry_sensorValuesSpecimen[AS726x_VIOLET]*0.99){
    if(abs(test_variance) >= stain_margin_percent*10){
      result=String("Positive");
      result_switch = 2;
      //digitalWrite(A3, HIGH);     
      digitalWrite(Green_pin, HIGH);  
    } else {
      result=String("Ambiguous");
      result_switch = 4;
      //digitalWrite(A3, LOW);
      //digitalWrite(A2, LOW);      
      digitalWrite(Red_pin, LOW);
      digitalWrite(Green_pin, LOW);
    }
  } 
  
  else {
    if(abs(test_variance) >= max_negative_stain_margin_percent*10){
      result = String("REPEAT TEST (t inc)");
      result_switch = 1;    
    } else {
      result=String("Negative");
      result_switch = 3;
      //digitalWrite(A2, HIGH);
      digitalWrite(Red_pin, HIGH);
    }
  }
}

void cooloff(){
  Serial.println("Processing cooloff");
  //delay(5000);
  tcaselect(tca_ams1);
  uint8_t t1 = ams1.readTemperature();

  tcaselect(tca_ams2);
  uint8_t t2 = ams2.readTemperature();
  
  Serial.print("Temp: t1"); Serial.print(t1); Serial.print(" t2"); Serial.println(t2);
  
  while((ams1.readTemperature() > board_temp1+3) && (ams2.readTemperature() > board_temp2+3)){
    delay(1000);
  }
}

void waitForButtonPress(){
  while(digitalRead(buttonPin)==HIGH){
    delay(10);
  }
  // buffer the input read
  delay(300);
  return;
}

void QR_decoder() {
  int last_keyboard = millis();
  int start_time = millis();
  while(millis()-start_time<=timer*60000){ 
    readString ="";
    while (Serial1.available() > 0) { 
      incomingByte = Serial1.read();
      String thisString = "";
      if (incomingByte <10) {
        thisString = "0"+ String(incomingByte, HEX);     
      } else {
        thisString = String(incomingByte, HEX);
      }
      readString += thisString;
      delay(2);
    }
    delay(1);
    
    if (readString>0){
      if (readString != "02000001003331"){
        //Serial.println(readString);
        decodeString = "";
        char charBuf[50];
        readString.toCharArray(charBuf, 50);
        for (int i = 0; i < strlen(charBuf); i += 2) {
          char val = charBuf[i] > 0x39 ? (charBuf[i] - 'A') * 16 : (charBuf[i] - '0') * 16;
          val += charBuf[i+1] > 0x39 ? (charBuf[i+1] - 'A') : (charBuf[i+1] - '0');
          decodeString += val; 
          }
        QR_received = true;
        //Serial.println(decodeString);
      }
    }
  }
}
/*
void SD_write(){
//write file
  myFile = SD.open(filename, FILE_WRITE);
  if (myFile) {
    Serial.print("Writing to ");
    Serial.print(filename);
    myFile.println("testing 1, 2, 3.");
    myFile.close();
    Serial.println("Writing done");
  } 
  else {
    Serial.println("error opening test.txt");
  }
}
*/
//------------------------Eink functions
///////////////////EXTERNAL FUNCTION////////////////////////////////////////////////////////////////////////
/////////////////////delay//////////////////////////////////////
void driver_delay_us(unsigned int xus)  //1us
{
  for(;xus>1;xus--);
}
void driver_delay_xms(unsigned long xms) //1ms
{  
    unsigned long i = 0 , j=0;

    for(j=0;j<xms;j++)
  {
        for(i=0; i<256; i++);
    }
}
void DELAY_S(unsigned int delaytime)     
{
  int i,j,k;
  for(i=0;i<delaytime;i++)
  {
    for(j=0;j<4000;j++)           
    {
      for(k=0;k<222;k++);
                
    }
  }
}
//////////////////////SPI///////////////////////////////////
void SPI_Delay(unsigned char xrate)
{
  unsigned char i;
  while(xrate)
  {
    for(i=0;i<2;i++);
    xrate--;
  }
}


void SPI_Write(unsigned char value)                                    
{                                                           
    unsigned char i;  
   SPI_Delay(1);
    for(i=0; i<8; i++)   
    {
        EPD_W21_CLK_0;
       SPI_Delay(1);
       if(value & 0x80)
          EPD_W21_MOSI_1;
        else
          EPD_W21_MOSI_0;   
        value = (value << 1); 
       SPI_Delay(1);
       driver_delay_us(1);
        EPD_W21_CLK_1; 
        SPI_Delay(1);
    }
}

void Epaper_Write_Command(unsigned char command)
{
  SPI_Delay(1);
  EPD_W21_CS_0;                   
  EPD_W21_DC_0;   // command write
  SPI_Write(command);
  EPD_W21_CS_1;
}
void Epaper_Write_Data(unsigned char command)
{
  SPI_Delay(1);
  EPD_W21_CS_0;                   
  EPD_W21_DC_1;   // command write
  SPI_Write(command);
  EPD_W21_CS_1;
}

/////////////////EPD settings Functions/////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
void EPD_HW_Init(void)
{
  EPD_W21_RST_0;  // Module reset      
  delay(1); //At least 10ms delay 
  EPD_W21_RST_1; 
  delay(1); //At least 10ms delay 

  Epaper_READBUSY();   
  Epaper_Write_Command(0x12);  //SWRESET
  Epaper_READBUSY();   
    
  Epaper_Write_Command(0x01); //Driver output control      
  Epaper_Write_Data(0xC7);
  Epaper_Write_Data(0x00);
  Epaper_Write_Data(0x00);

  Epaper_Write_Command(0x11); //data entry mode       
  Epaper_Write_Data(0x01);

  Epaper_Write_Command(0x44); //set Ram-X address start/end position   
  Epaper_Write_Data(0x00);
  Epaper_Write_Data(0x18);    //0x0C-->(18+1)*8=200

  Epaper_Write_Command(0x45); //set Ram-Y address start/end position          
  Epaper_Write_Data(0xC7);   //0xC7-->(199+1)=200
  Epaper_Write_Data(0x00);
  Epaper_Write_Data(0x00);
  Epaper_Write_Data(0x00); 

  Epaper_Write_Command(0x3C); //BorderWavefrom
  Epaper_Write_Data(0x05);  
      
  Epaper_Write_Command(0x18); //Reading temperature sensor
  Epaper_Write_Data(0x80);  

  Epaper_Write_Command(0x4E);   // set RAM x address count to 0;
  Epaper_Write_Data(0x00);
  Epaper_Write_Command(0x4F);   // set RAM y address count to 0X199;    
  Epaper_Write_Data(0xC7);
  Epaper_Write_Data(0x00);
  Epaper_READBUSY();
  
}

//////////////////////////////All screen update////////////////////////////////////////////
void EPD_WhiteScreen_ALL(const unsigned char * datas)
{
   unsigned int i;
    Epaper_Write_Command(0x24);   //write RAM for black(0)/white (1)
   for(i=0;i<ALLSCREEN_GRAGHBYTES;i++)
   {               
     Epaper_Write_Data(pgm_read_byte(&datas[i]));
   }
   EPD_Update();   
}
void Epaper_READBUSY(void)
{ 
  while(1)
  {   //=1 BUSY
     if(isEPD_W21_BUSY==0) break;
  }  
}
void EPD_Update(void)
{   
  Epaper_Write_Command(0x22); //Display Update Control
  Epaper_Write_Data(0xF7);   
  Epaper_Write_Command(0x20); //Activate Display Update Sequence
  Epaper_READBUSY();  

}
