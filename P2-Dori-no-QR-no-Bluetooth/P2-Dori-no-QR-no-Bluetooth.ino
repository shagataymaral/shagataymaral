//Board: Arduino nano ble or Arduino nano
//Sensor: GY-AS7263
//Multiplexer: TCA9548A
#include "NEMO-images.h""
#include <Arduino.h>  
#include <ArduinoBLE.h>
#include <Wire.h>
#include "Adafruit_AS726x.h"

//LED and BTN
const int Btn_Pin = 4;
const int Red_pin = 3;
const int Green_pin = 2;

//Sensor & Multiplexer
#define TCAADDR 0x70
Adafruit_AS726x ams0;
Adafruit_AS726x ams1;
const int tca_ams0 = 0; //sensor U0, test (away from entrance slot)
const int tca_ams1 = 1; // sensor U1, control 1 (closer to entrance slot)

//Buffer to hold raw values
uint16_t sensorValues[AS726x_NUM_CHANNELS]; //wet control
uint16_t sensorValuesSpecimen[AS726x_NUM_CHANNELS]; //wet test
uint16_t dry_sensorValues[AS726x_NUM_CHANNELS]; //dry control
uint16_t dry_sensorValuesSpecimen[AS726x_NUM_CHANNELS]; //dry test

uint8_t base_temp0;
uint8_t base_temp1;
//uint8_t board_temp0;
//uint8_t board_temp1;

String result;
int result_switch = 0;

//Calculation settings
float stain_margin_percent=4;
float max_control_change_percent=4;
int16_t max_negative_stain_margin_percent=20;
uint16_t min_reading_threshold=40;
double margint;

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
const int CS_Pin = 10;
const int SDI_Pin = 11; 
const int SCK_Pin = 13; 

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

void tcaselect(uint8_t i) {
  if (i > 7) return;
  Wire.beginTransmission(TCAADDR);
  Wire.write(1 << i);
  Wire.endTransmission();  
}

void setup() {
  //Initialise display
  pinMode(BUSY_Pin, INPUT); 
  pinMode(RES_Pin, OUTPUT);
  pinMode(DC_Pin, OUTPUT);        
  pinMode(SCK_Pin, OUTPUT);    
  pinMode(SDI_Pin, OUTPUT);
  pinMode(CS_Pin, OUTPUT);

  //Initialise controllables
  pinMode(Btn_Pin, INPUT);
  pinMode(Red_pin, OUTPUT);
  pinMode(Green_pin, OUTPUT);
  
  Serial.begin(9600);
  while(!Serial); //Only for debug mode

  //Check Multiplexer breakout: verification in case board does not see sensors
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
  
  //Initialise NIR sensors 
  tcaselect(tca_ams0);
  if(!ams0.begin()){
    Serial.println("could not connect to sensor! Please check your wiring.");
    while(1);
  }
  delay(100);

  tcaselect(tca_ams1);
  if(!ams1.begin()){
    Serial.println("could not connect to sensor! Please check your wiring.");
    while(1);
  }
  delay(100);
}

void loop() {
  //Initialise: clear data from previous test
  Serial.println("Initialisation");
  digitalWrite(Red_pin, LOW);
  digitalWrite(Green_pin, LOW);

  //Record base temperature
  tcaselect(tca_ams0);
  base_temp0= ams0.readTemperature();

  tcaselect(tca_ams1);
  base_temp1= ams1.readTemperature();
  
  //TEXT: Greeting information & please press to start test
  EPD_HW_Init();
  EPD_WhiteScreen_ALL(gImage_neuome);
  delay(100);
  waitForButtonPress();

  //Step 1: Dry reading
  EPD_HW_Init();
  EPD_WhiteScreen_ALL(gImage_step1);
  delay(100);
  waitForButtonPress();

  tcaselect(tca_ams1); //control
  readSpectrometer(1, dry_sensorValues);
  tcaselect(tca_ams0); //test
  readSpectrometer(0, dry_sensorValuesSpecimen);
  
  if((dry_sensorValuesSpecimen[AS726x_VIOLET] < 100) || (dry_sensorValues[AS726x_VIOLET] < 100)){
    EPD_HW_Init(); //Electronic paper initialization
    EPD_WhiteScreen_ALL(gImage_no_strip); //Refresh the picture in full screen
    delay(100);
    
    Serial.println("Error no strip");
    return;
  }

  //Step 2: Whistle
  EPD_HW_Init();
  EPD_WhiteScreen_ALL(gImage_step2);
  delay(100);
  waitForButtonPress();

  //Step 2: Wet reading
  EPD_HW_Init();
  EPD_WhiteScreen_ALL(gImage_step3);
  delay(100);
  waitForButtonPress();

  tcaselect(tca_ams1); //test
  readSpectrometer(1, sensorValues);
  tcaselect(tca_ams0); //control
  readSpectrometer(0, sensorValuesSpecimen);

  if((sensorValues[AS726x_VIOLET] < 100)||(sensorValuesSpecimen[AS726x_VIOLET] < 100)){
    EPD_HW_Init();
    EPD_WhiteScreen_ALL(gImage_no_strip);
    delay(100);
    
    Serial.println("Error no strip");
    return;
  }

  //Compute result
  computeTest();
  Serial.print("YOUR TEST RESULT: "); Serial.print(result); Serial.println("");
  switch(result_switch){
    case 0:
      break;
    case 1: //ERROR
      digitalWrite(Green_pin, LOW);
      digitalWrite(Red_pin, LOW);

      digitalWrite(Green_pin, LOW);
      digitalWrite(Red_pin, LOW);
      
      EPD_HW_Init();
      EPD_WhiteScreen_ALL(gImage_error);
      delay(100);
      break;
    case 2: //POSITIVE
      digitalWrite(Red_pin, HIGH);
      
      EPD_HW_Init();
      EPD_WhiteScreen_ALL(gImage_pos);
      delay(100);
      break;
    case 3: //NEGATIVE
      digitalWrite(Green_pin, HIGH);
      
      EPD_HW_Init();
      EPD_WhiteScreen_ALL(gImage_neg);
      delay(100);
      break;
    case 4: //AMBIGIOUS
      digitalWrite(Green_pin, LOW);
      digitalWrite(Red_pin, LOW);
      
      EPD_HW_Init();
      EPD_WhiteScreen_ALL(gImage_amb);
      delay(100);
      break;
  }

  //End of test
  Serial.println("End of test, press btn for new test");
  waitForButtonPress();
}

void readSpectrometer(int n_ams, uint16_t ref[]){
  if (n_ams == 0){
    ams0.drvOn();
    ams0.startMeasurement();
    while(!ams0.dataReady()){
      delay(5);
    }
    ams0.readRawValues(ref);
    ams0.drvOff();
    
    while(ams0.readTemperature() > base_temp0+3){
      Serial.println("temperature delay");
      delay(1000);
    }
    
    Serial.print("reading="); Serial.println(ref[AS726x_VIOLET]);
  }
  else if (n_ams == 1){
    ams1.drvOn();
    ams1.startMeasurement();
    while(!ams1.dataReady()){
      delay(5);
    }
    ams1.readRawValues(ref);
    ams1.drvOff();

    while(ams1.readTemperature() > base_temp1+3){
      Serial.println("temperature delay");
      delay(1000);
    }
    Serial.print("reading="); Serial.println(ref[AS726x_VIOLET]);
  }
  else{
    Serial.print("Error, undefined sensor");
  }
}

void computeTest(){
  int16_t specimen_diff = sensorValuesSpecimen[AS726x_VIOLET]-dry_sensorValuesSpecimen[AS726x_VIOLET];
  int16_t test_variance = (specimen_diff*1000.0)/(dry_sensorValuesSpecimen[AS726x_VIOLET]*1.0);
  int16_t dry_diff = dry_sensorValues[AS726x_VIOLET] - sensorValues[AS726x_VIOLET];
  int16_t control_variance =  (dry_diff*1000.0)/dry_sensorValues[AS726x_VIOLET]*1.0;
  margint = test_variance;
  
  Serial.print("Dry control and test"); Serial.print(dry_sensorValues[AS726x_VIOLET]); Serial.print(" "); Serial.println(dry_sensorValuesSpecimen[AS726x_VIOLET]);
  Serial.print("Wet control and test"); Serial.print(sensorValues[AS726x_VIOLET]); Serial.print(" "); Serial.println(sensorValuesSpecimen[AS726x_VIOLET]);
  Serial.print("control_variance ");Serial.print(control_variance); Serial.print(" test_variance ");Serial.println(test_variance); 
  
  //result switch: 1 repeat test, 2 positive, 3 negative, 4 ambigious
  //in case control increases and is more than 1 percent
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
    } 
    else {
      result=String("Ambiguous");
      result_switch = 4;  
    }
  } 
  else {
    if(abs(test_variance) >= max_negative_stain_margin_percent*10){
      result = String("REPEAT TEST (t inc)");
      result_switch = 1;    
    } 
    else {
      result=String("Negative");
      result_switch = 3;
    }
  }
}

//IF custom pcb low, if button module high
void waitForButtonPress(){
  while(digitalRead(Btn_Pin)==LOW){
    delay(10);
  }
  // buffer the input read
  delay(300);
  return;
}









//------------------------Eink functions
///EXTERNAL FUNCTION
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
///SPI
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

//EPD settings Functions
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

//All screen update
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
