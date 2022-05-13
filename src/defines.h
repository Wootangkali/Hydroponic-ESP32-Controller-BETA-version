#pragma once
#include <Arduino.h>

#include "Pin.h"


#include <TimerMs.h>


#include <Adafruit_CCS811.h>
#include "ClosedCube_TCA9548A.h"
#include "RTClib.h"
//#include <TM1637Display.h>
#include <LCD_I2C.h>
#include <Wire.h>
#include <SPI.h>
#include <OneWire.h> //One wire library
#include <DallasTemperature.h> //Library for DS18B20 Sensor
#include <ADS1256.h>
 #include <math.h>// Library for math function 

#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <CD74HC4067.h>



#include <PCF8574.h>

#define EXP_ADDRESS 0x27
#define PINA 0
#define PINB 1
#define BUTTON 2
ENCODER encoder(EXP_ADDRESS, PINA, PINB, BUTTON); // PCF8574 @ address 0b0100/A2/A1/A0, to GND = 0, to LOGIC (3,3V;5V) = 1

PCF8574 expander(0x22);


#define DEBOUNCE 100  // таймаут антидребезга, миллисекунды
#define holdbTimer 350 
 int encoder_pos;
int getMove ;
unsigned long debounceTimer; 
boolean btnState, btnFlag;

int i = 0; // position of rotary encoder
// int bott = 1;
#define INT 15 //PCF8574 interrupt pin vired to INT pin on Arduino UNO (INT0 = pin 2), for more see Arduino pinout
volatile byte readExp = 0; // switch variable, "Do i have read the pins on PCF8574?"
//boolean LEDflag = false;


               // s0 s1 s2 s3
CD74HC4067 my_mux(25, 33, 32, 35);  // create a new CD74HC4067 object with its four control pins

const int g_common_pin = 4; // select a pin to share with the 16 channels of the CD74HC4067


#define DHTPIN 26     // Digital pin connected to the DHT sensor 
#define DHTTYPE    DHT11     // DHT 11
DHT_Unified dht(DHTPIN, DHTTYPE);


#define TCA9548A_I2C_ADDRESS	0x70
// Define the TM1637Display connections pins
// #define CLK 27
// #define DIO 26

#define ONE_WIRE_BUS 27 //data pin  DQ pin of DS18B20 connected to digital pin D5

//define sound speed in cm/uS
#define SOUND_SPEED 0.034
  float Etemp = 22; //convert milli volt to temperature degree Celsius
  float Wtemp = 22;
  //Pin Assignment and declearation Start
  OneWire oneWire(ONE_WIRE_BUS); //Ste up one wire instance
  
  DallasTemperature sensors(&oneWire); //pass one wire reference to DS18B20 library
  DeviceAddress sensor1 = { 0x28, 0x52, 0x37, 0x14, 0x64, 0x20, 0x1, 0x19 };
  DeviceAddress sensor2 = { 0x28, 0x63, 0xFD, 0x7, 0x64, 0x20, 0x1, 0xEC };


// Create rtc and display object
RTC_DS3231 rtc;
//TM1637Display display = TM1637Display(CLK, DIO);
ClosedCube::Wired::TCA9548A tca9548a;

ADS1256 ads;
//Header declearation Start


LCD_I2C lcd(0x27, 16, 2); // Default address of most PCF8574 modules, change according
// const int trigPin = 14;
// const int echoPin = 12;
// long duration;
// float distanceCm;

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch


 float sensorValue = 0 ;
float tdsValue = 0 ;
  float Voltage ;
  float pHValue ;
  long phTot;   //, temTot
  float phAvg; //, temAvg
  int x;
  const float C = 16.96; //Constant of straight line (Y = mx + C)pH = a*xV + b
  //4,01 = a*(3,04)+b
  //6,86 = a*(2,54)+b
  //Тогда я получаю результат y = -5,7*21338.
  const float m = -4.35; // Slope of straight line (Y = mx + C)
  //Pin Assignment and declearation end






// Initialize a PCF8574 at I2C-address 0x20


//If you had a PCF8575 instead you'd use the below format
//PCF857x pcf8575(0x20, &Wire, true);

   
