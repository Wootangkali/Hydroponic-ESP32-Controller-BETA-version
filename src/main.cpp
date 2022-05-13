// Include the libraries
// #pragma once
#include <Arduino.h>
#include "defines.h"
#include "Pin.h"
  const float C = 16.96; //Constant of straight line (Y = mx + C)pH = a*xV + b
  //4,01 = a*(3,04)+b
  //6,86 = a*(2,54)+b
  //Тогда я получаю результат y = -5,7*21338.
  const float m = -4.35; // Slope of straight line (Y = mx + C)
  //Pin Assignment and declearation end


#include "Adafruit_CCS811.h"

Adafruit_CCS811 ccs;

 volatile bool PCFInterruptFlag = false;

    void PCFInterrupt() {
      PCFInterruptFlag = true;
    };
void TCA9548A(uint8_t bus) //function of TCA9548A
{
  Wire.beginTransmission(0x70);  // TCA9548A address is 0x70
  Wire.write(1 << bus);          // send byte to select bus
  Wire.endTransmission();
};
void expInterrupt()
{
  detachInterrupt(INT); // deactivate interrupt, so we don't interrupt our interruption
  readExp = 0;  // if pressed, interrupt sets readExp to 1, to read expander
  Serial.println("INT");
};

void setup() {
 Wire.begin();
encoder.init(HIGH); //sets pins A, B and BUTTON pin to LOW/HIGH
 encoder.setP( 3, LOW);
  encoder.setP( 4, LOW);
   encoder.setP( 5, LOW);
    encoder.setP( 6, LOW);
     encoder.setP( 7, LOW);

     
 attachInterrupt(INT, expInterrupt, HIGH); //attach interrupt to INT pin, what function to execute, trigger state


  Serial.println("serial start");

 

 pinMode(g_common_pin, OUTPUT); // set the initial mode of the common pin.

TCA9548A(0);
 lcd2.begin(); // If you are using more I2C devices using the Wire library use lcd.begin(false)
                 // this stop the library(LCD_I2C) from calling Wire.begin()
    lcd2.backlight();   



   dht.begin();
  // Print temperature sensor details.
  sensor_t sensor;
  dht.temperature().getSensor(&sensor);
    dht.humidity().getSensor(&sensor);

TCA9548A(3);

 if(!ccs.begin()){
    Serial.println("Failed to start sensor! Please check your wiring.");
    while(1);
  };

  // Wait for the sensor to be ready
  while(!ccs.available());



//  Specsheets say PCF8574 is officially rated only for 100KHz I2C-bus
  //PCF8575 is rated for 400KHz
  TCA9548A(6);
  Wire.setClock(100000L);
  pcf1.begin();
 pcf1.resetInterruptPin();
 pcf1.write(0, 1);
  pcf1.write(1, 0);
  pcf1.write(2, 1);
  pcf1.write(3, 0);
  pcf1.write(4, 0);
  pcf1.write(5, 1);
  pcf1.write(6, 0);
  pcf1.write(7, 1);


 

  ads.init( 5, 17, 16, 1700000 );
  Serial.println( ads.speedSPI );
  sensors.begin(); //Start the DS18B20 Library
	// Begin serial communication
 //pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
 // pinMode(echoPin, INPUT); // Sets the echoPin as an Input  
	Serial.begin(115200);
 Wire.begin();
  TCA9548A(1);
    lcd.begin(); // If you are using more I2C devices using the Wire library use lcd.begin(false)
                 // this stop the library(LCD_I2C) from calling Wire.begin()
    lcd.backlight();

 


	// Check if RTC is connected correctly
   TCA9548A(7);
	if (! rtc.begin()) {
		Serial.println("Couldn't find RTC");
		while (1);
	};
	// Check if the RTC lost power and if so, set the time
	if (rtc.lostPower()) {
		Serial.println("RTC lost power, lets set the time!");
		// The following line sets the RTC to the date & time this sketch was compiled:
		rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
		// This line sets the RTC with an explicit date & time, for example to set
		// January 21, 2014 at 3am you would call:
		//rtc.adjust(DateTime(2014, 1, 21, 3, 0, 0));
	};

	// Set the display brightness (0-7)
	// display.setBrightness(7);
	
	// // Clear the display
	// display.clear();
TCA9548A(0);
lcd2.print("     Hello"); // You can make spaces using well... spaces
    lcd2.setCursor(5, 1); // Or setting the cursor in the desired position.
    lcd2.print("World!");


 
}

void loop() {
 ads.readInputToAdcValuesArray();





    // float temVoltage = temAvg * 5 / 8.388608 ; //convert sensor reading into milli volt
    float phVoltage =  ads.adcValues[0] ;
    // * 5 / 8.388608 / 1000000;  //convert sensor reading into milli volt
   float phAvg = phVoltage *5  / 8.388608 / 1000000;

    float pHValue = phAvg*m+C;




 
 digitalWrite(g_common_pin, HIGH);

  
        my_mux.channel(0);
        // delay(1500);
    
     digitalWrite(g_common_pin, LOW);
    //  delay(1500);


  // i = i + encoder.getMovement();
btnState = !encoder.getButton();

getMove = encoder.getMovement();

  if (readExp == 0) // if readExp is set, read expander
  {
    i = i + encoder.getMovement();
    // Serial.println(String(i) + "/button: " + String(encoder.getButton()));
    readExp = 1;
    attachInterrupt(INT, expInterrupt, LOW); // attach again
  }



// if (encoder.getMovement() == 1);
//  { Serial.println("getMovement+++");};

// if (encoder.getMovement() == -1);
//  { Serial.println("getMovement---");};




if (btnState && !btnFlag && (millis() - debounceTimer > DEBOUNCE)) {
    btnFlag = true;              // запомнили что нажата
    debounceTimer = millis();    // запомнили время нажатия
    Serial.println("press");
  };
  if (!btnState && btnFlag) {    // если отпущена и была нажата (btnFlag 1)
    btnFlag = false;             // запомнили что отпущена
    debounceTimer = millis();    // запомнили время отпускания
    Serial.println("release");
  };
    

if (getMove == 1 && !btnState)
{ Serial.println("Right");
};

if (getMove == -1  && !btnState)
{ Serial.println("Left");
};

 if (btnState && btnFlag and (getMove == 1) and  (millis() - debounceTimer > holdbTimer)) 
 {
 
Serial.println("Hold++");
};

if (btnState and btnFlag and (getMove == -1) and  (millis() - debounceTimer > holdbTimer)) 
 {
 
Serial.println("Hold--");

  };



//   TCA9548A(6);
//   Serial.println(pcf1.read(0));
// Serial.println(pcf1.read(1));
// Serial.println(pcf1.read(2));
// Serial.println(pcf1.read(3));
// Serial.println(pcf1.read(4));
// Serial.println(pcf1.read(5));
// Serial.println(pcf1.read(6));
// Serial.println(pcf1.read(7));
// //  delay(1000);
//   pcf1.write(0, 1);
//   Serial.print("++");
// //  delay(1000);
//  Serial.print("--");
//   pcf1.write(0, 0);
//   //  delay(1000);
//    Serial.print("+");
//   pcf1.write(0, 1);
// //  delay(1000);
//  Serial.print("-");
//   pcf1.write(0, 0);
 
sensors.requestTemperatures(); // Send the command to get temperatures
	// Get current date and time
 DateTime now = rtc.now();
  //  каждую секунду
  if (millis() - myTimer >= 10000) {
    myTimer = millis(); // сбросить таймер
    // digitalWrite(13, LEDflag); // вкл/выкл
    // LEDflag = !LEDflag; // инвертировать флаг
 
  };
	// Create time format to display
	// int displaytime = (now.hour() * 100) + now.minute();

	// Display the current time in 24 hour format with leading zeros and a center colon enabled
	// display.showNumberDecEx(displaytime, 0b11100000, true);
//    TCA9548A(1);
//  digitalWrite(trigPin, LOW);
//   delayMicroseconds(2);
//   // Sets the trigPin on HIGH state for 10 micro seconds
//   digitalWrite(trigPin, HIGH);
//   delayMicroseconds(10);
//   digitalWrite(trigPin, LOW);
//   // Reads the echoPin, returns the sound wave travel time in microseconds
//   duration = pulseIn(echoPin, HIGH);
//   // Calculate the distance
//   distanceCm = duration * SOUND_SPEED/2;
  
     
  //   TCA9548A(7);
	 
  //   for(int i = 3; i < 8; i++){
  //   Serial.print("P"+String(i)+": ");
  //   Serial.print(String(encoder.readP(i))); // read input i, returns 0 or 1
  //   Serial.print("/");
  // };

  
  // Prints the distance in the Serial Monitor
  // Serial.print("Distance (cm): ");
  // Serial.println(distanceCm);
 //Serial.println(displaytime);
  // set cursor to first column, first row
  TCA9548A(0);
  lcd2.setCursor(0, 0);
  // print message
  lcd2.print("Distance");
  // set cursor to first column, second row
  lcd2.setCursor(0,1);
  // lcd.print(distanceCm);
  // lcd.print(" cm");

  float Etemp = sensors.getTempC(sensor1); //convert milli volt to temperature degree Celsius
  float Wtemp = sensors.getTempC(sensor2);


    lcd2.clear();
    lcd2.setCursor(0,0);
    lcd2.print("Env.Tmp.");
    lcd2.setCursor(12,0);
    lcd2.print("Wat.Tmp.");
    lcd2.setCursor(1,1);
    lcd2.print(Etemp);
    lcd2.setCursor(6,1);
    lcd2.write(B11011111);
    lcd2.setCursor(7,1);
    lcd2.print("C");
    lcd2.setCursor(13,1);
    lcd2.print(Wtemp);
    lcd2.setCursor(18,1);
    lcd2.write(B11011111);
    lcd2.setCursor(19,1);
    lcd2.print("C");
    lcd2.setCursor(0,2);
    lcd2.print("PH Value of Solution");
    lcd2.setCursor(0,3);
    lcd2.print(pHValue, 3);
    lcd2.setCursor(6,3);
    lcd2.print("PH");
    lcd2.setCursor(14,3);
    lcd2.print(tdsValue);
    lcd2.setCursor(17,3);
    lcd2.print("Ppm");


sensors_event_t event;
  dht.temperature().getEvent(&event);
 
   
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  

  // 3 раза в секунду
 if (millis() - myTimer1 >= 20000) {
   myTimer1 = millis(); // сбросить таймер
Serial.print("Etemp");
 Serial.println(Etemp);
Serial.print("Wtemp");
  Serial.println(Wtemp);

  // Serial.print("TDS Value = ");
  // Serial.print(tdsValue);
  // Serial.println(" ppm");
    //myTimer1 = millis(); // сбросить таймер
    // Serial.println("timer 1");
     Serial.print("PHvOlt");
// Serial.println(ads.adcValues[0] * 5 / 8.388608 / 1000000, 7);
Serial.println(phAvg, 7);

                     Serial.print("PH=");
// Serial.println(ads.adcValues[0] * 5 / 8.388608 / 1000000, 7);
Serial.println(pHValue, 3);

   Serial.print("Ppmv=");
  //Serial.println(ads.adcValues[1] * 5 / 8.388608 / 1000000, 7);  // Raw ADC integer value +/- 23 bits
Serial.println(tdsValue);

  
    Serial.print( "      " );
    //  Serial.print(F("Temperature: "));
    // Serial.print(event.temperature);
    // Serial.println(F("°C"));
 Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));

 };


 

 
  // Serial.println();




 
   TCA9548A(3);
if(ccs.available()){
    if(!ccs.readData());

};

  // каждые 2 секунды
  if (millis() - myTimer2 >= 20000) {
    myTimer2 = millis(); // сбросить таймер
     Serial.println("timer 2");

  
      Serial.print("CO2: ");
      Serial.print(ccs.geteCO2());
      Serial.print("ppm, TVOC: ");
      Serial.println(ccs.getTVOC());
   
} ;
  // каждые 5 секунд
  if (millis() - myTimer3 >= 5000) {
   myTimer3 = millis(); // сбросить таймер
    Serial.println("timer 3");
   
 // phTot ;
 // temTot = 0;
 // phAvg ;
// temAvg = 0;
  };
//taking 10 sample and adding with 10 milli second delay
//  for(x=0; x<10 ; x++)
   // {
    //    phTot += ads.adcValues[ 0 ];
    //    temTot += ads.adcValues[ 1 ];
   
   // };
   // float temAvg = temTot/1000000;
    //float phAvg = phTot/10000000;
    // // float temVoltage = temAvg * 5 / 8.388608 ; //convert sensor reading into milli volt
  //   float phVoltage =  ads.adcValues[0] ;  //convert sensor reading into milli volt
  //  float phAvg = phVoltagE * 5 / 8.388608 / 1000000;

  //  float pHValue = phVoltage*m+C;
 
    float TempDif = fabs(Etemp-Wtemp); //calculating the absolute value of floating
    float   sensorValue = ads.adcValues[1];
    Voltage = sensorValue * 5 / 8.388608  / 1000000; //Convert analog reading to Voltage
    tdsValue=(133.42/Voltage*Voltage*Voltage - 255.86*Voltage*Voltage + 857.39*Voltage)*0.5; //Convert voltage value to TDS value 
  
 };


