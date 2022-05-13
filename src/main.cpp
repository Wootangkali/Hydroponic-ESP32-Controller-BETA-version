// Include the libraries
// #pragma once
 #include <Arduino.h>
#include "defines.h"
#include "Pin.h"



// переменные времени
// (период, мс), (0 не запущен / 1 запущен), (режим: 0 период / 1 таймер)
TimerMs tmr1(10000, 1, 1);
TimerMs tmr2(10000, 1, 1);
TimerMs tmr3(10000, 1, 1);
TimerMs tmr4(10000, 1, 1);
TimerMs tmr5(10000, 1, 1);

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
  readExp = 1;  // if pressed, interrupt sets readExp to 1, to read expander
  Serial.println("INT");
};

void setup() {
    //tmr.setTimerMode();
  tmr1.setPeriodMode();
tmr2.setPeriodMode();
tmr3.setPeriodMode();
tmr4.setPeriodMode();
tmr5.setPeriodMode();


 Wire.begin();
encoder.init(HIGH); //sets pins A, B and BUTTON pin to LOW/HIGH
 
  TCA9548A(2);
  Wire.setClock(400000L);
expander.setP( 8, INPUT);



 attachInterrupt(INT, expInterrupt, LOW); //attach interrupt to INT pin, what function to execute, trigger state


  Serial.println("serial start");

 

 pinMode(g_common_pin, OUTPUT); // set the initial mode of the common pin.




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


 

  ads.init( 5, 17, 16, 1700000 );
  Serial.println( ads.speedSPI );
  sensors.begin(); //Start the DS18B20 Library
	// Begin serial communication
 //pinMode(trigPin, OUTPUT); // Sets the trigPin as an Output
 // pinMode(echoPin, INPUT); // Sets the echoPin as an Input  
	Serial.begin(115200);
 //Wire.begin();
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


 
}

void loop() {

  

 digitalWrite(g_common_pin, HIGH);

  
        my_mux.channel(0);
        // delay(1500);
    
     digitalWrite(g_common_pin, LOW);
    //  delay(1500);


  i = i + encoder.getMovement();
btnState = !encoder.getButton();

getMove = encoder.getMovement();

// if (encoder.getMovement() == 1);
//  { Serial.println("getMovement+++");};

// if (encoder.getMovement() == -1);
//  { Serial.println("getMovement---");};




if (btnState && !btnFlag && (millis() - debounceTimer > DEBOUNCE)) {
    btnFlag = true;              // запомнили что нажата
    debounceTimer = millis();    // запомнили время нажатия
    Serial.println("press");
  }
  if (!btnState && btnFlag) {    // если отпущена и была нажата (btnFlag 1)
    btnFlag = false;             // запомнили что отпущена
    debounceTimer = millis();    // запомнили время отпускания
    Serial.println("release");
  }
    

if (getMove == 1 && !btnState)
{ Serial.println("Right");
}

if (getMove == -1  && !btnState)
{ Serial.println("Left");
}

 if (btnState && btnFlag and (getMove == 1) and  (millis() - debounceTimer > holdbTimer)) 
 {
 
Serial.println("Hold++");
}

if (btnState and btnFlag and (getMove == -1) and  (millis() - debounceTimer > holdbTimer)) 
 {
 
Serial.println("Hold--");

  }


 

	// Get current date and time
 
  //  каждую секунду
  // if (tmr1.tick()) {
  
  //   //myTimer = millis(); // сбросить таймер
  //   // digitalWrite(13, LEDflag); // вкл/выкл
  //   // LEDflag = !LEDflag; // инвертировать флаг
  //    sensors.requestTemperatures(); // Send the command to get temperatures
  // //  TCA9548A(7);
	// // DateTime now = rtc.now();
  // }
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
  
 
  
  // Prints the distance in the Serial Monitor
  // Serial.print("Distance (cm): ");
  // Serial.println(distanceCm);
 //Serial.println(displaytime);
  // set cursor to first column, first row
  TCA9548A(1);
  lcd.setCursor(0, 0);
  // print message
  lcd.print("Distance");
  // set cursor to first column, second row
  lcd.setCursor(0,1);
  // lcd.print(distanceCm);
  // lcd.print(" cm");

  float Etemp = sensors.getTempC(sensor1); //convert milli volt to temperature degree Celsius
  float Wtemp = sensors.getTempC(sensor2);


    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Env.Tmp.");
    lcd.setCursor(12,0);
    lcd.print("Wat.Tmp.");
    lcd.setCursor(1,1);
    lcd.print(Etemp);
    lcd.setCursor(6,1);
    lcd.write(B11011111);
    lcd.setCursor(7,1);
    lcd.print("C");
    lcd.setCursor(13,1);
    lcd.print(Wtemp);
    lcd.setCursor(18,1);
    lcd.write(B11011111);
    lcd.setCursor(19,1);
    lcd.print("C");
    lcd.setCursor(0,2);
    lcd.print("PH Value of Solution");
    lcd.setCursor(0,3);
    lcd.print(pHValue, 3);
    lcd.setCursor(6,3);
    lcd.print("PH");
    lcd.setCursor(14,3);
    lcd.print(tdsValue, 0);
    lcd.setCursor(17,3);
    lcd.print("Ppm");



  // 3 раза в секунду
  if (tmr2.tick()) {
   
Serial.print("Etemp");
 Serial.println(Etemp);
Serial.print("Wtemp");
  Serial.println(Wtemp);

  Serial.print("TDS Value = ");
  Serial.print(tdsValue);
  Serial.println(" ppm");
    //myTimer1 = millis(); // сбросить таймер
    // Serial.println("timer 1");
     Serial.print("PHv==");
Serial.println(ads.adcValues[0] * 5 / 8.388608 / 1000000, 7);

   Serial.print("Ppmv=");
  Serial.println(ads.adcValues[1] * 5 / 8.388608 / 1000000, 7);  // Raw ADC integer value +/- 23 bits
    Serial.print( "      " );
sensors_event_t event;
  dht.temperature().getEvent(&event);
  if (isnan(event.temperature)) {
    Serial.println(F("Error reading temperature!"));
  }
  else {
    Serial.print(F("Temperature: "));
    Serial.print(event.temperature);
    Serial.println(F("°C"));
  }
  // Get humidity event and print its value.
  dht.humidity().getEvent(&event);
  if (isnan(event.relative_humidity)) {
    Serial.println(F("Error reading humidity!"));
  }
  else {
    Serial.print(F("Humidity: "));
    Serial.print(event.relative_humidity);
    Serial.println(F("%"));
  }
 
 
 
   if (tmr2.tick()) {


 TCA9548A(2);
for(int i = 0; i < 8; i++){
    Serial.print("P"+String(i)+": ");
    Serial.print(String(expander.readP(i))); // read input i, returns 0 or 1
    Serial.print("/");
  }
  }
  //Serial.println(String(expander.readP(8), BIN)); // read all inputs, returns 0-255




  // каждые 2 секунды
  if (tmr3.tick()) {

    
  }
  // каждые 5 секунд
  if (tmr4.tick()) {

   
    ads.readInputToAdcValuesArray();
       TCA9548A(3);
if(ccs.available()){
    if(!ccs.readData()){
      Serial.print("CO2: ");
      Serial.print(ccs.geteCO2());
      Serial.print("ppm, TVOC: ");
      Serial.println(ccs.getTVOC());
    }
    else{
      Serial.println("ERROR!");
      while(1);
    }
} 
 // phTot ;
 // temTot = 0;
 // phAvg ;
// temAvg = 0;
  }
//taking 10 sample and adding with 10 milli second delay
  for(x=0; x<10 ; x++)
    {
        phTot += ads.adcValues[ 0 ];
    //    temTot += ads.adcValues[ 1 ];
   
    }
   // float temAvg = temTot/1000000;
    float phAvg = phTot/10000000;
    // float temVoltage = temAvg * 5 / 8.388608 ; //convert sensor reading into milli volt
    float phVoltage =  phAvg * 5 / 8.388608 ;//convert sensor reading into milli volt
   

    float pHValue = phVoltage*m+C;
 
    float TempDif = fabs(Etemp-Wtemp); //calculating the absolute value of floating
    float   sensorValue = ads.adcValues[ 2 ];
    Voltage = sensorValue * 5 / 8.388608  / 1000000; //Convert analog reading to Voltage
    tdsValue=(133.42/Voltage*Voltage*Voltage - 255.86*Voltage*Voltage + 857.39*Voltage)*0.5; //Convert voltage value to TDS value 
   }

  






// // // // delay(500);


// // //  }
 

// // /*
// //  * PCF8574 GPIO Port Expand
// //  * https://www.mischianti.org/2020/03/13/pcf8574-i2c-digital-i-o-expander-rotary-encoder-part-2/
// //  *
// //  * PCF8574    ----- WeMos
// //  * A0         ----- GRD
// //  * A1         ----- GRD
// //  * A2         ----- GRD
// //  * VSS        ----- GRD
// //  * VDD        ----- 5V/3.3V
// //  * SDA        ----- D1(PullUp)
// //  * SCL        ----- D2(PullUp)
// //  * INT        ----- INT(PullUp)
// //  *
// //  * P0     ----------------- ENCODER PIN A
// //  * P1     ----------------- ENCODER PIN B
// //  * P2     ----------------- ENCODER BUTTON
// //  *
// //  */
// // // #include "Arduino.h"
// // // #include "PCF8574.h"
// // // #include <SPI.h>
// // // int encoderPinA = P0;
// // // int encoderPinB = P1;

// // // #define INTERRUPTED_PIN 15

// // // void ICACHE_RAM_ATTR updateEncoder();

// // // // initialize library
// // // PCF8574 pcf8574(0x27, INTERRUPTED_PIN, updateEncoder);

// // // volatile long encoderValue = 0;
// // // uint8_t encoderButtonVal = HIGH;

// // // void setup()
// // // {
// // // 	  Serial.begin (115200);
// // // 	  delay(500);

// // // 	  // encoder pins
// // // 	  pcf8574.pinMode(encoderPinA, INPUT_PULLUP);
// // // 	  pcf8574.pinMode(encoderPinB, INPUT_PULLDOWN);
// // // 	  // encoder button
// // // 	  pcf8574.pinMode(P2, INPUT_PULLDOWN);

// // // 	  // Set low latency with this method or uncomment LOW_LATENCY define in the library
// // // 	  // Needed for encoder
// // // 	  pcf8574.setLatency(0);

// // // 	  // Start library
// // // 		Serial.print("Init pcf8574...");
// // // 		if (pcf8574.begin()){
// // // 			Serial.println("OK");
// // // 		}else{
// // // 			Serial.println("KO");
// // // 		}
// // // }

// // // bool changed = false;

// // // void loop()
// // // {
// // // 	if (changed){
// // // 		Serial.print("ENCODER --> ");
// // // 		Serial.print(encoderValue);
// // // 		Serial.print(" - BUTTON --> ");
// // // 		Serial.println(encoderButtonVal?"HIGH":"LOW");
// // // 		changed = false;
// // // 	}
// // // }

// // // uint8_t encoderPinALast = LOW;
// // // uint8_t valPrecEncoderButton = HIGH;

// // // void updateEncoder(){
// // // 	// Encoder management
// // // 	uint8_t n = pcf8574.digitalRead(encoderPinA);
// // // 	if ((encoderPinALast == LOW) && (n == HIGH)) {
// // // 		if (pcf8574.digitalRead(encoderPinB) == LOW) {
// // // 			encoderValue--;
// // // 			changed = true; // Chnged the value
// // // 		} else {
// // // 			encoderValue++;
// // // 			changed = true; // Chnged the value
// // // 		}
// // // 	}
// // // 	encoderPinALast = n;

// // // 	// Button management
// // // 	encoderButtonVal = pcf8574.digitalRead(P2);
// // // 	if (encoderButtonVal!=valPrecEncoderButton){
// // // 	  changed = true; // Chnged the value of button
// // // 	  valPrecEncoderButton = encoderButtonVal;
// // // 	}
// // // }



// //  /*
// //  This sketch gives an example of using a rotary encoder for directional use only, it also captures
// //  the push-button that's on the rotary encoder.
 
// //  Switch input is designed to work with the task manager class which
// //  makes scheduling tasks trivial.

// //  Circuit / detail: https://www.thecoderscorner.com/products/arduino-downloads/io-abstraction/arduino-switches-handled-as-events/

// // */
// // //  #include <AUnit.h>

// // // #if defined(__AVR__)

// // // #include <util/atomic.h>
// // // #include "IoAbstraction.h"
// // // #include "MockIoAbstraction.h"

// // // using namespace aunit;

// // // // We can only reset the clock to a new value on AVR, this is very useful and allows us to ensure the
// // // // rollover cases work properly at least for milliseconds. As millisecond and microsecond logic are very
// // // // similar it gives some degree of confidence that it's working properly.
// // // //
// // // // Keep this test on it's own in this package. It messes around with the millisecond counter.

// // // void setMillis(unsigned long ms)
// // // {
// // //     extern unsigned long timer0_millis;
// // //     ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
// // //         timer0_millis = ms;
// // //     }
// // // }

// // // void dumpTaskTiming() {
// // //     Serial.println("Showing task timings");
// // //     TimerTask* task = taskManager.getFirstTask();
// // //     while(task) {
// // //         Serial.print(" - Timing "); Serial.println(task->microsFromNow());
// // //         task = task->getNext();
// // //     }
// // // }

// // // int avrCount1 = 0;
// // // int avrCount2 = 0;

// // // //
// // // // this test only runs on AVR - it sets the timer near to overflow and schedules some tasks
// // // //
// // // test(testClockRollover) {
// // //     avrCount1 = avrCount2 = 0;

// // //     // set the clock so that it will roll
// // //     uint32_t oldMillis = millis();
// // //     setMillis(0xfffffe70UL);

// // //     taskManager.scheduleOnce(1, [] {
// // //         avrCount1++;
// // //     }, TIME_SECONDS);

// // //     taskManager.scheduleFixedRate(250, [] {
// // //         avrCount2++;
// // //     }, TIME_MICROS);

// // //     // make sure it's still to wrap.
// // //     assertTrue(millis() > 100000000UL);

// // //     // now run the loop
// // //     dumpTaskTiming();
// // //     unsigned long start = millis();
// // //     while(avrCount1 == 0 && (millis() - start) < 5000) {
// // //         taskManager.yieldForMicros(10000);
// // //     }

// // //     dumpTaskTiming();

// // //     // the one second task should have executed exactly once.
// // //     assertEqual(avrCount1, 1);
// // //     assertMore(avrCount2, 1000);

// // //     // make sure millis has wrapped now.
// // //     assertTrue(millis() < 10000UL);

// // //     // and make sure the microsecond job is still going..
// // //     int avrCount2Then = avrCount2;
// // //     taskManager.yieldForMicros(10000);
// // //     assertTrue(avrCount2Then != avrCount2);

// // //     // reset the millisecond timer where it was before.
// // //     setMillis(oldMillis);
// // // }
// // // #endif // __AVR__
// // /*
// //  This sketch is an example of using either one or two rotary encoders along with two switches and an LED over an
// //  i2c 8574 IO expander.

// //  One switch is the encoders push switch, configured as non-repeating. The other switch is repeating and will toggle
// //  the LED. In this example the switches library is set up in interrupt mode, which means unless something is pressed
// //  down, there is no polling.

// //  Switch input is designed to work with the task manager class which makes scheduling tasks trivial.

// //  Circuit / detail: https://www.thecoderscorner.com/products/arduino-libraries/io-abstraction/rotary-encoder-switches-interrupt-pcf8574/

// //  2020-11-29: We've added the possibility of an additional encoder connected to the PCF8574, to give you two encoders
// //              on the same device. Each section of code for the 2nd encoder is wrapped with comments, to make it both
// //              easy to identify what you do to add another encoder, and secondly, to make it easy to comment out.
// //              We've also added an example of how to register a listener style callback for a key-press.

// // */

// // // We have a direct dependency on Wire and Arduino ships it as a library for every board
// // // therefore to ensure compilation we include it here.
// // #include <Wire.h>
// //  #include <SPI.h>
// // /*
// //  This sketch is an example of using either one or two rotary encoders along with two switches and an LED over an
// //  i2c 8574 IO expander.

// //  One switch is the encoders push switch, configured as non-repeating. The other switch is repeating and will toggle
// //  the LED. In this example the switches library is set up in interrupt mode, which means unless something is pressed
// //  down, there is no polling.

// //  Switch input is designed to work with the task manager class which makes scheduling tasks trivial.

// //  Circuit / detail: https://www.thecoderscorner.com/products/arduino-libraries/io-abstraction/rotary-encoder-switches-interrupt-pcf8574/

// //  2020-11-29: We've added the possibility of an additional encoder connected to the PCF8574, to give you two encoders
// //              on the same device. Each section of code for the 2nd encoder is wrapped with comments, to make it both
// //              easy to identify what you do to add another encoder, and secondly, to make it easy to comment out.
// //              We've also added an example of how to register a listener style callback for a key-press.

// // */

// // // We have a direct dependency on Wire and Arduino ships it as a library for every board
// // // therefore to ensure compilation we include it here.
// // #include <Wire.h>
// // #include <Wire.h>
// //  #include <SPI.h>
// // #include<IoAbstraction.h>
// // #include<IoAbstractionWire.h>
// // #include <TaskManagerIO.h>

// // // The pin onto which we connected the rotary encoders switch
// // const int spinWheelClickPin = 5;

// // // The pin onto which we connected the repeat button switch
// // const int repeatButtonPin = 4;

// // // the led pin on the IO ioDevice
// // const int ledPin = 1;

// // // The two pins where we connected the A and B pins of the encoder. I recommend you dont change these
// // // as the pin must support interrupts.
// // const int encoderAPin = 6;
// // const int encoderBPin = 7;

// // // the maximum value (starting from 0) that we want the encoder to represent. This range is inclusive.
// // const int maximumEncoderValue = 128;

// // // used when we toggle the LED state further down.
// // bool currentLedState;

// // // Start 2nd Encoder
// // // Here are the fields for the second rotary encoder
// // // If you only want one encoder you can comment these fields out.
// // const int encoder2Click = 2;
// // const int encoder2APin = 0;
// // const int encoder2BPin = 1;
// // HardwareRotaryEncoder *secondEncoder;
// // // End 2nd Encoder

// // //
// // // When the spinwheel is clicked and released, the following two functions will be run
// // //
// // void onSpinWheelClicked(uint8_t /*pin*/, bool heldDown) {
// //     Serial.print("Encoder button pressed ");
// //     Serial.println(heldDown ? "Held" : "Pressed");
// // }

// // void onSpinWheelButtonReleased(uint8_t /*pin*/, bool heldDown) {
// //     Serial.print("Encoder released - previously ");
// //     Serial.println(heldDown ? "Held" : "Pressed");
// // }

// // //
// // // When the repeat button is pressed, this listener class is notified. Every press and repeat is sent to
// // // onPressed while the release action is sent to onReleased. This method allows you to keep state between
// // // calls, and also to integrate the listener into an existing class if needed.
// // //
// // class RepeatButtonListener : public SwitchListener {
// // private:
// //     int counter;
// // public:
// //     void onPressed(pinid_t pin, bool held) override {
// //         ++counter;
// //         Serial.print("Repeat button pressed ");
// //         Serial.println(counter);
// //         ioDeviceDigitalWriteS(switches.getIoAbstraction(), ledPin, counter & 0x03);
// //     }

// //     void onReleased(pinid_t pin, bool held) override {
// //         Serial.print("Released after presses ");
// //         Serial.println(counter);
// //         counter = 0;
// //     }
// // } repeatListener;

// // //
// // // Each time the encoder value changes, this function runs, as we registered it as a callback
// // //
// // void onEncoderChange(int newValue) {
// //     Serial.print("Encoder change ");
// //     Serial.println(newValue);
// // }

// // void setup() {
// //     Serial.println("Starting interrupt switch PCF8574 example now");

// //     // Before doing anything else, we must initialise the wire and serial libraries, as we are using both.
// //     Serial.begin(115200);
// //     while(!Serial);
// //     Wire.begin();

// //     // First we set up the switches library, giving it the task manager and tell it where the pins are located
// //     // We could also of chosen IO through an i2c device that supports interrupts.
// //     // the second parameter is a flag to use pull up switching, (true is pull up).
// //     switches.initialiseInterrupt(ioFrom8574(0x27, 0), true );

// //     ioDevicePinMode(switches.getIoAbstraction(), ledPin, PULLDOWN);

// //     // now we add the switches, we dont want the spin wheel button to repeat, so leave off the last parameter
// //     // which is the repeat interval (millis / 20 basically) Repeat button does repeat as we can see.
// //     switches.addSwitch(spinWheelClickPin, onSpinWheelClicked);
// //     switches.onRelease(spinWheelClickPin, onSpinWheelButtonReleased);
// //     switches.addSwitchListener(repeatButtonPin, &repeatListener, 25);

// //     // now we set up the rotary encoder, first we give the A pin and the B pin.
// //     // we give the encoder a max value of 128, always minimum of 0.
// //     setupRotaryEncoderWithInterrupt(encoderAPin, encoderBPin, onEncoderChange);
// //     switches.changeEncoderPrecision(0, maximumEncoderValue, 100, true);

// //     // Start 2nd encoder
// //     Serial.println("Setting up second encoder now");

// //     // here we add a second encoder, you can comment out the below lines if you only want to use one encoder
// //     secondEncoder = new HardwareRotaryEncoder(encoder2APin, encoder2BPin, [](int direction) {
// //         Serial.print("Encoder direction: ");
// //         Serial.println(direction);
// //     });
// //     secondEncoder->changePrecision(-1, 1); // this means record direction changes only
// //     switches.setEncoder(1, secondEncoder); // put it into the 2nd available encoder slot.

// //     // Now add the 2nd encoders button for release notification only, you can use onRelease without first calling
// //     // addSwitch(..), and it will default everything, but you cannot invert logic this way.
// //     switches.onRelease(encoder2Click, [](pinid_t pin, bool held) {
// //         Serial.println("encoder 2 released");
// //     });
// //     // End 2nd encoder
// // }

// // void loop() {
// //     taskManager.runLoop();
   
// // }
// /*
// Arduino_I2C_Port_Expander v0.1.0 - beta

// This is a simple blink example. It blinks pin 13 on the slave board.

// For detailed instructions. Please visit this project on Github.
// https://github.com/jaretburkett/Arduino-I2C-Port-Expander

// Upload this code onto an Arduino Compatible microcontroller through
// the Arduino IDE. Connect to slave GND, VCC, SCL, SDA. Be sure to use 
// pullup resistors on the I2C lines. 

// Command examples for library assigned to io. with EXPAND io(0x01);
// change to fit your needs. 

// Commands:

// io.digitalWrite(pin, HIGH | LOW); - writes pin high or low
// io.digitalRead(pin); - Returns pin value as integer. 0 for low or 1 for high
// io.digitalReadPullup(pin); - Same as digital read, but activates the  internal pullup resistor first. 
// io.analogRead(pin); - Returns analog read val as int. Must call slaves digital pin number not A0. 
// io.analogWrite(pin, 0-255); - writes pwm to pin. Must be a pwm capable pin. 

// This code is released under a MIT license. 
// Created by Jaret Burkett
// */

// // include wire library first
// // #include <Wire.h>
// // // include port expander library second
// // #include <Arduino_I2C_Port_Expander.h>

// //  #include <SPI.h>

// // /* 	
// // Initialize the library class and name it "io". You can name it whatever
// // you want. For multiple port expanders, give each their own unique name. 
// // */
// // EXPAND io(0x01);  		//initialize an instance of the class with address 0x01
// // // EXPAND io2(0x25); 	// second port expander
// // int c = 0;
// // void setup()
// // {
// //   Wire.begin();
// //   Serial.begin(115200);  // start serial for output
// //   while(!Serial){}		// wait for serial port. Only needed on Leonardo based boards
// // }
 
// // void loop()
// // {
// // io.digitalWrite(13, HIGH);
// // delay(500);
// // io.digitalWrite(13, LOW);
// // delay(500);
// // c = io.digitalRead(53);

// // //Serial.println(c);

// // }
// /*
// I2C Test - Master
// Use with I2C_Slave.ino

// I2C communication between two Arduinos. The master will make a request from the slave.  The slave will respond.
// In this example, the slave will send 14 bytes of data.  I have a combination of bytes, integer, long unsigned integer and float data

// */
//  #include <SPI.h>
// #include <Wire.h>
// #include <EasyTransferI2C.h>

// //create object
// EasyTransferI2C ET;

// struct DATA_STRUCTURE{ //25 bytes total
//   float t1;       //4 bytes
//   float h1;       //4 bytes
//   float t2;       //4 bytes
//   float h2;       //4 bytes
//   uint8_t fanp;   //2 bytes
//   int rpm;        //2 bytes
//   int8_t mode;    //2 bytes
//   uint8_t color;  //2 bytes
//   bool door;      //1 byte
// };

// //give a name to the group of data
// DATA_STRUCTURE mydata;

// //define slave i2c address
// #define I2C_SLAVE_ADDRESS 9
// #define I2C_MASTER_ADDRESS 8

// //define some initial vars
// float t1;             // Printer/Enclosure temperature
// float h1;             // Printer/Enclosure humidity
// float t2;             // Filament Temperature
// float h2;             // Filament room humidity
// int rpm;             // RPM measurement from fan
// uint8_t fanp;         // Fan power (0-100)
// int8_t mode;    // LED mode (0=Rainbow,1=Solid,2=Temp,3=Breath,4=Heatbeat,5=Alarm)
// uint8_t color;  // LED Color (0=Black,1=Purple,2=Blue,3=Green,4=Red,5=Orange,6=Yellow,7=White)
// bool door;      // door state
// void receiveData(int numBytes) {}

// void sendData() {
//   Serial.println("Sending data!");
//   Serial.print("Printer/Enclosure temperature: ");
//   Serial.println(mydata.t1);
//   Serial.print("Printer/Enclosure humidity: ");
//   Serial.println(mydata.h1);
//   Serial.print("Filament temperature: ");
//   Serial.println(mydata.t2);
//   Serial.print("Filament humidity: ");
//   Serial.println(mydata.h2);
//   Serial.print("Fan power: ");
//   Serial.println(mydata.fanp);
//   Serial.print("Fan RPM: ");
//   Serial.println(mydata.rpm);
//   Serial.print("LED mode: ");
//   Serial.println(mydata.mode);
//   Serial.print("LED color: ");
//   Serial.println(mydata.color);
//   Serial.print("Door state: ");
//   Serial.println(mydata.door);
//   ET.sendData(I2C_MASTER_ADDRESS);
//   Serial.println("Done transmitting!");
// }

// void setup(){
//   Serial.begin(115200);
//   Wire.begin(I2C_SLAVE_ADDRESS);
//   //start the library, pass in the data details and the name of the serial port. Can be Serial, Serial1, Serial2, etc.
//   ET.begin(details(mydata), &Wire);
//   //define handler function on receiving data
//   //Wire.onReceive(receiveData);
// }

// void loop() {
//   Serial.println("starting loop");
//   mydata.t1 = random(100);       // Printer/Enclosure temperature
//   mydata.h1 = random(100);       // Printer/Enclosure humidity
//   mydata.t2 = random(100);       // Filament Temperature
//   mydata.h2 = random(100);       // Filament room humidity
//   mydata.rpm = random(1000);     // RPM measurement from fan
//   mydata.fanp  = random(100);  // Fan power (0-100)
//   mydata.mode  = random(5);  // LED mode (0=Rainbow,1=Solid,2=Temp,3=Breath,4=Heatbeat,5=Alarm)
//   mydata.color = random(7); // LED Color (0=Black,1=Purple,2=Blue,3=Green,4=Red,5=Orange,6=Yellow,7=White)
//   mydata.door = random(1);   // door state
//   Serial.println("done setting vars");
//   delay(3000);
//   Serial.println("Sending data...");
//   sendData();
//   Serial.println("Data sent...");
//   delay(2000);
// }

// Wire Slave Sender
// by Nicholas Zambetti <http://www.zambetti.com>

// Demonstrates use of the Wire library
// Sends data as an I2C/TWI slave device
// Refer to the "Wire Master Reader" example for use with this

// Created 29 March 2006

// This example code is in the public domain.


