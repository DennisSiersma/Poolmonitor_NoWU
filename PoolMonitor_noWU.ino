/**The MIT License (MIT)
  Copyright (c) 2015 by Daniel Eichhorn
  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:
  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.
  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYBR_DATUM HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
  See more at http://blog.squix.ch

  Adapted by Bodmer to use the faster TFT_ILI9341_ESP library:
  https://github.com/Bodmer/TFT_ILI9341_ESP

  Parts of PH and EZO code taken from WhiteBox Labs -- Tentacle Shield -- examples

  Adapted by DJS to be used as pool monitor 2017, including PH and ORP sensors
  and water temperature.
*/

/***********************************************************************************************
 * definitions and includes                                                                                *     
 ***********************************************************************************************/
//#define BLYNK_PRINT Serial
#include <Arduino.h>

#include <SPI.h>
#include <TFT_ILI9341_ESP.h>                    // Hardware-specific library
#include <Wire.h>                               // enable I2C.
#include <OneWire.h>                            // enable OneWire
#include <DallasTemperature.h>                  // Helper for watertemp sensor
#include <Ticker.h>                             // Timer for watchdog                      

// Additional UI functions
#include "GfxUi.h"


#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>

// check settings.h for adapting to your needs
#include "settings.h"
#include <BlynkSimpleEsp8266.h>
#include <TimeLib.h>
#include <WidgetRTC.h>
// Helps with connecting to internet
#include <WiFiManager.h>

/***********************************************************************************************
 * Blynk                                                                                       *     
 ***********************************************************************************************/


BlynkTimer timer;
WidgetRTC rtc;
WidgetTerminal terminal(V10);

//BLYNK_CONNECTED() {
  // Synchronize time on connection
//  rtc.begin();
//}

char Date[16];
char Time[16];

String currentTime = String(hour()) + ":" + minute() + ":" + second();
String currentDate = String(day()) + " " + month() + " " + year();

//sprintf(Date, "%04d/%02d/%02d", year(), month(), day());
//sprintf(Time, "%02d:%02d:%02d", hour(), minute(), second());

int cmdCalORP = 0;
int cmdCalPH7 = 0;
int cmdCalPH4 = 0;
int cmdCalPH10 = 0;
int cmdCalCORP = 0;
int cmdCalCPH = 0;
int cmdTempComp = 0;

/***********************************************************************************************
 * EZO stuff                                                                                   *     
 ***********************************************************************************************/
#define TOTAL_CIRCUITS 2                            // <-- CHANGE THIS | set how many I2C circuits are attached to the Tentacle

const unsigned int baud_host  = 9600;               // set baud rate for host serial monitor(pc/mac/other)
const unsigned int send_readings_every = 50000;     // set at what intervals the readings are sent to the computer (NOTE: this is not the frequency of taking the readings!)
unsigned long next_serial_time;

char sensordata[30];                                // A 30 byte character array to hold incoming data from the sensors
byte sensor_bytes_received = 0;                     // We need to know how many characters bytes have been received

byte code = 0;                                      // used to hold the I2C response code.
byte in_char = 0;                                   // used as a 1 byte buffer to store in bound bytes from the I2C Circuit.

int channel_ids[] = {98, 99};                       // <-- CHANGE THIS. A list of I2C ids that you set your circuits to.
char *channel_names[] = {"ORP", "PH"};              // <-- CHANGE THIS. A list of channel names (must be the same order as in channel_ids[]) - only used to designate the readings in serial communications

String readings[TOTAL_CIRCUITS];                    // an array of strings to hold the readings of each channel
String TEMP_val = "Hold";
String PH_val = "one";
String ORP_val = "sec";

char command_string[20];       // holds command to be send to probe
char ScmdCalORP[] = "Cal,225";
char ScmdCalPH7[] = "Cal,mid,7.00";
char ScmdCalPH4[] = "Cal,low,4.00";
char ScmdCalPH10[] = "Cal,high,10.00";
char ScmdCalCORP[] = "Cal,clear";
char ScmdCalCPH[] = "Cal,clear";  
char ScmdTempComp[] = "T,";  
byte cs_lenght;                               // counter for char lenght

int channel = 0;                              // INT pointer to hold the current position in the channel_ids/channel_names array

const unsigned int reading_delay = 1400;      // time to wait for the circuit to process a read command. datasheets say 1 second.
unsigned long next_reading_time;              // holds the time when the next reading should be ready from the circuit
boolean request_pending = false;              // wether or not we're waiting for a reading

const unsigned int blink_frequency = 250;     // the frequency of the led blinking, in milliseconds
unsigned long next_blink_time;                // holds the next time the led should change state
boolean led_state = LOW;                      // keeps track of the current led state


/***********************************************************************************************
 * Important: see settings.h to configure your settings!!!                                     *                                                                           *     
 ***********************************************************************************************/
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

// HOSTNAME for OTA update
#define HOSTNAME "ESP8266-OTA-"
TFT_ILI9341_ESP tft = TFT_ILI9341_ESP();       // Invoke custom library

boolean booted = true;

GfxUi ui = GfxUi(&tft);

/***********************************************************************************************
 * Declaring prototypes                                                                        *     
 ***********************************************************************************************/
void configModeCallback (WiFiManager *myWiFiManager);
//void drawProgress(uint8_t percentage, String text);
//void drawTime();

void drawEZO();
void requestTemp();
void drawSeparator(uint16_t y);
void receive_reading();
void request_reading();
void do_sensor_readings();
void do_serial();
long lastDownloadUpdate = millis();
long lastTempTime = millis();

/***********************************************************************************************
 * Serial interupt    (debugging)                                                              *     
 ***********************************************************************************************/
/*void Sent_serial() {
       // Sent serial data to Blynk terminal - Unlimited string readed
       String content = "";  //null string constant ( an empty string )
       char character;
       while(Serial.available()) {
            character = Serial.read();
            content.concat(character);
              
       }
       if (content != "") {
            Blynk.virtualWrite (V10, content);
       }  
}
*/
/***********************************************************************************************
 * Watchdog                                                                                    *     
 ***********************************************************************************************/
Ticker tickerOSWatch;

#define OSWATCH_RESET_TIME 60

static unsigned long last_loop;

void ICACHE_RAM_ATTR osWatch(void) {
    unsigned long t = millis();
    unsigned long last_run = abs(t - last_loop);
    if(last_run >= (OSWATCH_RESET_TIME * 1000)) {
      // save the hit here to eeprom or to rtc memory if needed
        //ESP.restart();  // normal reboot 
        terminal.println(F("------ RESET -------"));
        ESP.reset();  // hard reset
    }
}
/***********************************************************************************************
 * Clockdisplay (Blynk RTC test)                                                                *     
 ***********************************************************************************************/

void requestTime() {
  Blynk.sendInternal("rtc", "sync");
  
  // You can call hour(), minute(), ... at any time
  // Please see Time library examples for details

  currentTime = String(hour()) + ":" + minute() + ":" + second();
  currentDate = String(day()) + " " + month() + " " + year();
  Serial.print("Current time: ");
  Serial.print(currentTime);
  Serial.print(" ");
  Serial.print(currentDate);
  Serial.println();
}
/***********************************************************************************************
 * Setup                                                                                       *     
 ***********************************************************************************************/
void setup() {
  last_loop = millis();                                                       // watchdog loop count
  tickerOSWatch.attach_ms(((OSWATCH_RESET_TIME / 3) * 1000), osWatch);        // watchdog object
  //timer.setInterval(100, Sent_serial);                                      // set for debugging, setup serial interupt too.
  setSyncInterval(10*60); // Sync interval in seconds (10 minutes)

  // Display digital clock every 10 seconds
  timer.setInterval(10000L, requestTime);
  
  Serial.begin(baud_host);
  pinMode(13, OUTPUT);                                                        // set the led output pin

  Wire.begin();                                                               // enable I2C port.
  sensors.begin();                                                            // IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement
  next_serial_time = millis() + send_readings_every;                          // calculate the next point in time we should do serial communications

  tft.begin();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);

  //tft.setFreeFont(&ArialRoundedMTBold_14);
  tft.setTextFont(4);
  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(TFT_DARKGREY, TFT_BLACK);
  tft.drawString("Gebouwd door DJS 2017", 120, 240);
  tft.drawString("Groeten Thom & Chris", 120, 260);
  delay(500);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);

    //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;
  // Uncomment for testing wifi manager
  //wifiManager.resetSettings();
  //wifiManager.setAPCallback(configModeCallback);

  //or use this for auto generated name ESP + ChipID
  // wifiManager.autoConnect();

  //Manual Wifi
  //WiFi.begin(WIFI_SSID, WIFI_PWD);

  /***********************************************************************************************
 * Blynk stuff       *                                                                           *     
 *************************************************************************************************/
  Blynk.config(auth);
  Blynk.connect();
  //WiFi.printDiag(Serial);
  //Blynk.begin(auth, ssid, pass);
  
  while(Blynk.connect() == false);
  rtc.begin();
  
  WidgetTerminal terminal(V10);
  Blynk.virtualWrite(V10, "\n\n");
  Blynk.virtualWrite(V10, "    ___  __          __ \n");
  Blynk.virtualWrite(V10, "   / _ )/ /_ _____  / /__ \n");
  Blynk.virtualWrite(V10, "  / _  / / // / _ \\/  '_/ \n");  // to display a backslash, print it twice
  Blynk.virtualWrite(V10, " /____/_/\\_, /_//_/_/\\_\\ \n");  // to display a backslash, print it twice
  Blynk.virtualWrite(V10, "        /___/ v");
  Blynk.virtualWrite(V10, BLYNK_VERSION" on "BLYNK_INFO_DEVICE"\n\n");
  Blynk.virtualWrite(V10, "                 Project By DJS -\n");
  Blynk.virtualWrite(V10, "                 ... Starting!\n");
  terminal.flush();
  
  // OTA Setup
  String hostname(HOSTNAME);
  hostname += String(ESP.getChipId(), HEX);
  WiFi.hostname(hostname);
  ArduinoOTA.setHostname((const char *)hostname.c_str());
  ArduinoOTA.begin();

  tft.drawString("Verbinden met WiFi", 120, 200);
  tft.setTextPadding(240);                                                    // Pad next drawString() text to full width to over-write old text
  
 }

long lastDrew = 0;

  
/***********************************************************************************************
 * Main loop                                                                                   *     
 *************************************************************************************************/

void loop() {
  // Kick the watchdog
  last_loop=millis(); 
  // Handle OTA update requests
  ArduinoOTA.handle();

  // Check if we should update the clock
  //if (millis() - lastDrew > 30000 && rtc.getSeconds() == "00") {
  //  drawTime();
  //  lastDrew = millis();
  //}


  // Check if we should update weather information
  //if (millis() - lastDownloadUpdate > 1000 * UPDATE_INTERVAL_SECS) {
  //  updateData();
  //  lastDownloadUpdate = millis();
  //}

  // EZO update
  do_sensor_readings();
  do_serial();

  // temp sensor update
  if (millis() - lastTempTime > 1000 * UPDATE_TEMP_SECS) {
    requestTemp();
    lastTempTime = millis();
  }
  Blynk.run();
  timer.run(); // Initiates BlynkTimer
}


// Called if WiFi has not been configured yet
void configModeCallback (WiFiManager *myWiFiManager) {
tft.setTextDatum(BC_DATUM);
//tft.setFreeFont(&ArialRoundedMTBold_14);
tft.setTextFont(4);
tft.setTextColor(TFT_ORANGE);
tft.drawString("Wifi Manager", 120, 28);
tft.drawString("Please connect to AP", 120, 42);
tft.setTextColor(TFT_WHITE);
tft.drawString(myWiFiManager->getConfigPortalSSID(), 120, 56);
tft.setTextColor(TFT_ORANGE);
tft.drawString("To setup Wifi Configuration", 120, 70);
}


// Update the internet based information and update screen
void updateData() {
  
  drawTime();
  drawEZO();

  //if (booted) screenshotToConsole(); // Documentation support only!
  booted = false;
}

// Progress bar helper
void drawProgress(uint8_t percentage, String text) {
  //tft.setFreeFont(&ArialRoundedMTBold_14);
  tft.setTextFont(4);
  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.setTextPadding(240);
  tft.drawString(text, 120, 220);

  ui.drawProgressBar(10, 225, 240 - 20, 15, percentage, TFT_WHITE, TFT_BLUE);

  tft.setTextPadding(0);
}

// draws the clock
void drawTime() {
  //tft.setFreeFont(&ArialRoundedMTBold_14);
  tft.setTextFont(4);
  
  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth(" Ddd, 44 Mmm 4444 "));  // String width + margin
  tft.drawString(currentDate, 120, 14);

  //tft.setFreeFont(&ArialRoundedMTBold_36);
  tft.setTextFont(6);
  
  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextPadding(tft.textWidth(" 44:44 "));  // String width + margin
  tft.drawString(currentTime, 120, 50);

  drawSeparator(52);
  drawSeparator(153);
  tft.setTextPadding(0);
}



// draw sensordata and time
void drawEZO() {
  tft.setTextFont(4);
  tft.setTextSize(2);           // We are using a size multiplier of 1
  tft.setCursor(30, 10);    // Set cursor to x = 30, y = 175
  tft.setTextColor(TFT_WHITE, TFT_BLACK);  // Set text colour to white and background to black
  tft.println(currentTime);
  
  //Title
  //tft.setFreeFont(&ArialRoundedMTBold_36);
  tft.setTextFont(4);
  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(TFT_BLUE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("Pool Monitor"));
  tft.drawString("Pool Monitor", 120, 200 - 2);

  //TEMP
  tft.setTextDatum(BR_DATUM);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.setTextPadding(0); // Reset padding width to none
  tft.drawString("Temp ", 0, 240);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("-88.00`"));
  //if (TEMP_val.indexOf(".")) TEMP_val = TEMP_val.substring(0, TEMP_val.indexOf(".") + 1); // Make it .1 precision
  if (TEMP_val == "") TEMP_val = "?";  // Handle null return
  tft.drawString(TEMP_val + "`", 221, 240);
  tft.setTextDatum(BL_DATUM);
  tft.setTextPadding(0);
  //tft.setFreeFont(&ArialRoundedMTBold_14);
  tft.setTextFont(4);
  tft.drawString("C ", 221, 220);
  Blynk.virtualWrite (V1, TEMP_val);
  //tft.setTextDatum(MR_DATUM);
  //tft.setTextColor(TFT_WHITE, TFT_BLACK);
  //tft.setTextPadding(tft.textWidth("test"));
  //tft.drawString("test", 120, 240);

  //PH
  //tft.setFreeFont(&ArialRoundedMTBold_36);
  tft.setTextFont(4);
  tft.setTextDatum(BR_DATUM);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.setTextPadding(0); // Reset padding width to none
  tft.drawString("PH", 0, 280);
  tft.setTextDatum(BL_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth(" 3.777 "));
  tft.drawString(PH_val, 221, 280);
  Blynk.virtualWrite (V2, PH_val);

  //ORP
  tft.setTextDatum(BR_DATUM);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.setTextPadding(0); // Reset padding width to none
  tft.drawString("ORP", 0, 315);
  //tft.setTextDatum(BR_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("200.3"));
  tft.drawString(ORP_val, 220, 315);
  tft.setTextDatum(BL_DATUM);
  //tft.setTextPadding(0);
  //tft.setFreeFont(&ArialRoundedMTBold_14);
  tft.setTextFont(4);
  tft.drawString("mV", 221, 310);
  Blynk.virtualWrite (V3, ORP_val);
  //Cleanup for next string
  tft.setTextPadding(0); // Reset padding width to none
}



// if you want separators, uncomment the tft-line
void drawSeparator(uint16_t y) {
  tft.drawFastHLine(10, y, 240 - 2 * 10, 0x4228);
}

// determine the "space" split point in a long string
int splitIndex(String text)
{
  int index = 0;
  while ( (text.indexOf(' ', index) >= 0) && ( index <= text.length() / 2 ) ) {
    index = text.indexOf(' ', index) + 1;
  }
  if (index) index--;
  return index;
}

// Calculate coord delta from start of text String to start of sub String contained within that text
// Can be used to vertically right align text so for example a colon ":" in the time value is always
// plotted at same point on the screen irrespective of different proportional character widths,
// could also be used to align decimal points for neat formatting
int rightOffset(String text, String sub)
{
  int index = text.indexOf(sub);
  return tft.textWidth(text.substring(index));
}

// Calculate coord delta from start of text String to start of sub String contained within that text
// Can be used to vertically left align text so for example a colon ":" in the time value is always
// plotted at same point on the screen irrespective of different proportional character widths,
// could also be used to align decimal points for neat formatting
int leftOffset(String text, String sub)
{
  int index = text.indexOf(sub);
  return tft.textWidth(text.substring(0, index));
}


// blinks a led on pin 13 asynchronously
void blink_led() {
  if (millis() >= next_blink_time) {                  // is it time for the blink already?
    led_state = !led_state;                           // toggle led state on/off
    digitalWrite(13, led_state);                      // write the led state to the led pin
    next_blink_time = millis() + blink_frequency;     // calculate the next time a blink is due
  }
}

/***********************************************************************************************
 * Atlas Scientific EZO code                                                                                     *     
 ***********************************************************************************************/
// do serial communication in a "asynchronous" way

void do_serial() {
  if (millis() >= next_serial_time) {                // is it time for the next serial communication?
    for (int i = 0; i < TOTAL_CIRCUITS; i++) {       // loop through all the sensors
      Serial.print(channel_names[i]);                // print channel name
      Serial.print(":\t");
      Serial.println(readings[i]);                    // print the actual reading
      //Serial.println(i);
      PH_val = readings[1];
      ORP_val = readings[0];
      drawEZO();
      terminal.println(PH_val + "\n" + ORP_val + "\n");
      Serial.println(PH_val + " " + ORP_val);
    }
    next_serial_time = millis() + send_readings_every;
  }
}


// take sensor readings in a "asynchronous" way
void do_sensor_readings() {
  if (request_pending) {                          // is a request pending?
    if (millis() >= next_reading_time) {          // is it time for the reading to be taken?
      receive_reading();                          // do the actual I2C communication
    }
  } else {                                        // no request is pending,
    channel = (channel + 1) % TOTAL_CIRCUITS;     // switch to the next channel (increase current channel by 1, and roll over if we're at the last channel using the % modulo operator)
    request_reading();                            // do the actual I2C communication
  }
}



// Request a reading from the current channel
void request_reading() {
  request_pending = true;
  Wire.beginTransmission(channel_ids[channel]); // call the circuit by its ID number.
  Wire.write('r');                    // request a reading by sending 'r'
  Wire.endTransmission();                   // end the I2C data transmission.
  next_reading_time = millis() + reading_delay; // calculate the next time to request a reading
}

void send_command() {
  request_pending = true;
  terminal.println(command_string);
  terminal.flush();
  Wire.beginTransmission(channel_ids[channel]);  // call the circuit by its ID number.
  Wire.write(command_string);                               // request a reading by sending command
  Wire.endTransmission();                        // end the I2C data transmission.
  next_reading_time = millis() + reading_delay;  // calculate the next time to request a reading
  terminal.println("Sending to probe, wait for reply\n");
  Blynk.run();
  delay(1000);
  Blynk.run();
  delay(1000);
  Blynk.run();
  terminal.println("Requesting reply\n");
  sensor_bytes_received = 0;                        // reset data counter
  memset(sensordata, 0, sizeof(sensordata));        // clear sensordata array;

  Wire.requestFrom(channel_ids[channel], 48, 1);    // call the circuit and request 48 bytes (this is more then we need).
  //Wire.requestFrom(99, 48, 1);    // call the circuit and request 48 bytes (this is more then we need).
  code = Wire.read();

  while (Wire.available()) {          // are there bytes to receive?
    in_char = Wire.read();            // receive a byte.

    if (in_char == 0) {               // if we see that we have been sent a null command.
      Wire.endTransmission();         // end the I2C data transmission.
      break;                          // exit the while loop, we're done here
    }
    else {
      sensordata[sensor_bytes_received] = in_char;  // load this byte into our array.
      sensor_bytes_received++;
    }
  }

  switch (code) {                       // switch case based on what the response code is.
    case 1:                             // decimal 1  means the command was successful.
      terminal.println("OK\n");
      terminal.flush();
      break;                              // exits the switch case.

    case 2:                             // decimal 2 means the command has failed.
      terminal.println("error: command failed\n");
      terminal.flush();
      break;                              // exits the switch case.

    case 254:                           // decimal 254  means the command has not yet been finished calculating.
      terminal.println("reading not ready\n");
      terminal.flush();
      break;                              // exits the switch case.

    case 255:                           // decimal 255 means there is no further data to send.
      terminal.println("error: no data\n");
      terminal.flush();
      break;                              // exits the switch case.
  }
  terminal.println("Continuing...\n");
  terminal.flush();
  request_pending = false;                  // set pending to false, so we can continue to the next sensor
}


// Receive data from the I2C bus
void receive_reading() {
  sensor_bytes_received = 0;                        // reset data counter
  memset(sensordata, 0, sizeof(sensordata));        // clear sensordata array;

  Wire.requestFrom(channel_ids[channel], 48, 1);    // call the circuit and request 48 bytes (this is more then we need).
  code = Wire.read();

  while (Wire.available()) {          // are there bytes to receive?
    in_char = Wire.read();            // receive a byte.

    if (in_char == 0) {               // if we see that we have been sent a null command.
      Wire.endTransmission();         // end the I2C data transmission.
      break;                          // exit the while loop, we're done here
    }
    else {
      sensordata[sensor_bytes_received] = in_char;  // load this byte into our array.
      sensor_bytes_received++;
    }
  }

  switch (code) {                       // switch case based on what the response code is.
    case 1:                             // decimal 1  means the command was successful.
      readings[channel] = sensordata;
      break;                              // exits the switch case.

    case 2:                             // decimal 2 means the command has failed.
      readings[channel] = "error: command failed";
      break;                              // exits the switch case.

    case 254:                           // decimal 254  means the command has not yet been finished calculating.
      readings[channel] = "reading not ready";
      break;                              // exits the switch case.

    case 255:                           // decimal 255 means there is no further data to send.
      readings[channel] = "error: no data";
      break;                              // exits the switch case.
  }
  //Blynk.virtualWrite (V10, readings[channel]);
  request_pending = false;                  // set pending to false, so we can continue to the next sensor
}


/***********************************************************************************************
 * Temperature                                                                                      *     
 ***********************************************************************************************/
//Receive data from OneWire sensor
void requestTemp()  {
  // call sensors.requestTemperatures() to issue a global temperature
  // request to all devices on the bus

  Serial.print("Requesting temperature(s)...");
  sensors.requestTemperatures(); // Send the command to get temperatures
  Serial.println("DONE");
  Serial.print("Temperature for Device 1 is: ");
  Serial.print(sensors.getTempCByIndex(0)); // Why "byIndex"? You can have more than one IC on the same bus. 0 refers to the first IC on the wire
  Serial.println();
  terminal.print(sensors.getTempCByIndex(0));
  terminal.print("\n");
  TEMP_val = sensors.getTempCByIndex(0);
}

/***********************************************************************************************
 * Blynk                                                                                       *     
 ***********************************************************************************************/

 
BLYNK_WRITE(V11){
   cmdCalORP = param.asInt(); // Get the state of the VButton
   if (cmdCalORP == 1) {
    channel = 0;
    strcpy(command_string, ScmdCalORP);
    send_command();
   }
}
BLYNK_WRITE(V12){
    cmdCalPH7 = param.asInt(); // Get the state of the VButton
    if (cmdCalPH7 == 1) {
    channel = 1;
    strcpy(command_string, ScmdCalPH7);
    send_command();  
      }
} 
BLYNK_WRITE(V13){
    cmdCalPH4 = param.asInt(); // Get the state of the VButton
    if (cmdCalPH4 == 1) {
    channel = 1;
    strcpy(command_string, ScmdCalPH4);
    send_command();  
      }
} 

BLYNK_WRITE(V15){
    cmdCalCORP = param.asInt(); // Get the state of the VButton
    if (cmdCalCORP == 1) {
    channel = 1;
    strcpy(command_string, ScmdCalCORP);
    send_command();  
      }
} 
BLYNK_WRITE(V16){
    cmdCalCPH = param.asInt(); // Get the state of the VButton
    if (cmdCalCPH == 1) {
    channel = 1;
    strcpy(command_string, ScmdCalCPH);
    send_command();  
      }
} 
BLYNK_WRITE(V17){
    cmdTempComp = param.asInt(); // Get the state of the VButton
    if (cmdTempComp == 1) {
    channel = 1;
    strcat( ScmdTempComp, TEMP_val.c_str() );
    strcpy(command_string, ScmdTempComp);
    terminal.print("Sending temp to compensate: ");
    terminal.print(TEMP_val);
    terminal.print("\n");
    send_command();  
      }
} 
BLYNK_WRITE(InternalPinRTC) {
  long t = param.asLong();
  Serial.print("Unix time: ");
  Serial.print(t);
  Serial.println();
}
