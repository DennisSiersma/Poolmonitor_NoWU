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

// Fonts created by http://oleddisplay.squix.ch/
#include "ArialRoundedMTBold_14.h"
#include "ArialRoundedMTBold_36.h"
// Download helper
#include "WebResource.h"

#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>

// check settings.h for adapting to your needs
#include "settings.h"
#include <BlynkSimpleEsp8266.h>
// Helps with connecting to internet
#include <WiFiManager.h>
#include <JsonListener.h>
#include <WundergroundClient.h>
#include "TimeClient.h"

/***********************************************************************************************
 * Blynk                                                                                       *     
 ***********************************************************************************************/


BlynkTimer timer;
WidgetTerminal terminal(V10);

int cmdCalORP = 0;
int cmdCalPH7 = 0;
int cmdCalPH4 = 0;
int cmdCalPH10 = 0;
int cmdCalCORP = 0;
int cmdCalCPH = 0;
int cmdTempComp = 0;

//String inputString = "";
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

WebResource webResource;
TimeClient timeClient(UTC_OFFSET);

// Set to false, if you prefere imperial/inches, Fahrenheit
WundergroundClient wunderground(IS_METRIC);

/***********************************************************************************************
 * Declaring prototypes                                                                        *     
 ***********************************************************************************************/
void configModeCallback (WiFiManager *myWiFiManager);
void downloadCallback(String filename, int16_t bytesDownloaded, int16_t bytesTotal);
ProgressCallback _downloadCallback = downloadCallback;
void downloadResources();
void updateData();
void drawProgress(uint8_t percentage, String text);
void drawTime();
void drawCurrentWeather();
//void drawForecast();
//void drawForecastDetail(uint16_t x, uint16_t y, uint8_t dayIndex);
String getMeteoconIcon(String iconText);
void drawEZO();
void requestTemp();
void drawSeparator(uint16_t y);

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
 * Setup                                                                                       *     
 ***********************************************************************************************/
void setup() {
  last_loop = millis();                                                       // watchdog loop count
  tickerOSWatch.attach_ms(((OSWATCH_RESET_TIME / 3) * 1000), osWatch);        // watchdog object
  //timer.setInterval(100, Sent_serial);

  Serial.begin(baud_host);
  pinMode(13, OUTPUT);                                                        // set the led output pin

  Wire.begin();                                                               // enable I2C port.
  sensors.begin();                                                            // IC Default 9 bit. If you have troubles consider upping it 12. Ups the delay giving the IC more time to process the temperature measurement
  next_serial_time = millis() + send_readings_every;                          // calculate the next point in time we should do serial communications

  tft.begin();
  tft.setRotation(2);
  tft.fillScreen(TFT_BLACK);

  tft.setFreeFont(&ArialRoundedMTBold_14);
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

   SPIFFS.begin();
  if (SPIFFS.exists("/WU.jpg") == true) ui.drawJpeg("/WU.jpg", 0, 10);
  if (SPIFFS.exists("/Earth.jpg") == true) ui.drawJpeg("/Earth.jpg", 0, 320 - 56); // Image is 56 pixels high
  delay(1000);
  tft.drawString("Verbinden met WiFi", 120, 200);
  tft.setTextPadding(240);                                                    // Pad next drawString() text to full width to over-write old text
  // download images from the net. If images already exist don't download
  tft.drawString("Downloaden naar SPIFFS...", 120, 200);
  tft.drawString(" ", 120, 240);  // Clear line
  tft.drawString(" ", 120, 260);  // Clear line
  downloadResources();
  //listFiles();
  tft.drawString(" ", 120, 200);  // Clear line above using set padding width
  tft.drawString("Ophalen weer data...", 120, 220);
  //delay(500);

  // load the weather information
  updateData();
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
  if (millis() - lastDrew > 30000 && wunderground.getSeconds() == "00") {
    drawTime();
    lastDrew = millis();
  }


  // Check if we should update weather information
  if (millis() - lastDownloadUpdate > 1000 * UPDATE_INTERVAL_SECS) {
    updateData();
    lastDownloadUpdate = millis();
  }

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
tft.setFreeFont(&ArialRoundedMTBold_14);
tft.setTextColor(TFT_ORANGE);
tft.drawString("Wifi Manager", 120, 28);
tft.drawString("Please connect to AP", 120, 42);
tft.setTextColor(TFT_WHITE);
tft.drawString(myWiFiManager->getConfigPortalSSID(), 120, 56);
tft.setTextColor(TFT_ORANGE);
tft.drawString("To setup Wifi Configuration", 120, 70);
}

// callback called during download of files. Updates progress bar
void downloadCallback(String filename, int16_t bytesDownloaded, int16_t bytesTotal) {
  Serial.println(String(bytesDownloaded) + " / " + String(bytesTotal));

  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.setTextPadding(240);

  int percentage = 100 * bytesDownloaded / bytesTotal;
  if (percentage == 0) {
    tft.drawString(filename, 120, 220);
  }
  if (percentage % 5 == 0) {
    tft.setTextDatum(TC_DATUM);
    tft.setTextPadding(tft.textWidth(" 888% "));
    tft.drawString(String(percentage) + "%", 120, 245);
    ui.drawProgressBar(10, 225, 240 - 20, 15, percentage, TFT_WHITE, TFT_BLUE);
  }

}

// Download the bitmaps
void downloadResources() {
  // tft.fillScreen(TFT_BLACK);
  tft.setFreeFont(&ArialRoundedMTBold_14);
  char id[5];

  // Download WU graphic jpeg first and display it, then the Earth view
  webResource.downloadFile((String)"http://i.imgur.com/njl1pMj.jpg", (String)"/WU.jpg", _downloadCallback);
  if (SPIFFS.exists("/WU.jpg") == true) ui.drawJpeg("/WU.jpg", 0, 10);

  webResource.downloadFile((String)"http://i.imgur.com/v4eTLCC.jpg", (String)"/Earth.jpg", _downloadCallback);
  if (SPIFFS.exists("/Earth.jpg") == true) ui.drawJpeg("/Earth.jpg", 0, 320 - 56);

  //webResource.downloadFile((String)"http://i.imgur.com/IY57GSv.jpg", (String)"/Horizon.jpg", _downloadCallback);
  //if (SPIFFS.exists("/Horizon.jpg") == true) ui.drawJpeg("/Horizon.jpg", 0, 320-160);

  //webResource.downloadFile((String)"http://i.imgur.com/jZptbtY.jpg", (String)"/Rainbow.jpg", _downloadCallback);
  //if (SPIFFS.exists("/Rainbow.jpg") == true) ui.drawJpeg("/Rainbow.jpg", 0, 0);

  for (int i = 0; i < 19; i++) {
    sprintf(id, "%02d", i);
    webResource.downloadFile("http://www.squix.org/blog/wunderground/" + wundergroundIcons[i] + ".bmp", wundergroundIcons[i] + ".bmp", _downloadCallback);
  }
  for (int i = 0; i < 19; i++) {
    sprintf(id, "%02d", i);
    webResource.downloadFile("http://www.squix.org/blog/wunderground/mini/" + wundergroundIcons[i] + ".bmp", "/mini/" + wundergroundIcons[i] + ".bmp", _downloadCallback);
  }
  for (int i = 0; i < 24; i++) {
    webResource.downloadFile("http://www.squix.org/blog/moonphase_L" + String(i) + ".bmp", "/moon" + String(i) + ".bmp", _downloadCallback);
  }
}

// Update the internet based information and update screen
void updateData() {
  // booted = true;  // Test only
  // booted = false; // Test only

  if (booted) ui.drawJpeg("/WU.jpg", 0, 10); // May have already drawn this but it does not take long
  else tft.drawCircle(22, 22, 16, TFT_DARKGREY); // Outer ring - optional

  if (booted) drawProgress(20, "Updaten tijd...");
  else fillSegment(22, 22, 0, (int) (20 * 3.6), 16, TFT_NAVY);

  timeClient.updateTime();
  if (booted) drawProgress(50, "Updaten omstandigheden...");
  else fillSegment(22, 22, 0, (int) (50 * 3.6), 16, TFT_NAVY);

  wunderground.updateConditions(WUNDERGRROUND_API_KEY, WUNDERGRROUND_LANGUAGE, WUNDERGROUND_COUNTRY, WUNDERGROUND_CITY);
  if (booted) drawProgress(70, "Updaten voorspellingen...");
  else fillSegment(22, 22, 0, (int) (70 * 3.6), 16, TFT_NAVY);

  wunderground.updateForecast(WUNDERGRROUND_API_KEY, WUNDERGRROUND_LANGUAGE, WUNDERGROUND_COUNTRY, WUNDERGROUND_CITY);
  if (booted) drawProgress(90, "Initialiseren EZO sensoren...");
  else fillSegment(22, 22, 0, (int) (90 * 3.6), 16, TFT_NAVY);

  wunderground.updateAstronomy(WUNDERGRROUND_API_KEY, WUNDERGRROUND_LANGUAGE, WUNDERGROUND_COUNTRY, WUNDERGROUND_CITY);
  // lastUpdate = timeClient.getFormattedTime();
  // readyForWeatherUpdate = false;
  if (booted) drawProgress(100, "Klaar...");
  else fillSegment(22, 22, 0, 360, 16, TFT_NAVY);

  if (booted) delay(2000);

  if (booted) tft.fillScreen(TFT_BLACK);
  else   fillSegment(22, 22, 0, 360, 22, TFT_BLACK);

  //tft.fillScreen(TFT_CYAN); // For text padding and update graphics over-write checking only
  drawTime();
  drawCurrentWeather();
  //drawForecast();
  drawEZO();

  //if (booted) screenshotToConsole(); // Documentation support only!
  booted = false;
}

// Progress bar helper
void drawProgress(uint8_t percentage, String text) {
  tft.setFreeFont(&ArialRoundedMTBold_14);

  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.setTextPadding(240);
  tft.drawString(text, 120, 220);

  ui.drawProgressBar(10, 225, 240 - 20, 15, percentage, TFT_WHITE, TFT_BLUE);

  tft.setTextPadding(0);
}

// draws the clock
void drawTime() {
  tft.setFreeFont(&ArialRoundedMTBold_14);

  String date = wunderground.getDate();

  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth(" Ddd, 44 Mmm 4444 "));  // String width + margin
  tft.drawString(date, 120, 14);

  tft.setFreeFont(&ArialRoundedMTBold_36);

  String timeNow = timeClient.getHours() + ":" + timeClient.getMinutes();

  tft.setTextDatum(BC_DATUM);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);
  tft.setTextPadding(tft.textWidth(" 44:44 "));  // String width + margin
  tft.drawString(timeNow, 120, 50);

  drawSeparator(52);

  tft.setTextPadding(0);
}

// draws current weather information
void drawCurrentWeather() {
  // Weather Icon
  String weatherIcon = getMeteoconIcon(wunderground.getTodayIcon());
  //uint32_t dt = millis();
  ui.drawBmp(weatherIcon + ".bmp", 0, 59);
  //Serial.print("Icon draw time = "); Serial.println(millis()-dt);

  // Weather Text

  String weatherText = wunderground.getWeatherText();
  //weatherText = "Heavy Thunderstorms with Small Hail"; // Test line splitting with longest(?) string

  tft.setFreeFont(&ArialRoundedMTBold_14);

  tft.setTextDatum(BR_DATUM);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);

  int splitPoint = 0;
  int xpos = 230;
  splitPoint =  splitIndex(weatherText);
  if (splitPoint > 16) xpos = 235;

  tft.setTextPadding(tft.textWidth(" Heavy Thunderstorms"));  // Max anticipated string width + margin
  if (splitPoint) tft.drawString(weatherText.substring(0, splitPoint), xpos, 72);
  tft.setTextPadding(tft.textWidth(" with Small Hail"));  // Max anticipated string width + margin
  tft.drawString(weatherText.substring(splitPoint), xpos, 87);

  tft.setFreeFont(&ArialRoundedMTBold_36);

  tft.setTextDatum(TR_DATUM);
  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("-88`"));

  // Font ASCII code 96 (0x60) modified to make "`" a degree symbol
  weatherText = wunderground.getCurrentTemp();
  if (weatherText.indexOf(".")) weatherText = weatherText.substring(0, weatherText.indexOf(".")); // Make it integer temperature
  if (weatherText == "") weatherText = "?";  // Handle null return
  tft.drawString(weatherText + "`", 221, 100);

  tft.setFreeFont(&ArialRoundedMTBold_14);

  tft.setTextDatum(TL_DATUM);
  tft.setTextPadding(0);
  if (IS_METRIC) tft.drawString("C ", 221, 100);
  else  tft.drawString("F ", 221, 100);

  weatherText = wunderground.getWindDir() + " ";
  weatherText += String((int)(wunderground.getWindSpeed().toInt() * WIND_SPEED_SCALING)) + WIND_SPEED_UNITS;

  tft.setTextPadding(tft.textWidth("Variable 888 mph ")); // Max string length?
  tft.drawString(weatherText, 114, 136);

  weatherText = wunderground.getWindDir();

  int windAngle = 0;
  String compassCardinal = "";
  switch (weatherText.length()) {
    case 1:
      compassCardinal = "N O Z W "; // Not used, see default below
      windAngle = 90 * compassCardinal.indexOf(weatherText) / 2;
      break;
    case 2:
      compassCardinal = "NO ZO ZW NW";
      windAngle = 45 + 90 * compassCardinal.indexOf(weatherText) / 3;
      break;
    case 3:
      compassCardinal = "NNO ONO OZO ZZO ZZW WZW WNW NNW";
      windAngle = 22 + 45 * compassCardinal.indexOf(weatherText) / 4; // Should be 22.5 but accuracy is not needed!
      break;
    default:
      if (weatherText == "Variable") windAngle = -1;
      else {
        compassCardinal = "North Oost  Zuid West"; // Possible strings
        windAngle = 90 * compassCardinal.indexOf(weatherText) / 6;
      }
      break;
  }

  tft.fillCircle(128, 110, 23, TFT_BLACK); // Erase old plot, radius + 1 to delete stray pixels
  if ( windAngle >= 0 ) fillSegment(128, 110, windAngle - 15, 30, 22, TFT_GREEN); // Might replace this with a bigger rotating arrow
  tft.drawCircle(128, 110, 6, TFT_RED);
  tft.drawCircle(128, 110, 22, TFT_DARKGREY);    // Outer ring - optional

  drawSeparator(153);

  tft.setTextPadding(0); // Reset padding width to none
}

// draws the three forecast columns
/*void drawForecast() {
  drawForecastDetail(10, 171, 0);
  drawForecastDetail(95, 171, 2);
  drawForecastDetail(180, 171, 4);
  drawSeparator(171 + 69);
  }

  // helper for the forecast columns
  void drawForecastDetail(uint16_t x, uint16_t y, uint8_t dayIndex) {
  tft.setFreeFont(&ArialRoundedMTBold_14);

  String day = wunderground.getForecastTitle(dayIndex).substring(0, 3);
  day.toUpperCase();

  tft.setTextDatum(BC_DATUM);

  tft.setTextColor(TFT_ORANGE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("WWW"));
  tft.drawString(day, x + 25, y);

  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextPadding(tft.textWidth("-88   -88"));
  tft.drawString(wunderground.getForecastHighTemp(dayIndex) + "   " + wunderground.getForecastLowTemp(dayIndex), x + 25, y + 14);

  String weatherIcon = getMeteoconIcon(wunderground.getForecastIcon(dayIndex));
  ui.drawBmp("/mini/" + weatherIcon + ".bmp", x, y + 15);

  tft.setTextPadding(0); // Reset padding width to none
  }
*/
// draw sensordata
void drawEZO() {
  //Title
  tft.setFreeFont(&ArialRoundedMTBold_36);
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
  tft.setFreeFont(&ArialRoundedMTBold_14);
  tft.drawString("C ", 221, 220);
  Blynk.virtualWrite (V1, TEMP_val);
  //tft.setTextDatum(MR_DATUM);
  //tft.setTextColor(TFT_WHITE, TFT_BLACK);
  //tft.setTextPadding(tft.textWidth("test"));
  //tft.drawString("test", 120, 240);

  //PH
  tft.setFreeFont(&ArialRoundedMTBold_36);
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
  tft.setFreeFont(&ArialRoundedMTBold_14);
  tft.drawString("mV", 221, 310);
  Blynk.virtualWrite (V3, ORP_val);
  //Cleanup for next string
  tft.setTextPadding(0); // Reset padding width to none
}

// Helper function, should be part of the weather station library and should disappear soon
String getMeteoconIcon(String iconText) {
  if (iconText == "F") return "chanceflurries";
  if (iconText == "Q") return "chancerain";
  if (iconText == "W") return "chancesleet";
  if (iconText == "V") return "chancesnow";
  if (iconText == "S") return "chancetstorms";
  if (iconText == "B") return "clear";
  if (iconText == "Y") return "cloudy";
  if (iconText == "F") return "flurries";
  if (iconText == "M") return "fog";
  if (iconText == "E") return "hazy";
  if (iconText == "Y") return "mostlycloudy";
  if (iconText == "H") return "mostlysunny";
  if (iconText == "H") return "partlycloudy";
  if (iconText == "J") return "partlysunny";
  if (iconText == "W") return "sleet";
  if (iconText == "R") return "rain";
  if (iconText == "W") return "snow";
  if (iconText == "B") return "sunny";
  if (iconText == "0") return "tstorms";


  return "unknown";
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

// Draw a segment of a circle, centred on x,y with defined start_angle and subtended sub_angle
// Angles are defined in a clockwise direction with 0 at top
// Segment has radius r and it is plotted in defined colour
// Can be used for pie charts etc, in this sketch it is used for wind direction
#define DEG2RAD 0.0174532925 // Degrees to Radians conversion factor
#define INC 2 // Minimum segment subtended angle and plotting angle increment (in degrees)
void fillSegment(int x, int y, int start_angle, int sub_angle, int r, unsigned int colour)
{
  // Calculate first pair of coordinates for segment start
  float sx = cos((start_angle - 90) * DEG2RAD);
  float sy = sin((start_angle - 90) * DEG2RAD);
  uint16_t x1 = sx * r + x;
  uint16_t y1 = sy * r + y;

  // Draw colour blocks every INC degrees
  for (int i = start_angle; i < start_angle + sub_angle; i += INC) {

    // Calculate pair of coordinates for segment end
    int x2 = cos((i + 1 - 90) * DEG2RAD) * r + x;
    int y2 = sin((i + 1 - 90) * DEG2RAD) * r + y;

    tft.fillTriangle(x1, y1, x2, y2, x, y, colour);

    // Copy segment end to sgement start for next segment
    x1 = x2;
    y1 = y2;
  }
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

