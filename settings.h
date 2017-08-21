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
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
See more at http://blog.squix.ch

Adapted by Bodmer to use the faster TFT_ILI9341_ESP library:
https://github.com/Bodmer/TFT_ILI9341_ESP

*/

// Setup
const int UPDATE_INTERVAL_SECS = 10 * 60; // Update every 10 minutes
const int UPDATE_TEMP_SECS = 30; 


// Wifi credentials
char ssid[] = "Siersma2";
char pass[] = "Dustin01!";


// Pins for the TFT interface are defined in the User_Config.h file inside the TFT_ILI9341_ESP library
// These are the ones I used on a NodeMCU:
// #define TFT_DC D3
// #define TFT_CS D8
///#define TFT_RST -1 // Minus one means no pin allocated, connect to NodeMCU RST pin

//OneWire pin and precision for sensor
#define ONE_WIRE_BUS 2              //data pin for tempsensor, use a 4.7k Ohm resistor between data and vcc!
#define TEMPERATURE_PRECISION 9     //9 bit precision

//Blynk Auth token
char auth[] = "0a3b9f9e4c3446508446b726ffb6a66f";


//Ticker Watchdog




/***************************
 * End Settings
 **************************/
