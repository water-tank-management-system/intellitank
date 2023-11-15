/*
  author node
  1. this program conducted for communication using UART, sensor data acquisition, and actuation
*/

/**
  *************************************************************************************
  * @file    CAPSTONE_proto.ino
  * @author  Riko Kumara, Muhammad Nurhafiz Sidiq, & Naufal Muafi
  * @version V1.1
  * @date    17-Okt-2023
  * @brief   This file provides all the Cube Animation and Interfacing
  *************************************************************************************
  * @attention
  *
  * COPYRIGHT(c) 2023 Riko Kumara 
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of Riko Kumara Production nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ************************************************************************************
  */

// **** Deklarasi keperluan dari program Hafiz
/**********************
  Deklarasi untuk nilai konstanta
  1. konstanta nilai minimal persentase ketinggian air
  2. konstanta nilai maksimal persentase ketinggian air
  3. konstanta nilai maksimal tingkat kekeruhan
***********************/
#define maxHeightPerc 35  // persentase maksimal ketinggian airnya
#define minHeightPerc 20  // persentase minimal ketinggian airnya
#define maxTurbidity 25   // nilai NTU maksimal kekeruhan
// end of deklarasi konstanta


/*****************
   Definisi untuk state 
   - state untuk membaca sensor
   - state untuk melakukan aktuasi
   - state untuk mengirimkan data ke esp
   - state untuk menerima data dari esp
*****************/
const int read_sensor = 95;
const int actuation = 99;
const int arduino_TX = 96;
const int arduino_RX = 97;
// end of deklarasi state

// Deklarasi variabel
long celcius = 0, height = 0, tss = 0, water_flow = 0;
int state = arduino_TX;
uint16_t cleaning_state_system = 0,
         filling_state_system = 0, auto_mode = 0, cleaning_state_user = 0, filling_state_user = 0;
String data_in;
String data1, data2, data3;
uint8_t indexOfA, indexOfB, indexOfC;
// end of deklarasi variabel


// deklarasi fungsi
void parse_the_data();  // untuk mengambil data dari esp
void transmit_data();   // untuk mengirim data ke esp
void filling_on();      // lakukan pengisian air
void filling_off();



/*Includes.......................................................................................*/
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <EEPROM.h>
#include <LiquidCrystal_I2C.h>
#include <FIR.h>


/*Pin Definition.................................................................................*/
#define trig_Pin 3
#define echo_Pin 4
#define turb_Pin A3
#define turb2_Pin A6
#define temp1_Pin A0
#define temp2_Pin A1
#define flow_Pin 2
#define valve1_Pin 11  // untuk pemakaian dan sensor
#define valve2_Pin 12  // untuk pembuangan
#define pump_Pin 8
#define clean_Pin 9

/*Steinhart Coeff Definition......................................................................*/
#define SERIESRESISTOR 10000      // The Value of Series Resistor
#define NOMINAL_RESISTANCE 10000  // The Nominal Resistance Value of NTC
#define NOMINAL_TEMPERATURE 25    // The Nominal Temperature Value of NTC
#define BCOEFFICIENT 3950         // Coefficient of the NTC

/*Tank Parameter Definition.......................................................................*/
#define TANK_HEIGHT 40  // Tank Height in cm
#define TANK_RADIUS 17  // Tank Radius in cm
#define MAX_LEVEL 28    // Max water level in cm

/*OLED Parameter Definition.......................................................................*/
#define SCREEN_WIDTH 128     // OLED display width, in pixels
#define SCREEN_HEIGHT 64     // OLED display height, in pixels
#define OLED_RESET 4         // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C  // See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/*Private Variables...............................................................................*/
const int temp_Pin[2] = { temp1_Pin, temp2_Pin };
int pump_state = 0;
int clean_state = 0;
int valve1_state = 0;
int valve2_state = 0;
int turbRead = 0;
int tempRead = 0;
float distance = 0;
long durations = 0;
float temperature[2] = { 0, 0 };
float delta_temp = 0;
float turbidity = 0;
float turb_volt = 0;
float turb_amp = 0;
float tank_vol = 0;
float flow_calibrate = 7.5;
float flow_rate = 0;

const int act_transmit = 101;
const int act_retrieve = 102;
const int act_actuating = 103;
int state_act = act_actuating;


volatile byte pulseCount = 0;

unsigned long totalMLitres = 0;
unsigned long oldTime = 0;
unsigned long printTime = 0;

unsigned int flowMLitres = 0;

String dataIn;
char inChar;

int countpin = 0;

int read_waterLevel(void);
int read_turbidity();
int read_flow();
int read_temperature(float tempRead);
void print_data(void);
void pulseCounter(void);

void short_blink();

const unsigned char WTMS[] PROGMEM = {
  // 'WTMSS, 128x64px
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x0c, 0x00, 0x00, 0x00, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x38, 0x0f, 0xff, 0xfc, 0x0f, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xf8, 0x3f, 0xff, 0xff, 0x07, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xfc, 0x00, 0xff, 0x80, 0x0f, 0x86, 0x73, 0x39, 0xfb, 0xf7, 0xc0, 0xfc, 0xe3, 0x9d, 0x98, 0x00,
  0x7e, 0x00, 0x00, 0x00, 0x3f, 0x86, 0x77, 0x39, 0xfb, 0xe7, 0xe0, 0xfc, 0xe3, 0xdd, 0xb8, 0x00,
  0x3f, 0xe0, 0x00, 0x03, 0xfe, 0x06, 0xf6, 0x78, 0x63, 0x06, 0x60, 0x31, 0xe3, 0xdd, 0xb0, 0x00,
  0x0f, 0xff, 0xff, 0xff, 0xf8, 0x03, 0xfe, 0x7c, 0x63, 0xe6, 0x60, 0x31, 0xf3, 0xfd, 0xf0, 0x00,
  0x80, 0x7f, 0xff, 0xff, 0x00, 0x03, 0xfe, 0x6c, 0x63, 0xe7, 0xe0, 0x31, 0xb3, 0xfd, 0xe0, 0x00,
  0xe0, 0x00, 0x00, 0x00, 0x03, 0x83, 0xdc, 0xfc, 0x63, 0x07, 0xc0, 0x33, 0xfb, 0x3d, 0xf0, 0x00,
  0xfe, 0x00, 0x00, 0x00, 0x1f, 0x81, 0xdc, 0xfe, 0x63, 0xe6, 0xc0, 0x33, 0xfb, 0x3d, 0xb8, 0x00,
  0xff, 0xf8, 0x00, 0x07, 0xff, 0x81, 0x9c, 0xee, 0x63, 0xf6, 0x60, 0x33, 0xbb, 0x1d, 0x98, 0x00,
  0xe3, 0xff, 0xff, 0xff, 0xf1, 0x81, 0x88, 0x82, 0x43, 0xe2, 0x20, 0x36, 0x19, 0x08, 0x8c, 0x00,
  0xe0, 0x07, 0xff, 0xfc, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xe0, 0x00, 0x00, 0x00, 0x01, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xe7, 0x00, 0x00, 0x00, 0x31, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xe7, 0xfe, 0x00, 0x3f, 0xf9, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xe7, 0xff, 0xff, 0xff, 0xf9, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xe7, 0xff, 0xff, 0xff, 0xf9, 0x86, 0x1c, 0x70, 0xc6, 0x18, 0x3c, 0x7e, 0xe3, 0xbf, 0x31, 0xbf,
  0xe7, 0xff, 0xff, 0xff, 0xf9, 0x87, 0x1c, 0x70, 0xe6, 0x38, 0x7e, 0x7e, 0xe3, 0xbf, 0x39, 0xbf,
  0xe7, 0xff, 0xc0, 0xff, 0xf9, 0x87, 0xbc, 0x78, 0xe6, 0x3c, 0xe6, 0x60, 0xf7, 0xb8, 0x39, 0x8c,
  0xe7, 0xfc, 0x00, 0x0f, 0xf9, 0x87, 0xfc, 0xf8, 0xf6, 0x7c, 0xc0, 0x7e, 0xff, 0xbf, 0x3d, 0x8c,
  0xe7, 0xf0, 0x00, 0x03, 0xf9, 0x87, 0xfc, 0xd8, 0xfe, 0x7c, 0xcf, 0x7e, 0xff, 0xbf, 0x3f, 0x8c,
  0xe7, 0xe0, 0x00, 0x01, 0xf9, 0x86, 0xec, 0xfc, 0xde, 0x7e, 0xc7, 0x70, 0xdd, 0xb8, 0x37, 0x8c,
  0xe7, 0xe0, 0x1f, 0x01, 0xf9, 0x86, 0xcd, 0xfc, 0xde, 0xfe, 0x7e, 0x7e, 0xc9, 0xbf, 0x33, 0x8c,
  0xe7, 0xf1, 0xff, 0xf3, 0xf9, 0x86, 0x4d, 0xcd, 0xce, 0xef, 0x7e, 0x7e, 0xc1, 0xbf, 0x33, 0x8c,
  0xe7, 0xff, 0xf0, 0x1f, 0xf9, 0x86, 0x09, 0x8c, 0xc6, 0xc3, 0x3c, 0x7e, 0x41, 0x9f, 0x31, 0x8c,
  0xe7, 0xff, 0x00, 0x0f, 0xf9, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xe7, 0xfc, 0x00, 0x0f, 0xf9, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xe7, 0xfe, 0x00, 0x1f, 0xf9, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xe7, 0xff, 0xfe, 0x3f, 0xf9, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xe7, 0xff, 0xff, 0xff, 0xf9, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0xe7, 0xff, 0xc0, 0x7f, 0xf9, 0x83, 0xcc, 0x6f, 0xbf, 0x3f, 0x61, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0xe7, 0xff, 0xc0, 0xff, 0xf9, 0x87, 0xee, 0xcf, 0x9f, 0x3f, 0x71, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0xe7, 0xff, 0xe1, 0xff, 0xf9, 0x86, 0x47, 0xcc, 0x8e, 0x30, 0x7b, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0xe7, 0xff, 0xe3, 0xff, 0xf9, 0x87, 0x87, 0x8f, 0x0e, 0x3e, 0x7f, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0xe7, 0xff, 0xf3, 0xff, 0xf9, 0x83, 0xe3, 0x8f, 0x8e, 0x3e, 0x7f, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0xe7, 0xff, 0xff, 0xff, 0xf9, 0x81, 0xe3, 0x83, 0xce, 0x3e, 0x6f, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0xe7, 0xff, 0xff, 0xff, 0xf1, 0x86, 0x63, 0x8d, 0xce, 0x30, 0x6d, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0xe0, 0x3f, 0xff, 0xff, 0x01, 0x87, 0xe3, 0x8f, 0xce, 0x3f, 0x65, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0xf8, 0x00, 0x00, 0x00, 0x07, 0x83, 0xc3, 0x87, 0x8e, 0x3f, 0x61, 0xc0, 0x00, 0x00, 0x00, 0x00,
  0x7f, 0x00, 0x00, 0x00, 0x3f, 0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x3f, 0xfc, 0x00, 0x07, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x03, 0xff, 0xff, 0xff, 0xf0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x1f, 0xff, 0xfc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x07, 0xc0, 0xe1, 0xfe, 0x3f, 0x9f, 0xe3, 0xf1, 0xc6, 0x3f, 0x80, 0x38, 0x00, 0x03, 0xe1, 0xf8,
  0x0f, 0xe1, 0xe1, 0xfe, 0x7f, 0x9f, 0xe7, 0xf1, 0xe6, 0x3f, 0x80, 0x3c, 0x00, 0x07, 0xf3, 0xf8,
  0x1c, 0xe1, 0xe1, 0xc6, 0x71, 0x07, 0x0e, 0x31, 0xe6, 0x30, 0x00, 0x7c, 0x00, 0x06, 0x33, 0x18,
  0x18, 0x03, 0x31, 0xc6, 0x7f, 0x03, 0x0c, 0x33, 0xfe, 0x3f, 0x00, 0x6c, 0x00, 0x0e, 0x33, 0xf8,
  0x18, 0x03, 0xf1, 0xfe, 0x0f, 0x83, 0x0c, 0x33, 0xbe, 0x7f, 0x00, 0xee, 0x00, 0x0e, 0x31, 0xf8,
  0x1c, 0xe7, 0xf1, 0xfc, 0x63, 0x86, 0x0e, 0x73, 0xbe, 0x70, 0x01, 0xfe, 0x00, 0x0e, 0x70, 0x18,
  0x1f, 0xc7, 0xf9, 0x80, 0x7f, 0x86, 0x07, 0xf3, 0x1c, 0x7f, 0x01, 0xfe, 0x00, 0x07, 0xe3, 0xf8,
  0x0f, 0x8c, 0x19, 0x80, 0x3f, 0x06, 0x07, 0xc3, 0x0c, 0x7f, 0x01, 0x86, 0x00, 0x03, 0xc3, 0xe0,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xc0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x7f, 0xc0, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

// Make an instance of the FIR filter. In this example we'll use
// floating point values and an 8 element filter. For a moving average
// that means an 8 point moving average.
FIR<float, 8> fir;


void setup() {
  Serial.begin(9600);  // Starts the serial communication
  //Arduino_SS.begin(9600);
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    //Serial.println(F("SSD1306 allocation failed"));

    for (;;)
      ;  // Don't proceed, loop forever
  }

  delay(100);
  display.clearDisplay();
  // Display LOGO
  display.drawBitmap(
    (display.width() - 128) / 2,
    (display.height() - 64) / 2,
    WTMS, 128, 64, 1);

  display.display();
  delay(3000);

  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(valve1_Pin, OUTPUT);
  pinMode(valve2_Pin, OUTPUT);
  pinMode(pump_Pin, OUTPUT);
  pinMode(clean_Pin, OUTPUT);
  pinMode(trig_Pin, OUTPUT);
  pinMode(echo_Pin, INPUT);
  pinMode(temp_Pin, INPUT);
  pinMode(turb_Pin, INPUT);
  pinMode(turb2_Pin, INPUT);
  pinMode(flow_Pin, INPUT);

  attachInterrupt(digitalPinToInterrupt(flow_Pin), pulseCounter, FALLING);  // Setup Interrupt

  printTime = millis();

  display.clearDisplay();
  display.display();
  //delay(500);

  float coef[8] = { 1., 1., 1., 1., 1., 1., 1., 1. };

  // Set the coefficients
  fir.setFilterCoeffs(coef);

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  //  display.setCursor(5, 5);
  //  display.print(F("G: "));
  //  display.setCursor(25, 5);
  //  display.println(fir.getGain());
  //  display.display();
  //  delay(1000);
  //  display.clearDisplay();
  fir.getGain();
  for (int i = 0; i < 20; i++) {
    // Read the initial value for FIR
    read_waterLevel();
  }
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  // ***************start of program  riko************************
  digitalWrite(LED_BUILTIN, LOW);

  //actuator_test();
  if ((millis() - oldTime) > 1000) {
    read_flow();
  }
  // **********end of program riko*******************

  /******************************************
     Agar komunikasi dapat dilakukan dengan baik maka digunakan mekanisme switch dengan 4 case
     case pertama : membaca nilai sensor
     case kedua   : mengirimkan data sensor ke esp
     case ketiga  : retrieve data dari esp
     case keempat : melakukan aktuasi berdasarkan kondisi
  *********************************************/
  switch (state) {

    case arduino_TX:
      transmit_data();

      state = arduino_RX;  // ganti state agar arduino menerima data dari esp melalui serial komunikasi
      break;

    case arduino_RX:
      while (Serial.available() > 0) {
        data_in = Serial.readStringUntil('\n');  // argumennya tipe char
        parse_the_data();
        //Serial.println("Data recieved");
        //Serial.print("auto_mode = ");
        //Serial.println(auto_mode);
        //Serial.print("cleaning_state_user = ");
        //Serial.println(cleaning_state_user);
        //Serial.print("filling_state_user = ");
        //Serial.println(filling_state_user);
        //Serial.println("========================");
        state = read_sensor;
        delay(100);
      }
      break;

    case read_sensor:
      // mengambil empat data dari sensor
      celcius = read_temperature();  // ubah random menjadi fungsi yang return nilai temperature in *C
      height = read_waterLevel();    // ubah random menjadi fungsi yang return nilai height -> berikan nilai height dalam bentuk persentase. height = hasilbaca/heightmax x 100%
      tss = read_turbidity();        // ubah random menjadi fungsi yang return nilai turbidity in NTU
      water_flow = read_flow();      // ubah random menjadi fungsi yang return nilai water_flow in L/min
      // end of ngambil data dari sensor


      state = actuation;  // ganti state agar arduino mengirimkan keenam data tersebut ke esp melalui serial komunikasi
      break;

      // ===================== End of Communication section ==========================

    case actuation:
      //    ************************ start of filling logic ***********************
      // jika volume air : min < volume < max
      // buka valve 1 agar air bisa dipake
      // tutup valve 2
      height = read_waterLevel();
      tss = read_turbidity();
      water_flow = read_flow();
      display_data();
      transmit_data();

      if (height < minHeightPerc && pump_state == 0 && cleaning_state_system == 0) {  // jika tangki kosong dan dalam auto mode maka filling_state_system true, jika manual mode maka false
        if (auto_mode > 0) {
          while (height <= maxHeightPerc) {
            if (auto_mode < 1) {
              break;
            }

            switch (state_act) {

              case act_actuating:
                filling_on();
                state_act = act_transmit;
                break;

              case act_transmit:
                height = read_waterLevel();
                celcius = read_temperature();
                water_flow = read_flow();
                transmit_data();
                display_data();
                state_act = act_retrieve;
                break;

              case act_retrieve:
                while (Serial.available() > 0) {
                  data_in = Serial.readStringUntil('\n');  // argumennya tipe char
                  parse_the_data();
                  state_act = act_actuating;
                }
                break;
            }
          }

        }
        // jika tidak mode auto -> Serial.println("Kirim notif kosong dan Pengisian dilakukan secara otomatis");
        else {
          filling_off();
          height = read_waterLevel();

          //Serial.println("Mengirim notifikasi tangki kosong");
        }
        height = read_waterLevel();

      }

      else if (height > minHeightPerc && pump_state == 0) {  // keadaan normal
        normal_volume();
      }

      // jika air kurang dari minimal dan tidak sedang melakukan pembersihan serta dalam auto mode maka nyalakan pompa


      else if (height > maxHeightPerc && pump_state == 1) {
        filling_off();
        height = read_waterLevel();
      }

      // jika user menekan button filling, maka filling_state_system true
      if (filling_state_user > 0) {
        while (height <= maxHeightPerc && filling_state_user > 0) {
          if (filling_state_user == 0) {
            break;
          }

          //tss = read_turbidity();

          switch (state_act) {

            case act_actuating:
              filling_on();
              state_act = act_transmit;
              break;

            case act_transmit:
              height = read_waterLevel();
              celcius = read_temperature();
              water_flow = read_flow();
              transmit_data();
              display_data();
              state_act = act_retrieve;
              break;

            case act_retrieve:
              while (Serial.available() > 0) {
                data_in = Serial.readStringUntil('\n');  // argumennya tipe char
                parse_the_data();
                state_act = act_actuating;
              }
              break;
          }
        }
        filling_off();
      }

      //  ****************** end of filling logic *********************

      // ******************* start of cleaning logic ******************
      if (tss > maxTurbidity && cleaning_state_system == 0) {  // jika tangki kotor dan dalam auto mode maka cleaning_state_system true, jika manual mode maka false
        if (auto_mode > 0) {
          cleaning_state_system = 1;
          height = read_waterLevel();
          transmit_data();
          display_data();
          while (height > minHeightPerc + 4) {
            if (auto_mode < 1) {
              break;
            }

            height = read_waterLevel();
            transmit_data();
            display_data();
            get_empty();

            if (height <= minHeightPerc + 4) {
              //filling_off();
              height = read_waterLevel();
              transmit_data();
              display_data();
              break;
            }
          }
          if (height <= minHeightPerc + 4) {


            while (countpin < 20) {
              if (auto_mode < 1) {
                break;
              }


              if (countpin >= 20) {
                countpin = 0;
                break;
              }
              height = read_waterLevel();
              transmit_data();
              display_data();
              switch (state_act) {

                height = read_waterLevel();
                transmit_data();
                display_data();

                case act_actuating:
                  citric_on();
                  state_act = act_transmit;
                  break;

                case act_transmit:
                  height = read_waterLevel();
                  celcius = read_temperature();
                  water_flow = read_flow();
                  tss = read_turbidity();
                  transmit_data();
                  display_data();
                  state_act = act_retrieve;
                  break;

                case act_retrieve:
                  while (Serial.available() > 0) {
                    data_in = Serial.readStringUntil('\n');  // argumennya tipe char
                    parse_the_data();
                    state_act = act_actuating;
                  }
                  break;
              }
              countpin++;
            }
          }
          countpin = 0;
          citric_off();


          //Serial.println("Pembersihan dilakukan melalui perintah user");
          cleaning_state_system = 0;
        } else {  // kalo gak mode auto
          tss = read_turbidity();
          //Serial.println("Mengirim notifikasi bersihin");
        }
      } else {
        tss = read_turbidity();
      }

      // jika user menekan button clening, maka cleaning_state_system true
      if (cleaning_state_user > 0 && cleaning_state_system == 0) {
        cleaning_state_system = 1;
        height = read_waterLevel();
        transmit_data();
        display_data();
        while (height > minHeightPerc + 4) {
          height = read_waterLevel();
          transmit_data();
          display_data();
          get_empty();

          if (height <= minHeightPerc + 4) {
            //filling_off();
            height = read_waterLevel();
            transmit_data();
            display_data();
            break;
          }
        }
        if (height <= minHeightPerc + 4) {


          while (countpin < 20) {

            if (countpin >= 20) {
              countpin = 0;
              break;
            }
            height = read_waterLevel();
            transmit_data();
            display_data();
            switch (state_act) {

              height = read_waterLevel();
              transmit_data();
              display_data();

              case act_actuating:
                citric_on();
                state_act = act_transmit;
                break;

              case act_transmit:
                height = read_waterLevel();
                celcius = read_temperature();
                water_flow = read_flow();
                tss = read_turbidity();
                transmit_data();
                display_data();
                state_act = act_retrieve;
                break;

              case act_retrieve:
                while (Serial.available() > 0) {
                  data_in = Serial.readStringUntil('\n');  // argumennya tipe char
                  parse_the_data();
                  state_act = act_actuating;
                }
                break;
            }
            countpin++;
          }
        }
        countpin = 0;
        citric_off();



        //Serial.println("Pembersihan dilakukan melalui perintah user");
        cleaning_state_system = 0;
      } else {
        tss = read_turbidity();
      }

      // ******************* end of cleaning logic ******************


      state = arduino_TX;
      //Serial.println("Ni kelar actuation");
      break;

  }  // akhir dari switch

  digitalWrite(LED_BUILTIN, HIGH);
  delay(1);
  read_waterLevel();
  read_temperature();
  read_turbidity();
  display_data();

}  // akhir dari fungsi loop

//void loop() {




/* 
  if((millis() - printTime) >= 1000){
    print_data();
    printTime = millis();
  }
    
  /*
  if(countpin == 0){
    digitalWrite(valve1_Pin, HIGH);
    countpin++;
  }
  else if(countpin == 1){
    digitalWrite(valve1_Pin, LOW);
    delay(10);
    digitalWrite(valve2_Pin, HIGH);
    countpin++;
  }
  else if(countpin == 2){
    digitalWrite(valve2_Pin, LOW);
    delay(10);
    digitalWrite(clean_Pin, HIGH);
    countpin++;
  }
  else if(countpin == 3){
    digitalWrite(clean_Pin, LOW);
    delay(10);
    digitalWrite(pump_Pin, HIGH);
    countpin++;
  }
  else if(countpin == 4){ 
    digitalWrite(pump_Pin, LOW);
    delay(10);
    countpin=0;
  }
   
  
  */


//} // akhir loop

void display_data(void) {

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(5, 5);
  display.print(F("T1:"));
  display.setCursor(25, 5);
  display.print(temperature[0], 1);

  display.setCursor(70, 5);
  display.print(F("T2:"));
  display.setCursor(90, 5);
  display.println(temperature[1], 1);

  display.setCursor(5, 15);
  display.print(F("Turb:"));
  display.setCursor(38, 15);
  display.print(turbidity, 1);
  display.setCursor(70, 15);
  display.println(F("| adc"));
  display.setCursor(102, 15);
  display.println(analogRead(turb_Pin));

  display.setCursor(5, 25);
  display.print(F("Tvol:"));
  display.setCursor(38, 25);
  display.print(turb_volt, 3);
  display.setCursor(70, 25);
  display.print(F("| v1 ["));
  display.setCursor(108, 25);
  display.print(valve1_state);
  display.setCursor(115, 25);
  display.println(F("]"));


  display.setCursor(5, 35);
  display.print(F("Tamp:"));
  display.setCursor(38, 35);
  display.print(turb_amp, 3);
  display.setCursor(70, 35);
  display.print(F("| v2 ["));
  display.setCursor(108, 35);
  display.print(valve2_state);
  display.setCursor(115, 35);
  display.println(F("]"));

  display.setCursor(5, 45);
  display.print(F("Volm:"));
  display.setCursor(38, 45);
  display.print(height, 1);
  display.setCursor(70, 45);
  display.print(F("| cc ["));
  display.setCursor(108, 45);
  display.println(cleaning_state_system);
  display.setCursor(115, 45);
  display.println(F("]"));

  display.setCursor(5, 55);
  display.print(F("Flow:"));
  display.setCursor(38, 55);
  display.print(flow_rate);
  display.setCursor(70, 55);
  display.print(F("| pc ["));
  display.setCursor(108, 55);
  display.println(pump_state);
  display.setCursor(115, 55);
  display.println(F("]"));

  display.display();

  delay(1);
}

void print_data(void) {
  Serial.println("|---------------------------|");
  Serial.print("> Dist : ");
  Serial.print(distance);
  Serial.println("      cm");
  Serial.print("> Temp1: ");
  Serial.print(temperature[0]);
  Serial.println("  'C");
  Serial.print("> Temp2: ");
  Serial.print(temperature[1]);
  Serial.println("  'C");
  Serial.print("> Turb : ");
  Serial.print(turbidity);
  Serial.println(" NTU");
  Serial.print("> TurbV: ");
  Serial.print(turb_volt);
  Serial.println(" V");
  Serial.print("> WFlow: ");
  Serial.print(int(flow_rate));
  Serial.println("   L/min");
  Serial.println("|---------------------------|");
}

int read_waterLevel() {

  digitalWrite(trig_Pin, LOW);  // Clears the trigPin
  delayMicroseconds(2);
  digitalWrite(trig_Pin, HIGH);  // Sets the trigPin on HIGH state for 10 ms
  delay(10);
  digitalWrite(trig_Pin, LOW);  // Reads the echoPin

  durations = pulseIn(echo_Pin, HIGH);  // Calculating the distance
  float dist = durations / 58.;         // Convert the distance to centimeter

  distance = fir.processReading(dist);

  tank_vol = (22 / 7) * square(TANK_RADIUS) * (TANK_HEIGHT - distance) / 1000;
  int water_level = 100 * (TANK_HEIGHT - distance) / MAX_LEVEL;
  if (water_level > 100) {
    return 100;
  } else if (water_level < 0) {
    return 0;
  } else {
    return water_level;
  }
}

int read_temperature() {
  for (int i = 0; i < 2; i++) {
    tempRead = analogRead(temp_Pin[i]);
    float Resistance;
    float steinhart;

    Resistance = (1023. / tempRead) - 1;  // Convert to Resistance Value
    Resistance = SERIESRESISTOR / Resistance;
    steinhart = Resistance / NOMINAL_RESISTANCE;        // (R/Ro)
    steinhart = log(steinhart);                         // ln(R/Ro)
    steinhart /= BCOEFFICIENT;                          // 1/B * ln(R/Ro)
    steinhart += 1.0 / (NOMINAL_TEMPERATURE + 273.15);  // + (1/To)
    steinhart = 1.0 / steinhart;                        // Invert
    steinhart -= 273.15;                                // convert to C
    temperature[i] = steinhart;
  }
  int temp = temperature[0];
  return temp;
}

int read_turbidity() {
  turb_volt = 0;
  turb_amp = 0;
  for (int i = 0; i < 800; i++) {
    turb_volt += ((float)analogRead(turb_Pin) / 1023) * 5.;
    turb_amp += ((float)analogRead(turb2_Pin) / 1023) * 5.;
  }
  turb_volt = turb_volt / 800;
  turb_volt = round_to_dp(turb_volt, 3);

  turb_amp = turb_amp / 800;
  turb_amp = round_to_dp(turb_amp, 3);

  //turbidity = -1120.4*square(turb_volt)+5742.3*turb_volt-1970.8;
  turbidity = (1143515 - 781231.5 * (turb_volt) + 178051.5 * square(turb_volt) - 13529.19 * square(turb_volt) * turb_volt)/100;
  /*
  Serial.print("> ADCv : ");
  Serial.println(analogRead(turb_Pin));
  Serial.print("> Volt : ");
  Serial.println(turb_volt, 3); 
  Serial.print("> vAMP : ");
  Serial.println(turb_amp, 3); 
  Serial.print("> Turb : ");
  Serial.println(turbidity);   
  Serial.println("|---------------------------|");
  */
  return turbidity;
}

int read_flow() {

  detachInterrupt(flow_Pin);  // Disable the interrupt while calculating flow rate and sending the value to the host
  flow_rate = ((1000.0 / (millis() - oldTime)) * pulseCount) / flow_calibrate;
  oldTime = millis();

  // Divide the flow rate in litres/minute by 60 to determine how many litres have
  // passed through the sensor in this 1 second interval, then multiply by 1000 to
  // convert to millilitres.
  flowMLitres = (flow_rate / 60) * 1000;

  // Add the millilitres passed in this second to the cumulative total
  totalMLitres += flowMLitres;  // the total volume of water passed since the start

  unsigned int frac;

  // Reset the pulse counter so we can start incrementing again
  pulseCount = 0;

  attachInterrupt(digitalPinToInterrupt(flow_Pin), pulseCounter, FALLING);
  return flow_rate;
}

void actuator_test(void) {

  digitalWrite(valve1_Pin, LOW);
  digitalWrite(valve2_Pin, LOW);
  digitalWrite(pump_Pin, LOW);
  digitalWrite(clean_Pin, LOW);
  delay(200);

  digitalWrite(valve1_Pin, HIGH);
  digitalWrite(valve2_Pin, HIGH);
  digitalWrite(pump_Pin, HIGH);
  digitalWrite(clean_Pin, HIGH);
  delay(500);

  digitalWrite(valve1_Pin, LOW);
  digitalWrite(valve2_Pin, LOW);
  digitalWrite(pump_Pin, LOW);
  digitalWrite(clean_Pin, LOW);
  delay(200);

  digitalWrite(valve1_Pin, HIGH);
  delay(500);
  digitalWrite(valve1_Pin, LOW);
  delay(200);

  digitalWrite(valve2_Pin, HIGH);
  delay(500);
  digitalWrite(valve2_Pin, LOW);
  delay(200);

  digitalWrite(clean_Pin, HIGH);
  delay(500);
  digitalWrite(clean_Pin, LOW);
  delay(200);

  digitalWrite(pump_Pin, HIGH);
  delay(500);
  digitalWrite(pump_Pin, LOW);
  delay(200);
}

void pulseCounter(void) {
  pulseCount++;
}

float round_to_dp(float in_value, int decimal_place) {
  float multiplier = powf(10.0f, decimal_place);
  in_value = roundf(in_value * multiplier) / multiplier;
  return in_value;
}

void parse_the_data() {
  indexOfA = data_in.indexOf("A");
  indexOfB = data_in.indexOf("B");
  indexOfC = data_in.indexOf("C");


  data1 = data_in.substring(0, indexOfA);
  data2 = data_in.substring(indexOfA + 1, indexOfB);
  data3 = data_in.substring(indexOfB + 1, indexOfC);

  auto_mode = data1.toInt();
  cleaning_state_user = data2.toInt();
  filling_state_user = data3.toInt();
  //Serial.print(typeid(auto_mode).name()); Serial.print(typeid(cleaning_state_user).name()); Serial.println(typeid(filling_state_user).name());
}




void transmit_data() {
  //Serial.println("Data transmitted");
  Serial.print(celcius);
  Serial.print("A");
  //Serial.print("Temperature: ");
  //Serial.println(celcius);  // kirimkan data temperature melalui serial communication
  Serial.print(height);
  Serial.print("B");
  //Serial.print("Height: ");
  //Serial.println(height);  // kirimkan data height melalui serial communication
  Serial.print(tss);
  Serial.print("C");
  //Serial.print("Turbidity: ");
  //Serial.println(turbidity);  // kirimkan data turbidity melalui serial communication
  Serial.print(water_flow);
  Serial.print("D");
  //Serial.print("Water flow: ");
  //Serial.println(water_flow);  // kirimkan data water_flow melalui serial communication
  Serial.print("\n");
  //Serial.println("========================");
  delay(100);
}



void filling_on() {
  digitalWrite(valve1_Pin, LOW);  // tutup valve1
  digitalWrite(valve2_Pin, LOW);  // tutup valve2
  digitalWrite(pump_Pin, HIGH);   // nyalakan pompa
  valve1_state = 0;
  valve2_state = 0;
  pump_state = 1;
}

void filling_off() {
  digitalWrite(valve1_Pin, LOW);  // buka valve untuk sensor water_flow dan tempat keluar air, orinya high
  digitalWrite(valve2_Pin, LOW);  // tutup valve pembuangan
  digitalWrite(pump_Pin, LOW);    // hentikan pengisian
  valve1_state = 0;               // orinya 1
  valve2_state = 0;
  pump_state = 0;  // kembalikan nilai pump_state menjadi 0
}

void get_empty() {  // buang air pake valve 2
  digitalWrite(valve1_Pin, LOW);
  digitalWrite(valve2_Pin, HIGH);
  valve1_state = 0;
  valve2_state = 1;
}

void citric_on() {
  digitalWrite(clean_Pin, HIGH);  //harusnya clean
  digitalWrite(valve2_Pin, HIGH);
  valve2_state = 1;
}

void citric_off() {
  digitalWrite(clean_Pin, LOW);
  digitalWrite(valve1_Pin, LOW);
  digitalWrite(valve2_Pin, LOW);
  valve1_state = 0;
  valve2_state = 0;
}


void normal_volume() {
  digitalWrite(valve1_Pin, HIGH);  // originalnya high
  digitalWrite(valve2_Pin, LOW);
  valve1_state = 1;  // originalnya 1
  valve2_state = 0;
  height = read_waterLevel();
}
// this is the end of code
