/*  
    RSX for CANSAT-2018
    This is the onboard software for CANSAT-2018 competition (probe)
    Team 1138: RSX from University of Toronto / UTIAS
    Code Written by Eric Boszin and Philip Lu
*/

// Libraries
#include <Wire.h>
#include "SFE_BMP180.h"
#include <Servo.h>
#include "TinyGPS++.h"
#include <SoftwareSerial.h>

//  Pins
#define PIN_RXD 0
#define PIN_TXD 1
#define PIN_LED 13
#define PIN_BUZZER 3
#define PIN_BATT A3
#define PIN_RX 8
#define PIN_TX 7

// Telemetry
double dataVoltage, dataTemp, dataPress, dataAlt;
float dataGPSLat, dataGPSLong, dataGPSAlt;
uint32_t dataGPSTime, dataGPSNum;

// BMP180
SFE_BMP180 bmp180;
double P0;

// Servos
Servo heatServo, paraServo, relServo;
#define OPEN 180 //change these
#define CLOSE 0 //change these
#define ON 180 // or 0 for opposite direction
#define OFF 90

// GPS
TinyGPSPlus gps;
SoftwareSerial ss(PIN_RX, PIN_TX);

// Misc.


void setup() {
  Serial.begin(9600);

  // LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);

  // GPS
  ss.begin(9600);
  
  //  Buzzer
  pinMode(PIN_BUZZER, OUTPUT);

  // Battery
  pinMode(PIN_BATT, INPUT);

  // BMP180
  bool success;
  success = bmp180.begin();
  if(!success){
    Serial.println("BMP180 init fail");
  }
  P0 = readBMP();

  // Servo
  heatServo.attach(5); // middle
  paraServo.attach(6); // bottom
  relServo.attach(9);  // top
  // Set defaults for servos
  heatServo.write(CLOSE);
  paraServo.write(CLOSE);
  relServo.write(OFF);
  
}

void loop(){
    readGPS();
}


void readGPS(){
  if(ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated()){
      
      // Latitude in degrees (double)
      dataGPSLat = gps.location.lat();  
      
      Serial.print("Latitude= "); 
      Serial.print(dataGPSLat, 1); 
      
           
      // Longitude in degrees (double)
      dataGPSLong = gps.location.lng();
      
      Serial.print(" Longitude= "); 
      Serial.println(dataGPSLong, 1); 
      

      // Raw time in HHMMSSCC format (u32)
      dataGPSTime = gps.time.value();
      
      Serial.print("Raw time in HHMMSSCC = "); 
      Serial.println(dataGPSTime); 
      

      // Altitude in meters (double)
      dataGPSAlt = gps.altitude.meters();
      
      Serial.print("Altitude in meters = "); 
      Serial.println(dataGPSAlt, 1); 
      

      // Number of satellites in use (u32)
      dataGPSNum = gps.satellites.value();
      
      Serial.print("Number os satellites in use = "); 
      Serial.println(dataGPSNum); 
      
    }
  }
}

double readBMP(){
  char status;
  double T, P;
  bool success = false;

  status = bmp180.startTemperature();
  if (status != 0) {
    delay(4.5);
    status = bmp180.getTemperature(T);
    if (status != 0) {
      status = bmp180.startPressure(3);
      if (status != 0) {
        delay(status);
        status = bmp180.getPressure(P, T);
        if (status != 0) {
          
          dataPress = 0.1 * P;
          /*
          Serial.print("Pressure: ");
          Serial.print(dataPress, 1);
          Serial.println(" kPa");
          */
          
          dataTemp = T;
          /*
          Serial.print("Temperature: ");
          Serial.print(dataTemp, 1);
          Serial.println(" C");
          */
          
          dataAlt = bmp180.altitude(dataPress, P0); 
          /*
          Serial.print("Altitude: ");
          Serial.print(dataAlt, 1);
          Serial.println(" m");
          */
          return 0.1 * P;
        }
      }
    }
  }
}

void readVoltage() {
  double voltage = analogRead(PIN_BATT);
  dataVoltage = (double) voltage / 102.3;
}

void landBuzzer(){
  while(1){
    uint16_t et = 0;
    while (et < 500) {
    digitalWrite(PIN_BUZZER, HIGH);
    delayMicroseconds(900);

    digitalWrite(PIN_BUZZER, LOW);
    delayMicroseconds(900);
    et += 2;
    }
  }
}
/*
void send_telemetry() {
  Serial.print(dataAlt, 1);
  Serial.print(",");
  Serial.print(dataPress, 1);
  Serial.print(",");
  Serial.print(dataTemp, 1); 
  Serial.print(",");
  Serial.print(dataVoltage, 1); 
  Serial.print(",");
  Serial.print(dataGPSTime); //
  Serial.print(",");
  Serial.print(dataGPSLat, 1); // 
  Serial.print(",");
  Serial.print(dataGPSLong, 1); //
  Serial.print(",");
  Serial.print(dataGPSAlt, 1); //
  Serial.print(",");
  Serial.print(dataGPSNum); //
  Serial.print(",");
  Serial.print(madgwick.getRoll()); //
  Serial.print(",");
  Serial.print(madgwick.getPitch()); //
  Serial.print(",");
  Serial.print(madgwick.getYaw()); //
  Serial.print(",");
  Serial.print(flightState); //
  Serial.print("\n");
  Serial.flush();
}
*/

