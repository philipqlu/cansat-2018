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
#define PIN_RX 7
#define PIN_TX 8
#define PIN_SDA A4
#define PIN_SCL A5

// Telemetry
double dataVoltage, dataTemp, dataPress, dataAlt;
double dataGPSLat, dataGPSLong, dataGPSAlt;
uint32_t dataGPSNum, dataGPSTime;
double dataTiltX, dataTiltY, dataTiltZ;
uint8_t flightState, nextCycle;
double lastAlt, globalTimer;
uint32_t timeNow, lastCycle;


// BMP180
SFE_BMP180 bmp180;
double P0;
#define LANDED 5  // liftoff/landed threshold *
#define DESCENT 2 // lastAlt - dataAlt threshold *
#define TIMER 50 // globalTimer threshold; timing servo release and landed state *

// Servos
Servo heatServo, paraServo, relServo;
#define OPEN 180 // *
#define CLOSE 0 // *
#define ON 180 // or 0 for opposite direction *
#define OFF 90

// GPS
TinyGPSPlus gps;
SoftwareSerial ss(PIN_RX, PIN_TX);

// Misc.


void setup() {
  Serial.begin(9600);

  // LED
  pinMode(PIN_LED, OUTPUT);

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
  delay(1000); // Wait for BMP to get better pressure reading
  P0 = readBMP();

  // MPU9250

  // BMP/MPU
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);

  // Servo
  heatServo.attach(9); // top, yellow wire !facing arduino
  paraServo.attach(5); // middle, yellow wire !facing arduino
  relServo.attach(6);  // bottom, yellow wire facing arduino
  
  // Set defaults for servos
  heatServo.write(CLOSE);
  paraServo.write(CLOSE);
  relServo.write(OFF);

  flightState = 1;
}

void loop(){

  getCommand();
  
  readGPS();
  
  timeNow = millis();

  if (timeNow - nextCycle >= lastCycle) {
    lastCycle = timeNow;
    getFlightState();
    
    switch (flightState) {
      case 1: // prelaunch
              readVoltage();
              readBMP();
  //            readMPU(); // *
              nextCycle = 1000;
              break;
    
      case 2: // ascending
              readVoltage();
              readBMP();
  //            readMPU(); // *
              nextCycle = 200;
              break;
    
      case 3: // stabilizing
              heatServo.write(OPEN);
              readVoltage();
              readBMP();
  //            readMPU(); // *
              nextCycle = 200;
              break;
    
      case 4: // release
              relServo.write(ON);       // write speed to turn
              paraServo.write(OPEN);    // write position
              if (globalTimer == TIMER){
                relServo.write(OFF);    // write speed to stop turn
              }
              globalTimer ++;
              readVoltage();
              readBMP();
  //            readMPU(); // *
              nextCycle = 1000;
              break;
    
      case 5: //descent
              if (abs(dataAlt - lastAlt) < LANDED){
                globalTimer++;
              }
              readVoltage();
              readBMP();
  //            readMPU(); // *
              nextCycle = 200;
              break;
    
      case 6: // landed
              landBuzzer();
              break;
    }
    sendTelemetry();
  }
}


void getFlightState() {
  if (flightState == 1 && (dataAlt > LANDED)) {
    flightState = 2; //Ascending
  }
  else if (flightState == 2 && ((lastAlt - dataAlt) > DESCENT)) {
    flightState = 3; //Stabilizing
  }
  else if (flightState == 3 && (dataAlt <= 300)) {
    flightState = 4; // Release
  }
  else if (flightState == 4 && globalTimer >= TIMER) { 
      flightState = 5; //Descent
      globalTimer = 0;
  }
  else if (flightState == 5 && (dataAlt < LANDED) && globalTimer >= TIMER) {
      flightState = 6; //Landed
  }
}

void getCommand() {
  if (Serial.available()) {
    String command;
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') {
        interpretCommand(command);
        command = "";
      }
      else {
        command += c;
      }
    }
  }
}

void interpretCommand(String command) {
  String com = command.substring(0, 1);
  if (com.equals("f")) {
    int forceCom = command.substring(2).toInt();
    if (forceCom == 0) {
      Reset();
    } if (forceCom == 1) {
      flightState = 1;
    } else if (forceCom == 2) {
      flightState = 2;
    } else if (forceCom == 3) {
      flightState = 3;
    } else if (forceCom == 4) {
      flightState = 4;
    } else if (forceCom == 5) {
      flightState = 5;
    } else if (forceCom == 6) {
      flightState = 6;
    }
  }
}

void readGPS(){
  while(ss.available() > 0){
    gps.encode(ss.read());
    if (gps.location.isUpdated() && gps.altitude.isUpdated()){
      // Latitude in degrees (double)
      dataGPSLat = gps.location.lat();  

      // Longitude in degrees (double)
      dataGPSLong = gps.location.lng();

      // Raw time in HHMMSSCC format (u32)
      dataGPSTime = 0.01 * gps.time.value(); // 0.01 to chop off trailing 0s

      // Altitude in meters (double)
      dataGPSAlt = gps.altitude.meters();

       // Number of satellites in use (u32)
      dataGPSNum = gps.satellites.value();
      /*
      Serial.print("Latitude= "); 
      Serial.print(dataGPSLat, 6); 
      Serial.print(" Longitude= "); 
      Serial.println(dataGPSLong, 6); 
      Serial.print("Raw time in HHMMSS = "); 
      Serial.println(dataGPSTime);
      Serial.print("Altitude in meters = "); 
      Serial.println(dataGPSAlt, 1); 
      Serial.print("Number of satellites in use = "); 
      Serial.println(dataGPSNum); 
      */
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
          dataTemp = T;
          lastAlt = dataAlt;
          dataAlt = bmp180.altitude(dataPress, P0); 
          /*
          Serial.print("Pressure: ");
          Serial.print(dataPress, 1);
          Serial.println(" kPa");
          Serial.print("Temperature: ");
          Serial.print(dataTemp, 1);
          Serial.println(" C");
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
  bool LED = true;
  while(1){
    digitalWrite(PIN_LED, LED);
    uint16_t et = 0;
    while (et < 500) {
      digitalWrite(PIN_BUZZER, HIGH);
      delayMicroseconds(400);
      digitalWrite(PIN_BUZZER, LOW);
      delayMicroseconds(400);
      et ++;
    }
    LED = !LED;
  }
}

void sendTelemetry() {
  Serial.print(dataAlt, 1);
  Serial.print(",");
  Serial.print(dataPress, 1);
  Serial.print(",");
  Serial.print(dataTemp, 1); 
  Serial.print(",");
  Serial.print(dataVoltage, 1); 
  Serial.print(",");
  Serial.print(dataGPSTime);
  Serial.print(",");
  Serial.print(dataGPSLat, 6); 
  Serial.print(",");
  Serial.print(dataGPSLong, 6);
  Serial.print(",");
  Serial.print(dataGPSAlt, 1); 
  Serial.print(",");
  Serial.print(dataGPSNum);
  Serial.print(",");
  Serial.print(dataTiltX, 1); // *
  Serial.print(",");
  Serial.print(dataTiltY, 1); // *
  Serial.print(",");
  Serial.print(dataTiltZ, 1); // *
  Serial.print(",");
  Serial.print(flightState); // *
  Serial.print("\n");
  Serial.flush();
}

void Reset(){
  Serial.begin(9600);

  // LED
  pinMode(PIN_LED, OUTPUT);

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
  delay(1000); // Wait for BMP to get better pressure reading
  P0 = readBMP();

  // MPU9250

  // BMP/MPU
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);

  // Servo
  heatServo.attach(9); // top, yellow wire !facing arduino
  paraServo.attach(5); // middle, yellow wire !facing arduino
  relServo.attach(6);  // bottom, yellow wire facing arduino
  // Set defaults for servos
  heatServo.write(CLOSE);
  paraServo.write(CLOSE);
  relServo.write(OFF);

  dataVoltage, dataTemp, dataPress, dataAlt = 0,0,0,0;
  dataGPSLat, dataGPSLong, dataGPSAlt = 0,0,0;
  dataGPSNum, dataGPSTime = 0,0;
  dataTiltX, dataTiltY, dataTiltZ = 0,0,0;
  flightState = 0;
  lastAlt, globalTimer = 0,0;
}


