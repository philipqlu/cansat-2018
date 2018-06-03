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
#include "MPU9250.h"       // https://github.com/bolderflight/MPU9250


//  Pins
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
double dataTiltgX, dataTiltgY, dataTiltgZ, 
       tiltX, tiltY, tiltZ;
uint8_t flightState;
uint16_t nextCycle;
double lastAlt, globalTimer;
uint32_t timeBefore, timeNow, lastCycle, lastTelemCycle;//, init_time, t; // used for simulation


// BMP180
SFE_BMP180 bmp180;
double P0;
#define LANDED 5  // liftoff/landed threshold
#define STOPPED 1
#define DESCENT 3 // (lastAlt - dataAlt) threshold
#define TIMER 5000/200 // globalTimer threshold; timing servo release
#define LANDTIMER 5000/200 // globalTimer threshold; timing landed state

// Servos
Servo heatServo, paraServo, relServo;
#define OPEN 180 
#define CLOSE 0 
#define ON 0 
#define OFF 90
#define SCREW 180

// GPS
TinyGPSPlus gps;
SoftwareSerial ss(PIN_RX, PIN_TX);

// MPU9250
MPU9250 IMU(Wire, 0x68);

void setup() {
  Serial.begin(9600);

  // LED
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH);
  
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
  if(IMU.begin() < 0){
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
  }

  // BMP/MPU
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);

  // Servo setup
  heatServo.attach(9); // top, yellow wire !facing arduino
  heatServo.write(CLOSE);
  delay(800);
  heatServo.detach(); 
  
  paraServo.attach(5); // middle, yellow wire !facing arduino
  paraServo.write(CLOSE);
  delay(800);
  paraServo.detach(); 

  ss.begin(9600);

  flightState = 1;
  lastCycle = millis();
  lastTelemCycle = lastCycle;
  //init_time = millis(); // used for simulation
}

void loop(){
  String command;
  while (Serial.available()) {
    char com = Serial.read();
    if (com == '\n') {
      if (command.equals("f00")) {
        Reset();
      } else if (command.equals("f01")) {
        flightState = 1;
      } else if (command.equals("f02")) {
        flightState = 2;
      } else if (command.equals("f03")) {
        flightState = 3;
      } else if (command.equals("f04")) {
        flightState = 4;
      } else if (command.equals("f05")) {
        flightState = 5;
      } else if (command.equals("f06")) {
        flightState = 6;
      }command = "";
      Serial.flush();
      smartDelay(100);
    }
    else {
      command += com;
    }
  } 
  readGPS();
  readMPU();
 
  timeNow = millis();
  if (lastCycle > timeNow) lastCycle = timeNow;
  if (lastTelemCycle > timeNow) lastTelemCycle = timeNow;
  
  if (timeNow - nextCycle >= lastCycle) {
    lastCycle = timeNow;
    readBMP();
    readVoltage();
    getFlightState();
    
    switch (flightState) {
      case 1: // prelaunch
              nextCycle = 1000;
              break;
    
      case 2: // ascending
              nextCycle = 200;
              break;
    
      case 3: // stabilizing
              nextCycle = 200;
              break;
    
      case 4: // release
              globalTimer ++;
              if (globalTimer >= TIMER){
                relServo.write(OFF);    // write speed to stop turn
                smartDelay(1000);
                relServo.detach();
              }
              nextCycle = 200;
              break;
    
      case 5: //descent
              if (abs(dataAlt - lastAlt) < LANDED){
                globalTimer++;
              }
              nextCycle = 200;
              break;
    
      case 6: // landed
              landBuzzer();
              break;
    }
    if (timeNow - 1000 >= lastTelemCycle && flightState != 6) {
      lastTelemCycle = timeNow;
      sendTelemetry();
    }
  }
}


void getFlightState() {
  if (flightState == 1 && (dataAlt > LANDED)) {
    flightState = 2; //Ascending
  }
  else if (flightState == 2 && ((lastAlt - dataAlt) > DESCENT) && dataAlt > 400) {
    flightState = 3; //Stabilizing
    ss.end();
    heatServo.attach(9);
    heatServo.write(OPEN);
    delay(1000);
    heatServo.detach();
    ss.begin(9600);
  }
  else if (flightState == 3 && (dataAlt <= 300)) {
    flightState = 4; // Release
    ss.end();
    paraServo.attach(5);
    paraServo.write(OPEN);    // write position
    smartDelay(1000);
    paraServo.detach();
    relServo.attach(6); 
    relServo.write(ON);       // write speed to turn
    ss.begin(9600);
    globalTimer = 0;
  }
  else if (flightState == 4 && globalTimer >= TIMER) { 
      flightState = 5; //Descent
      globalTimer = 0;
  }
  else if (flightState == 5 && ((lastAlt - dataAlt) < STOPPED) && globalTimer >= LANDTIMER) {
      flightState = 6; //Landed
  }
}

void readMPU(){
  double  dataTiltgXBefore = dataTiltgX;
  double  dataTiltgYBefore = dataTiltgY;
  double  dataTiltgZBefore = dataTiltgZ;

  IMU.readSensor();

  dataTiltgX = IMU.getGyroX_rads();
  dataTiltgY = IMU.getGyroY_rads();
  dataTiltgZ = IMU.getGyroZ_rads();

  uint32_t t = millis();
  tiltX = tiltX + (t - timeBefore)/1000.0 * (dataTiltgX + dataTiltgXBefore) / 2.0;
  tiltY = tiltY + (t - timeBefore)/1000.0 * (dataTiltgY + dataTiltgYBefore) / 2.0;
  tiltZ = tiltZ + (t - timeBefore)/1000.0 * (dataTiltgZ + dataTiltgZBefore) / 2.0;
  timeBefore = t;
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
          return 0.1 * P;

/*  // Simulated Altitude
  char status;
  double T, P;
  
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
          t = (millis() - init_time)/1000;
          dataAlt = -700/225*(t-5)*(t-35);
          
          if (dataAlt <= 0 || dataAlt >=  800){
              dataAlt = 0;
             
          }
          dataAlt = dataAlt +(random(10,50)-30)/10;
          return 0.1 * P; */
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
  digitalWrite(PIN_LED, HIGH);
  uint16_t et = 0;
  while (et < 500) {
    digitalWrite(PIN_BUZZER, HIGH);
    delayMicroseconds(400);
    digitalWrite(PIN_BUZZER, LOW);
    delayMicroseconds(400);
    et ++;
  }
  digitalWrite(PIN_LED, LOW);
  delay(500);
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
  Serial.print(wrapTilt(tiltX), 2); 
  Serial.print(",");
  Serial.print(wrapTilt(tiltY), 2); 
  Serial.print(",");
  Serial.print(wrapTilt(tiltZ), 2); 
  Serial.print(",");
  Serial.print(flightState);
  Serial.print("\n");
  Serial.flush();
  smartDelay(100);
}

double wrapTilt(double toWrap){
  return (toWrap - toWrap/PI)*180/PI;
}

void Reset(){

  ss.end();
  
  // Servo setup
  heatServo.attach(9); // top, yellow wire !facing arduino
  heatServo.write(CLOSE);
  delay(800);
  heatServo.detach(); 
  
  paraServo.attach(5); // middle, yellow wire !facing arduino
  paraServo.write(CLOSE);
  delay(800);
  paraServo.detach(); 

  relServo.attach(6);
  relServo.write(SCREW); // used to attach the heatshield
  delay(4000);
  relServo.write(OFF);
  delay(800);
  relServo.detach();

  ss.begin(9600);

  P0 = readBMP();

  lastCycle = millis();
  lastTelemCycle = lastCycle;
  flightState = 1;
}

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    readGPS();
  } while (millis() - start < ms);
}
