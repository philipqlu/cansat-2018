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
double dataTiltaX, dataTiltaY, dataTiltaZ, 
       dataTiltgX, dataTiltgY, dataTiltgZ, 
       dataTiltmX, dataTiltmY, dataTiltmZ;
uint8_t flightState;
uint16_t nextCycle;
double lastAlt, globalTimer;
uint32_t timeNow, lastCycle, last_telem_cycle;


// BMP180
SFE_BMP180 bmp180;
double P0;
#define LANDED 5  // liftoff/landed threshold *
#define STOPPED 1
#define DESCENT 2 // (lastAlt - dataAlt) threshold *
#define TIMER 5000/200 // globalTimer threshold; timing servo release and landed state *
#define LANDTIMER 5000/200

// Servos
Servo heatServo, paraServo, relServo;
#define OPEN 180 // *
#define CLOSE 0 // *
#define ON 180 // or 0 for opposite direction *
#define OFF 90

// GPS
TinyGPSPlus gps;
SoftwareSerial ss(PIN_RX, PIN_TX);

// MPU9250
MPU9250 IMU(Wire, 0x68);

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
//  while(IMU.begin() < 0){
//    Serial.println("IMU initialization unsuccessful");
//    Serial.println("Check IMU wiring or try cycling power");
//  }
//  // setting the accelerometer full scale range to +/-8G 
//  IMU.setAccelRange(MPU9250::ACCEL_RANGE_8G);
//  // setting the gyroscope full scale range to +/-500 deg/s
//  IMU.setGyroRange(MPU9250::GYRO_RANGE_500DPS);
//  // setting DLPF bandwidth to 20 Hz
//  IMU.setDlpfBandwidth(MPU9250::DLPF_BANDWIDTH_20HZ);
//  // setting SRD to 19 for a 50 Hz update rate
//  IMU.setSrd(19);

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
  lastCycle = millis();
  last_telem_cycle = lastCycle;
}

void loop(){
  getCommand();
  
  readGPS();
  
  timeNow = millis();
  if (lastCycle > timeNow) lastCycle = timeNow;
  if (last_telem_cycle > timeNow) last_telem_cycle = timeNow;
  
  if (timeNow - nextCycle >= lastCycle) {
    lastCycle = timeNow;
    readBMP();
    getFlightState();
    
    switch (flightState) {
      case 1: // prelaunch
              readVoltage();
              readMPU(); // *
              nextCycle = 1000;
              break;
    
      case 2: // ascending
              readVoltage();
              readMPU(); // *
              nextCycle = 200;
              break;
    
      case 3: // stabilizing
              heatServo.write(OPEN);
              readVoltage();
              readMPU(); // *
              nextCycle = 200;
              break;
    
      case 4: // release
              relServo.write(ON);       // write speed to turn
              paraServo.write(OPEN);    // write position
              if (globalTimer >= TIMER){
                relServo.write(OFF);    // write speed to stop turn
              }
              globalTimer ++;
              readVoltage();
              readMPU(); // *
              nextCycle = 200;
              break;
    
      case 5: //descent
              if (abs(dataAlt - lastAlt) < LANDED){
                globalTimer++;
              }
              readVoltage();
              readMPU(); // *
              nextCycle = 200;
              break;
    
      case 6: // landed
              landBuzzer();
              break;
    }
    if (timeNow - 1000 >= last_telem_cycle) {
      last_telem_cycle = timeNow;
      sendTelemetry();
    }
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

void readMPU(){
  
//  IMU.readSensor();  
//                
//  dataTiltaX = IMU.getAccelX_mss();
//  dataTiltaY = IMU.getAccelY_mss();
//  dataTiltaZ = IMU.getAccelZ_mss();
//  dataTiltgX = IMU.getGyroX_rads();
//  dataTiltgY = IMU.getGyroY_rads();
//  dataTiltgZ = IMU.getGyroZ_rads();
//  dataTiltmX = IMU.getMagX_uT();
//  dataTiltmY = IMU.getMagY_uT();
//  dataTiltmZ = IMU.getMagZ_uT();
  /*
  Serial.print("aX ");
  Serial.print(dataTiltaX, 6);
  Serial.print(",");
  Serial.print(" aY ");
  Serial.print(dataTiltaY, 6); 
  Serial.print(",");
  Serial.print(" aZ ");
  Serial.print(dataTiltaZ, 6); 
  Serial.print(",");
  Serial.print(" gX ");
  Serial.print(dataTiltgX, 6); 
  Serial.print(",");
  Serial.print(" gY ");
  Serial.print(dataTiltgY, 6); 
  Serial.print(",");
  Serial.print(" gZ ");
  Serial.print(dataTiltgZ, 6); 
  Serial.print(",");
  Serial.print(" mX ");
  Serial.print(dataTiltmX, 6);
  Serial.print(",");
  Serial.print(" mY ");
  Serial.print(dataTiltmY, 6);
  Serial.print(",");
  Serial.print(" mZ ");
  Serial.println(dataTiltmZ, 6);
  */
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
  Serial.print(dataTiltaX, 2);  // Accelerometer *
  Serial.print(",");
  Serial.print(dataTiltaY, 2); // Accelerometer *
  Serial.print(",");
  Serial.print(dataTiltaZ, 2); // Accelerometer *
  Serial.print(",");
  Serial.print(dataTiltgX, 2); // Gyroscope *
  Serial.print(",");
  Serial.print(dataTiltgY, 2); // Gyroscope *
  Serial.print(",");
  Serial.print(dataTiltgZ, 2); // Gyroscope *
  Serial.print(",");
  Serial.print(dataTiltmX, 2); // Magnometer *
  Serial.print(",");
  Serial.print(dataTiltmY, 2); // Magnometer *
  Serial.print(",");
  Serial.print(dataTiltmZ, 2); // Magnometer *
  Serial.print(",");
  Serial.print(flightState);
  Serial.print("\n");
  Serial.flush();
}

void Reset(){

  // Set defaults for servos
  heatServo.write(CLOSE);
  paraServo.write(CLOSE);
  relServo.write(OFF);

  P0 = readBMP();

  lastCycle = millis();
  last_telem_cycle = lastCycle;
}


