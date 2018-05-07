
/*  RSX for CANSAT-2016
    This is the onboard software for CANSAT-2016 competition (glider)
    Team 6734: RSX from University of Toronto / UTIAS
    Code Written by Eric Boszin and Phil
*/

// To do: deffine servos positions and set to those positions

#include <SFE_BMP180.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_VC0706.h>
#include <SPI.h>
#include <SD.h>
#include "MPU9250.h"// https://github.com/bolderflight/MPU9250
#include <Servo.h>
#include <math.h>
#include "MadgwickAHRS.h" // https://github.com/arduino-libraries/MadgwickAHRS

//////////////////////////// constants //////////////////////////

// pins
#define PIN_RXD 0
#define PIN_TXD 1
#define PIN_BUZZER 3
#define HEATSHIELD_SERVO 5
#define COVER_SERVO 6
#define PIN_RX 7
#define PIN_TX 8
#define PARASHOOT_SERVO 9
#define MOSI 11
#define MISO 12
#define PIN_LED 13
#define PIN_BATT A3
#define PIN_SDA A4
#define PIN_SCL A5

// Servo Positions - TO TEST ------------------------------
#define

// Altitude - fix
#define ALTITUDE_TOL 5
#define ALT_DEPLOY 410

// GPS
#define GPS_UPDATE_TIME 1000


///////////////////////////////// Variables //////////////////////////
// Global Variables
uint8_t flight_state;
float last_alt;
uint32_t time_now, last_cycle, last_telem_cycle;
uint16_t next_cycle;
Servo heatshield_servo;
Servo HS_servo;
Servo parashoot_servo;

// BMP180
SFE_BMP180 bmp180;
double pressure_baseline; // baseline pressure

// GPS
SoftwareSerial serial_gps(gps_TX, gps_RX);
TinyGPS GPS;


/////////////////////////////////////////// Telemetry //////////////////////////
double data_altitude, data_pressure, data_voltage;
float data_gpsLAT, data_gpsLONG, data_gpsALT, data_gpsTime, init_gpsALT;
float data_pitot, data_temp, data_gpsSPEED;
float gx, gy, gz, ax, ay, az, mx, my, mz; // IMU/MPU data

uint8_t data_gpsNUM, data_comCNT;
char data_gpsTime[8];
byte month, day, hour, minute, second, hundredths;

// Madgwick algorithm for estimating orientation from MPU data
Madgwick madgwick;

void setup() {
  //  Serial Begin
  Serial.begin(9600);

  // BMP180 Init
  bmp180.begin();
  pressure_baseline = getPressure();

  //Analog Pin Setup
  pinMode(PIN_BATT, INPUT);
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);


/****************************************************************
  // Servo
    heatshield_servo.attach(5);
    HS_servo.attach(6);
    parashoot_servo.attach(9);
//  pinMode(HEATSHIELD_SERVO, OUTPUT);
//  pinMode(COVER_SERVO, OUTPUT);
//  pinMode(PARASHOOT_SERVO, OUTPUT);
  // Set defaults for servos
//  servoPosition(, );
//  servoPosition(, );
//  servoPosition(, );
******************************************************************/

  // Buzzer
  pinMode(PIN_BUZZER, OUTPUT);
  pt();

  // Other Intitiations
  flight_state = 1;
  last_alt = 0;
  last_cycle = millis();
  last_telem_cycle = last_cycle;
  next_cycle = 0;
  data_comCNT = 0;

//   Tilt sensor
  MPU9250 IMU(Wire,0x68);
  MPUstatus = IMU.begin();
//  if (MPUstatus < 0) {
//    Serial.println("IMU initialization unsuccessful");
//    Serial.println("Check IMU wiring or try cycling power");
//    Serial.print("Status: ");
//    Serial.println(MPUstatus);
//    while(1) {}
//  }
  
}

void loop() {

  // Command Received
  if (Serial.available()) {
    String command;
    while (Serial.available()) {
      char c = Serial.read();
      if (c == '\n') {
        interpret_command(command);
        command = "";
      }
      else {
        command += c;
      }
      smartdelay(10);
    }
  }

  time_now = millis();
  
  if (last_cycle > time_now) last_cycle = time_now;
  if (last_telem_cycle > time_now) last_telem_cycle = time_now;

  read_gps();
  read_tilt();
  
  // Get flight status
  if (time_now - next_cycle >= last_cycle ) {
    last_cycle = time_now; //this is put here so that sensor reading and serial writing time are not taken in account of.

    data_temp = (float) read_temp();
    data_pressure = (float) getPressure();
    data_altitude = (float) bmp180.altitude(data_pressure, pressure_baseline);
    get_flight_state();

    last_alt = data_altitude;

    ////////////////// switch case depending of flight status /////////////////////
    switch (flight_state) {
      case 1: // prelaunch
        data_voltage = read_voltage();
        next_cycle = 1000;
        break;

      case 2: // ascending
        data_voltage = read_voltage();
        next_cycle = 200;
        break;

      case 3: // stabalizing
        HS_servo.write(); // write position
        data_voltage = read_voltage();
        next_cycle = 200;
        break;

      case 4: // release
        heatshield_servo.write(); // write speed to turn
        smartdelay(1000);
        parashoot_servo.write(); // write position
        break;

      case 5: //descent
        data_voltage = read_voltage();
        next_cycle = 1000;
        break;

      case 6: // land
        data_voltage = read_voltage();
        pt();
        next_cycle = 1000;
        break;
    }
    if (time_now - 1000 >= last_telem_cycle) {
      last_telem_cycle = time_now;
      send_telemetry();
    }
  }
}

/****************************************************************
void get_flight_state() {
  if ((flight_state == 1 && ((data_altitude - last_alt) > ALTITUDE_TOL || data_altitude > 400))) { // think about the conditins to go t second state
    flight_state = 2; //ascending
  }
  else if (flight_state == 2 && (data_altitude - last_alt) < 0 && data_altitude >= ALT_DEPLOY) {
    flight_state = 3; //descending
  }
  else if (flight_state == 2 && abs(data_altitude) <= ALTITUDE_TOL) {
    flight_state = 1; // waiting (fails start back to waiting)
  }
  else if (flight_state == 3 && (data_altitude <= ALT_DEPLOY || last_alt <= ALT_DEPLOY)) {
    flight_state = 4; //deploying
  }
  else if (flight_state == 4) {
    flight_state = 5; //gliding
  }
  else if (flight_state == 5 && (data_altitude - last_alt) >= 0) {
    flight_state = 6; //landed
  }
}
******************************************************************/

void interpret_command(String command) {
  String com;

  com = command.substring(0, 1);
  if (com.equals("f")) {
    String com2;
    com2 = command.substring(2);
    int force_com = com2.toInt();
    if (force_com == 0) {
      setup();
    } if (force_com == 1) {
      flight_state = 1;
    } else if (force_com == 2) {
      flight_state = 2;
    } else if (force_com == 3) {
      flight_state = 3;
    } else if (force_com == 4) {
      flight_state = 4;
    } else if (force_com == 5) {
      flight_state = 5;
    } else if (force_com == 6) {
      flight_state = 6;
    }
  }
}

float read_voltage() {
  double voltage = analogRead(PIN_BATT);
  return (float) voltage / 102.3;
}

double getPressure()
{
  char status;
  double T, P, p0, a;

  status = bmp180.startTemperature();
  if (status != 0)
  {
    delay(status);
    status = bmp180.getTemperature(T);
    if (status != 0)
    {
      status = bmp180.startPressure(3);
      if (status != 0)
      {
        delay(status);
        status = bmp180.getPressure(P, T);
        if (status != 0)
        {
          return (P);
        }
      }
    }
  }
}

double read_temp()
{
  char status;
  double T, P, p0, a;

  status = bmp180.startTemperature();
  if (status != 0)
  {
    delay(status);
    status = bmp180.getTemperature(T);
    if (status != 0)
    {
      status = bmp180.startPressure(3);
      if (status != 0)
      {
        delay(status);
        status = bmp180.getPressure(P, T);
        if (status != 0)
        {
          return (T);
        }
      }
    }
  }
}

void read_gps() {

  bool newData = false;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 500;)
  {
    while (serial_gps.available())
    {
      char c = serial_gps.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (GPS.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  if (newData)
  {
      unsigned long age;
      GPS.f_get_position(&data_gpsLAT, &data_gpsLONG, &age);
      if(data_gpsALT == -1){ 
        data_gpsALT = GPS.f_altitude();
        init_gpsALT = data_gpsALT;
      }
      data_gpsALT = GPS.f_altitude()-init_gpsALT;
      data_gpsSPEED = GPS.f_speed_mps();
      data_gpsNUM = GPS.satellites();
      gps.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths); // GPS time
      
      sprintf(data_gpsTime, "%02d:%02d:%02d", hour, minute, second);
  }
}


static void smartdelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    read_gps();
  } while (millis() - start < ms);
}

void pt() {
  uint16_t et = 0;
  while (et < 500) {
    digitalWrite(PIN_BUZZER, HIGH);
    delayMicroseconds(900);

    // DOWN
    digitalWrite(PIN_BUZZER, LOW);
    delayMicroseconds(900);
    et += 2;
  }
}

//void servoPosition(int servopin, int angle)
//{
//  int pulsemicros = (int)11 * angle + 490;
//  for (int i = 0; i < 32; i++) { //gets about 90 degrees movement, call twice or change i<16 to i<32 if 180 needed
//    digitalWrite(servopin, HIGH);
//    delayMicroseconds(pulsemicros);
//    digitalWrite(servopin, LOW);
//    delay(25);
//  }
//}

void read_tilt() {
  IMU.readSensor();
  ax = IMU.getAccelX_mss();
  ay = IMU.getAccelY_mss();
  az = IMU.getAccelZ_mss();
  gx = IMU.getGyroX_rads();
  gy = IMU.getGyroY_rads();
  gz = IMU.getGyroZ_rads();
  mx = IMU.getMagX_uT()*pow(10, 6);
  my = IMU.getMagY_uT()*pow(10, 6);
  mz = IMU.getMagZ_uT()*pow(10, 6);
  madgwick.update(gx, gy, gz, ax, ay, az, mx, my, mz);
//  data_tiltX = madgwick.roll;
//  data_tiltY = madgwick.pitch;
//  data_tiltZ = madgwick.yaw;  
}

void send_telemetry() {

  Serial.print(data_altitude, 4);
  Serial.print(",");
  Serial.print(data_pressure / 10, 4);
  Serial.print(",");
  Serial.print(data_temp, 3);
  Serial.print(",");
  Serial.print(data_voltage, 2);
  Serial.print(",");
  Serial.print(data_gpsTime);
  Serial.print(",");
  Serial.print(data_gpsLAT, 4);
  Serial.print(",");
  Serial.print(data_gpsLONG, 4);
  Serial.print(",");
  Serial.print(data_gpsALT, 4);
  Serial.print(",");
  Serial.print(data_gpsNUM);
  Serial.print(",");
  Serial.print(madgwick.roll);
  Serial.print(",");
  Serial.print(madgwick.pitch);
  Serial.print(",");
  Serial.print(madgwick.yaw);
  Serial.print(",");
  Serial.print(flight_state);
  Serial.print("\n");
  Serial.flush();

}


