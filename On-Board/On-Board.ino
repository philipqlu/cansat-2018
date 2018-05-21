
/*  RSX for CANSAT-2016
    This is the onboard software for CANSAT-2016 competition (glider)
    Team 6734: RSX from University of Toronto / UTIAS
    Code Written by Eric Boszin and Phil
*/

#include "SFE_BMP180.h"
#include <Wire.h>
#include "TinyGPS.h"
#include <SoftwareSerial.h>
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
#define PARACHUTE_SERVO 9
#define MOSI 11
#define MISO 12
#define PIN_LED 13
#define PIN_BATT A3
#define PIN_SDA A4
#define PIN_SCL A5

// Thresholds for onboard logic - TO TEST
#define ALTITUDE_TOL 5
#define ALT_DEPLOY 670
#define DECS_THRESH 0 // set later, make a neg number
//#define EPS 0.01f
#define REL_DELAY 2
#define LAND_DELAY 5
#define HS_DELAY 3

// GPS
#define GPS_UPDATE_TIME 200

//Servos
#define OPEN 0
#define CLOSE 0 


///////////////////////////////// Variables //////////////////////////
// Global Variables
uint8_t flight_state, temp_timer;
float last_alt;
uint32_t time_now, last_cycle, last_telem_cycle;
uint16_t next_cycle;

// BMP180
SFE_BMP180 bmp180;
double pressure_baseline; // baseline pressure

// GPS
SoftwareSerial ss(PIN_TXD, PIN_RXD);
TinyGPS GPS;

//Servo
Servo heatshield_servo, HS_servo, parachute_servo;


//////////////////////////// Telemetry //////////////////////////
double data_altitude, data_pressure, data_voltage;
float data_gpsLAT, data_gpsLONG, data_gpsALT, init_gpsALT;
float data_temp;
uint8_t data_gpsNUM, data_comCNT; 
char data_gpsTime[8];
byte hour, minute, second,


// Madgwick algorithm for estimating orientation from MPU data
//Madgwick madgwick;

// Tilt
//MPU9250 IMU(Wire,0x68);

void setup() {
  //  Serial Begin
  Serial.begin(115200);

  // GPS
  ss.begin(9600);

  //LED
  pinMode(PIN_LED, OUTPUT);

  // BMP180 Init
  bmp180.begin();
  readBMP();
  pressure_baseline = readBMP();

  //Analog Pin Setup
  pinMode(PIN_BATT, INPUT);
  pinMode(PIN_SDA, INPUT);
  pinMode(PIN_SCL, INPUT);

  // Servo
    heatshield_servo.attach(5);
    HS_servo.attach(6);
    parachute_servo.attach(9);
    // Set defaults for servos
    heatshield_servo.write(CLOSE);
    HS_servo.write(CLOSE);
    parachute_servo.write(90);

  // Buzzer
  pinMode(PIN_BUZZER, OUTPUT);

  // Other Intitiations
  flight_state = 1;
  last_alt = 0;
  last_cycle = millis();
  last_telem_cycle = last_cycle;
  next_cycle = 0;
  data_comCNT = 0;
  temp_timer = 0;

//   Tilt sensor
  int MPUstatus;
  MPUstatus = IMU.begin();
//  while (MPUstatus < 0) {
//    Serial.println("IMU initialization unsuccessful");
//    Serial.println("Check IMU wiring or try cycling power");
//    Serial.print("Status: ");
//    Serial.println(MPUstatus);
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
    }
  }

  time_now = millis();

  // Legacy code: way of handling overflow?
  if (last_cycle > time_now) last_cycle = time_now;
  if (last_telem_cycle > time_now) last_telem_cycle = time_now;
  
  // Get flight status
  if (time_now - next_cycle >= last_cycle ) {
    last_cycle = time_now; //this is put here so that sensor reading and serial writing time are not taken into account

    read_gps(); // Update GPS
    read_tilt(); // Update tilt angles
    readBMP(); // Update temp and pressure values
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
        next_cycle = 200; // Monitor altitude at 5 Hz
        break;

      case 3: // stabilizing
        if(temp_timer == HS_DELAY*5) {
          HS_servo.write(OPEN); // write position
        }
        data_voltage = read_voltage();
        next_cycle = 200; // Monitor altitude at 5 Hz
        temp_timer ++;
        break;

      case 4: // release
        heatshield_servo.write(180); // write speed to turn
        parachute_servo.write(OPEN); // write position
        if (temp_timer == REL_DELAY){
          heatshield_servo.write(90); // write speed to stop turn
        }
        next_cycle = 1000;
        temp_timer ++;
        break;

      case 5: //descent
        if (abs(data_altitude - last_alt) < 0.01){
          temp_timer ++;
        }
        data_voltage = read_voltage();
        next_cycle = 1000;
        break;

      case 6: // land
        landBuzzer();
        break;
    }
    if (time_now - 1000 >= last_telem_cycle) {
      last_telem_cycle = time_now;
      send_telemetry();
    }
  }
}

void get_flight_state() {
  if (flight_state == 1 && ((data_altitude - last_alt) > ALTITUDE_TOL)) {
    flight_state = 2; //ascending
  }
  else if (flight_state == 2 && (data_altitude - last_alt) < DECS_THRESH) {
    flight_state = 3; //descending
  }
  else if (flight_state == 3 && data_altitude <= 300) {
    flight_state = 4; // Release
  }
  else if (flight_state == 4 && temp_timer >= REL_DELAY) {
    temp_timer = 0;
    flight_state = 5; //Descent
  }
  else if (flight_state == 5 && temp_timer >= LAND_DELAY) {
    flight_state = 6; //landed
  }
}

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

double readBMP()
{
  char status;
  double T, P;

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
          data_temp = T;
          data_pressure = P;
          return P;
        }
      }
    }
  }
}

void read_gps() {
   int year;
   byte month, day, hundredths;
   unsigned long age;
   bool newData = false;
   // For one second we parse GPS data and report some key values
   for (unsigned long start = millis(); millis() - start < 1000 ;)
   {
     while (ss.available())
     {
       char c = ss.read();
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
      data_gpsNUM = GPS.satellites();
      GPS.crack_datetime(&year,&month,&day,&hour,&minute,&second,&hundredths, &age); // GPS time
      
      sprintf(data_gpsTime, "%02d:%02d:%02d", hour, minute, second);
    }
}


void landBuzzer(){
  while(1){
    buzzer();
  }
}

void buzzer() {
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

void read_tilt() {
  float gx, gy, gz, ax, ay, az, mx, my, mz; 
  IMU.readSensor();
  ax = IMU.getAccelX_mss();
  ay = IMU.getAccelY_mss();
  az = IMU.getAccelZ_mss();
  gx = IMU.getGyroX_rads();
  gy = IMU.getGyroY_rads();
  gz = IMU.getGyroZ_rads();
  mx = IMU.getMagX_uT()*pow(10, 6); // TODO check if units are correct
  my = IMU.getMagY_uT()*pow(10, 6);
  mz = IMU.getMagZ_uT()*pow(10, 6);
  madgwick.update(gx, gy, gz, ax, ay, az, mx, my, mz);
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
  Serial.print(data_gpsLAT); // TODO see what the sigfigs are on this
  Serial.print(",");
  Serial.print(data_gpsLONG);
  Serial.print(",");
  Serial.print(data_gpsALT, 4);
  Serial.print(",");
  Serial.print(data_gpsNUM);
  Serial.print(",");
  Serial.print(madgwick.getRoll());
  Serial.print(",");
  Serial.print(madgwick.getPitch());
  Serial.print(",");
  Serial.print(madgwick.getYaw());
  Serial.print(",");
  Serial.print(flight_state);
  Serial.print("\n");
  Serial.flush();
}


