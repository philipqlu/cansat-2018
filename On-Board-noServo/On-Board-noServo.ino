
/*  RSX for CANSAT-2016
    This is the onboard software for CANSAT-2016 competition (glider)
    Team 6734: RSX from University of Toronto / UTIAS
    Code Written by Johnny Wang - zeyang.wang@mail.utoronto.ca
*/

#include <SFE_BMP180.h>
#include <Wire.h>
#include <TinyGPS.h>
#include <SoftwareSerial.h>
#include <Adafruit_VC0706.h>
#include <SPI.h>
#include <SD.h>
#include <MPU9250.h>  // https://github.com/bolderflight/MPU9250

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

// Servo Positions - TO TEST
#define

// Altitude - fix
#define ALTITUDE_TOL 5
#define ALT_DEPLOY 410

// GPS
#define GPS_UPDATE_TIME 1000

/////////////////////////////////////////// Variables //////////////////////////
// Global Variables
uint8_t flight_state;
float last_alt;
uint32_t time_now, last_cycle, last_telem_cycle;
uint16_t next_cycle;

// BMP180
SFE_BMP180 bmp180;
double pressure_baseline; // baseline pressure

// GPS
SoftwareSerial serial_gps(gps_TX, gps_RX);
TinyGPS GPS;

// MPU9250 Sensor
MPU9250 IMU(Wire,0x68);
int MPUstatus;


/////////////////////////////////////////// Telemetry //////////////////////////
double data_altitude, data_pressure, data_voltage;
float data_gpsLAT, data_gpsLONG, data_gpsALT, data_gpsTime, init_gpsALT;
float data_pitot, data_temp, data_gpsSPEED;
float data_tiltX, data_tiltY, data_tiltZ;
uint8_t data_gpsNUM, data_comCNT;
char data_gpsTime[8];
byte month, day, hour, minute, second, hundredths;

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

  // Servo
  pinMode(HEATSHIELD_SERVO, OUTPUT);
  pinMode(COVER_SERVO, OUTPUT);
  pinMode(PARASHOOT_SERVO, OUTPUT);
  // Set defaults for servos
  servoPosition(, );
  servoPosition(, );
  servoPosition(, );

  // Buzzer
  pinMode(PIN_BUZZER, OUTPUT);
  pt();

  // Other Intitiations
  flight_state = 0;
  last_alt = 0;
  last_cycle = millis();
  last_telem_cycle = last_cycle;
  next_cycle = 0;
  data_comCNT = 0;

  // Tilt sensor
  MPUstatus = IMU.begin();
  if (MPUstatus < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(MPUstatus);
    while(1) {}
  }
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
  if (flight_state == 5) {
    servoPosition(RWING_SERVO, OPEN_RWING);
    servoPosition(LWING_SERVO, OPEN_LWING);
  }

  // Get flight status
  if (time_now - next_cycle >= last_cycle ) {
    last_cycle = time_now; //this is put here so that sensor reading and serial writing time are not taken in account of.

    data_pressure = (float) getPressure();
    data_altitude = (float) bmp180.altitude(data_pressure, pressure_baseline);
    get_flight_state();

    last_alt = data_altitude;

    ////////////////// switch case depending of flight status /////////////////////
    switch (flight_state) {
      case 1: // waiting
        data_pitot = 0;
        data_temp = read_temp();
        data_voltage = read_voltage();
        next_cycle = 1000;
        break;

      case 2: // ascending
        data_pitot = 0;
        data_temp = read_temp();
        data_voltage = read_voltage();
        next_cycle = 200;
        break;

      case 3: // descending
        //descending
        data_pitot = 0;
        data_temp = read_temp();
        data_voltage = read_voltage();
        next_cycle = 200;
        break;

      case 4: // deploying
        //deploy
        servoPosition(RWING_SERVO, FOLD_RWING);
        servoPosition(LWING_SERVO, FOLD_LWING);
  //      delay(200);
        servoPosition(LATCHCAM_SERVO, OPEN_LATCH);
        smartdelay(1000);
        servoPosition(RWING_SERVO, OPEN_RWING);
        servoPosition(LWING_SERVO, OPEN_LWING);
        smartdelay(200);
        servoPosition(LATCHCAM_SERVO, CAM_ZERO);
        camera_angle = CAM_ZERO;
        break;

      case 5: //gliding
        //glide
        data_pitot = read_pitot();
        data_temp = read_temp();
        data_voltage = read_voltage();
        next_cycle = 1000;
        break;

      case 6: // land
        data_pitot = 0;
        data_temp = 0;
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

void get_flight_state() {
  if (flight_state == 0 && abs(data_altitude) <= ALTITUDE_TOL) {
    flight_state = 1; // waiting
  }
  else if ((flight_state == 1 && ((data_altitude - last_alt) > ALTITUDE_TOL || data_altitude > 400))) {
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

void interpret_command(String command) {
  String com;

  com = command.substring(0, 1);
  if (com.equals("s")) {
    take_pic();
  }
  else if (com.equals("m")) {
    String theta;
    theta = command.substring(2);
    int angle = theta.toInt();

    servoPosition(LATCHCAM_SERVO, angle);
    camera_angle = angle;
  }
  else if (com.equals("f")) {
    String com2;
    com2 = command.substring(2);
    int force_com = com2.toInt();
    if (force_com == 1) {
      flight_state = 3;
    }
    else if (force_com == 2) {
      flight_state = 6;
    }
  }
}

float read_voltage() {
  double voltage = analogRead(PIN_BATT);
  return (float) voltage / 102.3;
}

/*
 * Returns pressure from BMP180 in hPa (1 hPa = 100 Pa).
 */
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
      
      sprintf(sz, "%02d:%02d:%02d", hour, minute, second);
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

void take_pic() {
  cameraconnection.listen();
  if (! cam.takePicture()) {
    serial_gps.listen();
    return;
  }
  else {
    data_comCNT += 1;
    char filename[8] = "00.JPG";
    filename[0] = '0' + data_comCNT / 10;
    filename[1] = '0' + data_comCNT % 10;

    File imgFile = SD.open(filename, FILE_WRITE);
    uint16_t jpglen = cam.frameLength();
    byte wCount = 0; // For counting # of writes
    while (jpglen > 0) {
      // read 32 bytes at a time;
      uint8_t *buffer;
      uint8_t bytesToRead = min(32, jpglen); // change 32 to 64 for a speedup but may not work with all setups!
      buffer = cam.readPicture(bytesToRead);
      imgFile.write(buffer, bytesToRead);
      if (++wCount >= 64) { // Every 2K, give a little feedback so it doesn't appear locked up
        wCount = 0;
      }
      jpglen -= bytesToRead;
    }

    imgFile.close();
  }
  serial_gps.listen();
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

void servoPosition(int servopin, int angle)
{
  int pulsemicros = (int)11 * angle + 490;
  for (int i = 0; i < 32; i++) { //gets about 90 degrees movement, call twice or change i<16 to i<32 if 180 needed
    digitalWrite(servopin, HIGH);
    delayMicroseconds(pulsemicros);
    digitalWrite(servopin, LOW);
    delay(25);
  }
}

void read_tilt() {
  data_tiltX = IMU.getGyroX_rads();
  data_tiltY = IMU.getGyroY_rads();
  data_tiltZ = IMU.getGyroZ_rads();  
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
  Serial.print(data_tiltX);
  Serial.print(",");
  Serial.print(data_tiltY);
  Serial.print(",");
  Serial.print(data_tiltZ);
  Serial.print(",");
  Serial.print(flight_state);
  Serial.print("\n");
  Serial.flush();

}


