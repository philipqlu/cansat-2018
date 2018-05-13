

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
  else if (flight_state == 4 && REL_timer == REL_DELAY) {
    flight_state = 5; //Descent
  }
  else if (flight_state == 5 && descent_timer >= LAND_DELAY) {
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


