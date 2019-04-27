
//===============================================================================
//                    TEMPERATURES
//===============================================================================

void ReadTemps() {
  
  // Read temperature sensors
  // Loop time approx 800 ms
  // Loop time could be improved - library effectively waits 750ms for sensors to return
  // Needs some thought but could be better.
  
  if ((millis() - LastTempReadTime) > DELAY_READ_TEMPS || LastTempReadTime == 0) {
    
    LastTempReadTime = millis();
    TmpSensors.requestTemperatures();
    
    InsideTempAve  = 0.5 * (TmpSensors.getTempC(addrInsideTemp1)  + TmpSensors.getTempC(addrInsideTemp2));         //keep as integer type - not interested in the dps
    OutsideTempAve = 0.5 * (TmpSensors.getTempC(addrOutsideTemp1) + TmpSensors.getTempC(addrOutsideTemp2));
    
    sensor[TEMP_IN_AVE].value = PadInt(InsideTempAve,2," ");
    sensor[TEMP_OUT_AVE].value = PadInt(OutsideTempAve,2," ");

    }
    
 //   if (OutsideTempAve<MinOutside) MinOutside = OutsideTempAve;
 //   if (OutsideTempAve>MaxOutside) MaxOutside = OutsideTempAve;
 //   if (InsideTempAve<MinInside)   MinInside  = InsideTempAve;    
  //  if (InsideTempAve>MaxInside)   MaxInside  = InsideTempAve;   

  
  
 // IsDisplayChanged=true;
}

//===============================================================================
//                    VOLTAGES
//===============================================================================

void ReadVolts() {
  
  // Read battery voltages 
  // Loop time < 1 ms

  if ((millis() - LastVoltReadTime) > DELAY_READ_VOLTS || LastVoltReadTime == 0) {      
    LastVoltReadTime = millis();
    VoltsMain = analogRead(PIN_VOLTS_MAIN) * (5.0 / 1023.0) * (VOLT_BRIDGE_R1 + VOLT_BRIDGE_R2) / VOLT_BRIDGE_R2 * 10.0;   // convert to voltage and multiply by 10 so the result is an integer
    VoltsAux  = analogRead(PIN_VOLTS_AUX)  * (5.0 / 1023.0) * (VOLT_BRIDGE_R1 + VOLT_BRIDGE_R2) / VOLT_BRIDGE_R2 * 10.0;   // convert to voltage and multiply by 10 so the result is an integer
  }
  
  dtostrf(VoltsMain/10.0,4,1,chrBuffer);
  sensor[VOLTS_MAIN].value = chrBuffer;

  dtostrf(VoltsAux/10.0,4,1,chrBuffer);
  sensor[VOLTS_AUX].value = chrBuffer;

  
  
 // IsDisplayChanged=true;
}

//===============================================================================
//                    READ AND PROCESS ACCL AND GYRO DATA
//===============================================================================

void ReadAccls() {

  // Read accelerometers and gyros, and process data 
  // Loop time max 5 ms
  
  if (IsGyroDataReady()) {          // Check that gyros are ready

    getGyroData (GyroRaw);          // Get data from gyros
    getAccelData(AcclRaw);          // Get data from accls
    
    if (LastGyroReadTime < GyroReadTime) {    // micros() returns time in microseconds but rolls over once every approx 70seconds.  So check if rollover has happened and skip loop if so.
      dt = GyroReadTime - LastGyroReadTime;   // Calculate time since last data 
  
      // Calculate pitch and roll angle based on accelerometer readings
      AcclAnglePitch = asin(AcclRaw[AcclPitchAxis] * ACCL_GAIN * AcclCalibr[AcclPitchAxis] + AcclOffset[AcclPitchAxis] + AcclInstal[AcclPitchAxis]) * (180.0 / M_PI);
      AcclAngleRoll  = asin(AcclRaw[AcclRollAxis]  * ACCL_GAIN * AcclCalibr[AcclRollAxis]  + AcclOffset[AcclRollAxis]  + AcclInstal[AcclRollAxis] )  * (180.0 / M_PI);
   
      // Calculate pitch and roll rate based on gyro readings
      GyroRatePitch = -1 * (GyroRaw[GyroPitchAxis] - GyroOffset[GyroPitchAxis]) * GYRO_GAIN;    // Note (-1) which corrects axis orientation for pitch axis only
      GyroRateRoll  =  1 * (GyroRaw[GyroRollAxis]  - GyroOffset[GyroRollAxis] ) * GYRO_GAIN;
      GyroAnglePitch = GyroAnglePitch + GyroRatePitch * dt / 1000000.0;                         // Only reqd for debugging
      GyroAngleRoll  = GyroAngleRoll  + GyroRateRoll  * dt / 1000000.0;
      
      // Combine angle and rate in complimentary filter
      if (AnglePitch == -99) {
        AnglePitch    = AcclAnglePitch;  
        AngleRoll     = AcclAngleRoll;
      } else {
        AnglePitch    = FILTER_CONSTANT * ( AnglePitch + GyroRatePitch * dt / 1000000.0 ) + (1 - FILTER_CONSTANT) * AcclAnglePitch;  
        AngleRoll     = FILTER_CONSTANT * ( AngleRoll  + GyroRateRoll  * dt / 1000000.0 ) + (1 - FILTER_CONSTANT) * AcclAngleRoll;
      }

      sensor[PITCH_ANGLE].value = PadInt((int)AnglePitch,3," ");
      sensor[ROLL_ANGLE].value  = PadInt((int)AngleRoll,3," ");
      
  
      // Calculate pitch and roll gradients in percent
     // if (AnglePitch > 0) GradientPitch = tan( AnglePitch * M_PI / 180 ) * 100;
     // else GradientPitch = -tan( AnglePitch * M_PI / 180 ) * 100;
     // if (AngleRoll > 0)  GradientRoll  = tan( AngleRoll * M_PI / 180 ) * 100;
     // else GradientRoll  = -tan( AngleRoll * M_PI / 180 ) * 100;
  
    }  
  }
  
  LastGyroReadTime = GyroReadTime;        // Capture current read time
  
 // IsDisplayChanged=true;
  
}

//===============================================================================
//                    ACCELEROMETER ROUTINES
//===============================================================================

void initAcc() {    //Turning on the ADXL345, by default the device is in +-2g range reading

  writeTo(ACC, 0x2D, 0   );  // 0x2D = POWER_CTL register: clearing all bits puts in standby mode
  writeTo(ACC, 0x2D, 16  );  // 0x2D = POWER_CTL register: 16 = 00010000 sets auto-sleep mode=ON
  writeTo(ACC, 0x2D, 8   );  // 0x2D = POWER_CTL register: 8  = 00001000 sets to measurement mode
  writeTo(ACC, 0x2C, 0x0A);  // 0x2C = BW_RATE register: set data rate at 100Hz to match gyro
  
}

void getAccelData(int * result) {

  int regAddress = 0x32;                      //first axis-acceleration-data register on the ADXL345
  byte buff[A_TO_READ];
  
  readFrom(ACC, regAddress, A_TO_READ, buff); //read the acceleration data from the ADXL345
  result[0] = (((int)buff[1]) << 8) | buff[0];   
  result[1] = (((int)buff[3]) << 8) | buff[2];
  result[2] = (((int)buff[5]) << 8) | buff[4];

}


//===============================================================================
//                    GYRO ROUTINES
//===============================================================================

void initGyro() {                          //initializes the gyroscope
  writeTo(GYRO, 0x15, G_SMPLRT_DIV);   
  writeTo(GYRO, 0x16, G_DLPF_FS);    
  writeTo(GYRO, 0x17, G_INT_CFG);
  writeTo(GYRO, 0x3E, G_CLK_SEL);  
}

void calibrateGyro() {
  
  CalibStartTime = millis();
  
  while (millis() - CalibStartTime < CALIB_SAMPLE_TIME) { 
    if (IsGyroDataReady()) {                 // Check that gyros are ready  
      getGyroData (GyroRaw);                 // Get data from gyros
      GyroOffset[0] += GyroRaw[0];
      GyroOffset[2] += GyroRaw[2];
      CalibCounter++;
    }
   }
   
  GyroOffset[0] /= CalibCounter;    // calculate gyro offset (bias)
  GyroOffset[2] /= CalibCounter;    // calculate gyro offset (bias)
  
}

void getGyroData(int * result) {
  //Serial.println("getGyroData...");   
  // Read data from gyroscope
  //Serial.print("a"); 
  int regAddress = 0x1B;
  //Serial.print("b");
  byte buff[G_TO_READ];
  //Serial.print("c");  
  //Serial.print("rfg...");    
  readFrom(GYRO, regAddress, G_TO_READ, buff); //read the gyro data from the ITG3200
  //Serial.print("d");  
  GyroReadTime = micros();                    // get gyro read time - ststement as cose to actual read as possible to avoid interrupt.
 // Serial.print("a"); 
  result[0] = (buff[2] << 8) | buff[3];
 // Serial.print("b"); 
  result[1] = (buff[4] << 8) | buff[5];
 // Serial.print("c"); 
  result[2] = (buff[6] << 8) | buff[7];
 // Serial.print("d"); 
  result[3] = (buff[0] << 8) | buff[1]; // temperature
 // Serial.println("e");
}

bool IsGyroDataReady() {
  byte _buff[1];  
  readFrom(GYRO, 0x1A, 1, _buff); 
  return _buff[0] & 0x01;
}

//===============================================================================
//                    READ AND WRITE
//===============================================================================

void readFrom(int DEVICE, byte address, int num, byte buff[]) {
  
  //reads num bytes starting from address register on ACC in to buff array
  Wire.beginTransmission(DEVICE); //start transmission to ACC 
  Wire.write(address);            //sends address to read from
  Wire.endTransmission();         //end transmission

  Wire.beginTransmission(DEVICE); //start transmission to ACC
  Wire.requestFrom(DEVICE, num);  // request 6 bytes from ACC <-------THIS IS THE PROBLEM!!!!!

  int i = 0;
  while(Wire.available()){        //ACC may send less than requested (abnormal)
    buff[i] = Wire.read();        // receive a byte
    i++;
  }
  Wire.endTransmission();         //end transmission

}

void writeTo(int DEVICE, byte address, byte val) {

  //Writes a value to address register on ACC
  Wire.beginTransmission(DEVICE); //start transmission to ACC 
  Wire.write(address);            // send register address
  Wire.write(val);                // send value to write
  Wire.endTransmission();         //end transmission
}

