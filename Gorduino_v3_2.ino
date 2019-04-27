
// ------------------------------------------------------------------------------
#define SOFT_VERSION F("3.2")  
//    rework display logic to avoid corrupted screen errors
// ------------------------------------------------------------------------------
//#define SOFT_VERSION F("3.0")  
//    pitch and roll angles displayed at low speed
//    small change button logic
//    tidy up code and commenting
// ------------------------------------------------------------------------------
//#define SOFT_VERSION F("2.2C")   
//    updated time library
//    accl reading outside of queit time loop to minimise dt
// ------------------------------------------------------------------------------
//#define SOFT_VERSION F("2.2B")   
//    ints and longs storing times to unsigned
//    quiet time structure for reading sensors to improve gps data
//    gps initialisation to control sentence output and rate
// ------------------------------------------------------------------------------
//#define SOFT_VERSION F("2.2A")   
//    remove unused functions eg bigfont time                                  
//    implementation of F() macro to reduce RAM useage 
//    **NOTE BST code is commented out - needs reimplementing
//    return to summary screen after long press on any other screen
// ------------------------------------------------------------------------------

//===============================================================================
//-------------------------------------------------------------------------------
//
//                             DEFINE VARIABLES
//
//-------------------------------------------------------------------------------
//===============================================================================
 
  // Library Definitions
  #include      <Wire.h>                                             // library for I2C communication with accelerometer
  #include      <LiquidCrystal.h>                                    // library for communicating with lcd display
  #include      <OneWire.h>                                          // library for onewire communication with digital temp gauges
  #include      <DallasTemperature.h>                                // library for digital temp gauges
  #include      <TinyGPS++.h>                                        // library for controlling gps module
  #include      <SoftwareSerial.h>                                   // software serial emulator for control of gps
  #include      <Streaming.h>                                        // enables inline send to lcd commands using <<
  #include      <Time.h>                                             // time functions


#define TEMP_IN_AVE       0
#define TEMP_OUT_AVE      1
#define PITCH_ANGLE       2
#define ROLL_ANGLE        3
#define VOLTS_MAIN        4
#define VOLTS_AUX         5
#define TIME              6
#define TIME_ZONE         7
#define SPEED_MPH         8
#define COURSE_DEGS       9
#define ALTITUDE          10

#define SENSOR_ARRAY_SIZE 11

struct sensor_reading {
   char*   value;
   boolean isdisplayed;
   int     row;
   int     col;
} sensor[SENSOR_ARRAY_SIZE];

boolean IsDisplayChanged = true;

  
  // Define Times and Timers
  #define              TIME_FOR_LONG_PRESS   1000                           // time required for a long button press
  #define              TIME_BACKLIGHT_HIGH   8000                           // length of time backlight will remain high after button press
  #define              DELAY_DEBOUNCE        35                             // button debounce delay
  #define              DELAY_READ_TEMPS      60000                          // delay between reading temperature sensors
  #define              DELAY_READ_VOLTS      1100                           // delay between reading voltages
//  #define              DELAY_SCREEN_UPDATE   500                            // delay between updating lcd display
  #define              DELAY_GPS_STANDBY     180000                         // length of time gps not used for when engine is off before switching off

  // -- VOLTAGE VARIABLES --
  #define              PIN_VOLTS_MAIN        A0                             // defin pin to read main battery voltage
  #define              PIN_VOLTS_AUX         A1                             // defin pin to read aux battery voltage
  #define              VOLT_BRIDGE_R1        2400                           // resistance used in voltage bridge
  #define              VOLT_BRIDGE_R2        1200                           // resistance used in voltage bridge
  #define              CHARGING_VOLTS        12.6                           // define the voltage over which it is assumed that the engine is running
  unsigned int         LastVoltReadTime      = 0;                           // time that the voltages were last checked    
  unsigned int         VoltsMain;                                           // variable to hold the measured main battery voltage
  unsigned int         VoltsAux;                                            // variable to hold the measured aux battery voltage
  boolean              IsEngineRunning       = false;                       // boolean variable to flag whether engine is running
  
  // -- LCD VARIABLES --
  #define              PIN_LCD_CONTRAST      10                              // define digital pin for contrast control
  #define              PIN_LCD_BACKLIGHT     6                               // define digital pin for backlight control
  #define              LCD_CONTRAST          20                              // define contrast level
  #define              LCD_LIGHT_LOW         20                              // define backlight brightness when set to low
  #define              LCD_LIGHT_HIGH        80                              // define backlight brightness when set to high 
  unsigned long        LastDisplayTime       = 0;                            // time that the display was last updated  
  unsigned int         BackLightIndex        = 2;                            // defines backlight status: 2 = bright; 1 = dim; 0 = off
  LiquidCrystal        lcd(9, 8, 5, 4, 3, 2);                                // define pins for lcd display
      
  // -- TEMPERATURE VARIABLES --
  #define              PIN_DIGITAL_TEMP      7                               // set digital pin for temperature readings
  unsigned long        LastTempReadTime      = 0;                            // time that the temperatures were last checked  
//  int                  InsideTemp1;                                          // variable to hold measured temperature  
//  int                  InsideTemp2;                                          // variable to hold measured temperature 
//  int                  OutsideTemp1;                                         // variable to hold measured temperature
//  int                  OutsideTemp2;                                         // variable to hold measured temperature
  int                  InsideTempAve;                                        // variable to hold averaged temperature
  int                  OutsideTempAve;                                       // variable to hold averaged temperature
//  int                  MaxInside             = -99;
//  int                  MinInside             = 99;
//  int                  MaxOutside            =-99;
//  int                  MinOutside            = 99;  
  OneWire              ds(PIN_DIGITAL_TEMP);                                 // create instance of onewire library for communication with onewire devices
  DallasTemperature    TmpSensors(&ds);                                      // create instace of dallastemp library for reading temperature sensors
  DeviceAddress        addrOutsideTemp1      = {0x28, 0x6B, 0x55, 0xC5, 0x03, 0x00, 0x00, 0x3E};    // long grey lead probe   OUTSIDE FRONT
  DeviceAddress        addrOutsideTemp2      = {0x28, 0x7E, 0x5A, 0xC5, 0x03, 0x00, 0x00, 0xE1};    // long grey lead probe   OUTSIDE BACK 
  DeviceAddress        addrInsideTemp1       = {0x28, 0x31, 0x8F, 0x74, 0x04, 0x00, 0x00, 0x58};    // 5m black cable         INSIDE DOWN
  DeviceAddress        addrInsideTemp2       = {0x28, 0x0F, 0x85, 0xD0, 0x04, 0x00, 0x00, 0xBC};    // 10m black cable        INSIDE UP
 
  // -- ACCELEROMETER SETTINGS --
  #define              ACC                   0x53                            // ADXL345 ACC address
  #define              A_TO_READ             6                               // num of bytes we are going to read each time (two bytes for each axis)
  #define              ACCL_GAIN             0.0039                          // accelerometer gain
  #define              GYRO                  0x68                            // gyro address, binary = 11101001 when AD0 is connected to Vcc (see schematics of your breakout board)
  #define              G_SMPLRT_DIV          0x04                            // SMPLRT_DIV=7 (09=00001001) - sample rate is 1KHz/(9+1)=100Hz
  #define              G_DLPF_FS             0x1D                            // FS_SEL=+/-2000deg/sec, DLPF_CFG=5Hz,1KHz (1D=00011101)
  #define              G_INT_CFG             0x01                            // RAW_RDY_EN .. enabled
  #define              G_CLK_SEL             0x11                            // 11=00010001, should disable y axis and use gyro x axis for clock sync 
  #define              G_PWR_MGM             0x00                            // default power - note individual gyros can be turned off if not used
  #define              G_TO_READ             8                               // 2 bytes for each axis x, y, z
  #define              CALIB_SAMPLE_TIME     300                             // time over which to calibrate gyro
  #define              GYRO_GAIN             1/14.375                        // gyro gain
  #define              FILTER_CONSTANT       0.995                           // complimentary filter constant
  #define              PITCH_WARN_THRESH     10                              // Pitch threshold at which angles data is flagged on primary screen (in degrees)
  #define              ANGLES_DISPL_THRESH    2                              // speed in mph below which angle data is shown instead of speed/direction
  unsigned int         CalibStartTime;                                       // time at which gyro calibration subroutine is called 
  const int            AcclPitchAxis        = 0;                             // Define the axis for Roll angle accl
  const int            AcclRollAxis         = 2;                             // Define the axis for roll angle accl
  const int            GyroPitchAxis        = 2;                             // Define the axis for Roll angle gyro
  const int            GyroRollAxis         = 0;                             // Define the axis for roll angle gryo
  const float          AcclCalibr[3]        = {+0.999219, +0.0, +0.968097};  // calibrated accelerometer gains - axis 2 not reqd
  const float          AcclOffset[3]        = {-0.048948, +0.0, +0.031356};  // calibrated accelerometer offsets
  const float          AcclInstal[3]        = {+0.022994, +0.0, -0.029846};  // calibration of accelerometers as installed in van
//const float          GyroOffset[3]        = {+14.9890,  +0.0, -19.4354 };  // calibrated gyro offsets
  float                GyroOffset[3];
  unsigned long        GyroReadTime;                                         // Measure time at which sensors are read - for integration
  unsigned long        LastGyroReadTime;                                     // Store the last time sensors were read - for integration
  unsigned int         dt                   = 0;                             // Time Step = ReadTime - LastReadTime
  unsigned int         CalibCounter         = 0;
  int                  AcclRaw[3];                                           // Raw Accelerometer readings
  int                  GyroRaw[3];                                           // Raw gyro readings
  float                AcclAnglePitch;                                       // Calculated angle based on accelerometer reading
  float                AcclAngleRoll;                  
  float                GyroRatePitch;                                        // Calculated angle based on gyroscope reading
  float                GyroRateRoll;
  float                GyroAnglePitch;                                       // Only reqd for debugging
  float                GyroAngleRoll;                                        // Only reqd for debugging
  float                AnglePitch           = -99.0;                         // Final calculated angle output from complimentary filter; initialised to -99 to detect first loop
  float                AngleRoll;
  float                GradientPitch;
  float                GradientRoll;

  // -- GPS --
  #define              PIN_GPS_RX                  12                        // define GPS serial read pin
  #define              PIN_GPS_TX                  13                        // define GPS serial write pin
  #define              GPS_BAUD_RATE               9600                      // define GPS Baud rate
  #define              GPS_QUIET_TIME_THRESH       50                        // threshold for determining that GPS is not transmitting
  unsigned long        GpsOnTime                   = 0;                      // Time that GPS was switched on
  unsigned long        LastDataTime;                                         // time at which last serial data from GPS was processed
  boolean              IsGpsOn                     = true;                   // flag to identify whether GPS is switched on
  boolean              isQuietTime                 = true;                   // flag to ensure other processes are performed only once during GPS idle
  int                  x[3]                        = {-1190,-163, 253};      // gps coordinates of box defining uk boundary  // x is longitude = E-W coordinate
  int                  y[3]                        = {5015 ,5143,6126};      //  (*100 to store as int)                      // y is latitude = N-S coordinate
  SoftwareSerial       ss(PIN_GPS_RX, PIN_GPS_TX);                           // setup software serial
  TinyGPSPlus          gps;                                                  // create a gps instance 

  // -- TIME VARIABLES --
  boolean              IsBritishSummerTime         = false;                  // flag to identify whether it is british summer time
  
  // -- BUTTON VARIABLES --
  #define              PIN_BUTTON                  11                         // define digital pin for button   
  unsigned int         ButtonState                 = 0;                       // Current state of button
  unsigned int         LastButtonState             = 0;                       // State of button last time it was checked
  unsigned long        ButtonPressTime             = 0;                       // Length of time since button was last confirmed on
  unsigned long        LastActivityTime            = 0;                       // Time since button press or warning
  boolean              IsLongPress                 = false;                   // flag to identify if last botton press was long, short term flag to avoid going into short press loop
  boolean              LongPressFlag               = false;                   // flag to ensure after long press, the next short press takes us back to summary screen
 
  // -- DISPLAY VARIABLES --
  //   DisplayScreensShort = 0   --> Summary Screen --> 2 long press screens
  //   DisplayScreensShort = 1   --> GPS Screen     --> 4 long press screens
  //   DisplayScreensShort = 2   --> Temp Screen    --> 3 long press screens
  //   DisplayScreensShort = 3   --> Time Screen    --> 1 long press screens
  //   DisplayScreensShort = 4   --> Angles Screen  --> 3 long press screens  
  unsigned int         DisplayIndexShort           = 0;                       // Defines the current display, progressed by button press
  unsigned int         DisplayIndexLong            = 0;                       // Defines the current display, progressed by button press
  char                 chrBuffer[16];                                         // text buffer used for formatting variables for display output
  const int            DisplayScreensShort         = 4;                       // maximum index for DisplayIndexShort (= number of short press screens - 1) 
  const int            DisplayScreensLong[5]       = {1, 2, 2, 0, 2};         // maximum index for DisplayIndexLong (= number of long press screens - 1) for each short press screen
  boolean              Update_Screen;                                         // flag to force screen update
 
//===============================================================================
//-------------------------------------------------------------------------------
// 
//                             SETUP 
//
//-------------------------------------------------------------------------------
//===============================================================================

void setup(){
  
  // Set pin modes - self explanatory
  pinMode(PIN_VOLTS_MAIN,     INPUT);   
  pinMode(PIN_VOLTS_AUX,      INPUT);  
  pinMode(PIN_DIGITAL_TEMP,   INPUT);
  pinMode(PIN_BUTTON,         INPUT);
  pinMode(PIN_LCD_CONTRAST,   OUTPUT);
  pinMode(PIN_LCD_BACKLIGHT,  OUTPUT);
  
  // Initialise hardware serial comms for debugging
//  Serial.begin(9600);   
 
  // Initialise digital temp gauges
  TmpSensors.begin();                                 // start sensors library
  TmpSensors.setResolution(addrInsideTemp1, 12);     
  TmpSensors.setResolution(addrInsideTemp2, 12);        
  TmpSensors.setResolution(addrOutsideTemp1,12);        
  TmpSensors.setResolution(addrOutsideTemp2,12);        
  
  // Initialise GPS and time
  ss.begin(GPS_BAUD_RATE);                            // set GPS comms speed
  initGPS();                                          // send GPS setup commands
  setSyncProvider(gpsTimeSync);                       // library function used to sync gps time with arduino---gpsTimeSync is the name of the function -- default is 5mins
  setSyncInterval(1);

  // Initialise display
  lcd.begin(16,2);                                    // initialise lcd display
  analogWrite(PIN_LCD_CONTRAST, LCD_CONTRAST);        // set lcd contrast
  SetBackLightHigh();                                 // turn on backlight 
  LastActivityTime = millis();                        // initialised idle timer
  
  // Accelerometers and gyros
  Wire.begin();                                       // start wire i2c library
  initAcc();                                          // initialise accelerometers
  initGyro();                                         // initialise gyroscopes
  
  // Display version number on screen
  lcd.setCursor(0,0); lcd << F(" Gorduino v") << SOFT_VERSION;
  lcd.setCursor(0,1); lcd << F(" Calibrating...");
  calibrateGyro();
  lcd.setCursor(0,1); lcd << GyroOffset[0] << F("/") << GyroOffset[2] << F("/") << CalibCounter << F("  ");
  delay(300);
  
}

//===============================================================================
//-------------------------------------------------------------------------------
//
//                             MAIN LOOP
//
//-------------------------------------------------------------------------------
//===============================================================================

void loop(){

  //---------------------------------------------------------------------
  // Set screen update flag - only update screen if there is new data
  //---------------------------------------------------------------------

  Update_Screen = false;
  
  //---------------------------------------------------------------------
  // Detect button press - long and short press enabled
  //---------------------------------------------------------------------
  
  ButtonState = digitalRead(PIN_BUTTON);                                                  // read button state

  if (ButtonState == HIGH && ButtonState != LastButtonState) {                            // if button is pressed and it wasnt pressed on last loop
    ButtonPressTime = millis();                                                           // button has just been pressed: record the time   
  } else if (ButtonState == HIGH && (millis() - ButtonPressTime > TIME_FOR_LONG_PRESS)) { // detect long press - dont wait for button to be unpressed
    //lcd.clear();                                                                        // request to change display, so clear current one
    DisplayIndexLong ++;                                                                  // increment long screen display
    Update_Screen = true;
    if (DisplayIndexLong > DisplayScreensLong[DisplayIndexShort]) DisplayIndexLong = 0;   // roll over to zero if at the end of the cycle
    LastActivityTime = millis();                                                          // button press so reset LastActivityTime  
    ButtonPressTime  = millis();                                                          // reset buttonpresstime
    IsLongPress = true;                                                                   // identify a long press has happened to avoid going into short press routine when button is unpressed
    LongPressFlag = true;                                                                 // flag will stay true so we know to go straight back to summary screen after short press
  } else if (ButtonState == LOW && ButtonState != LastButtonState) {                      // when button is unpressed 
      Update_Screen = true;
      if (!IsLongPress) {                                                                   // if it wasnt a long press ...
      if ((millis() - ButtonPressTime > DELAY_DEBOUNCE) && (millis() - ButtonPressTime < TIME_FOR_LONG_PRESS)) {  // ...make sure it was a short press
        if (DisplayIndexLong != 0 ) {
          DisplayIndexLong = 0 ;                                                          // if on long menu then go back to short press menu before allow to progress short press menu
        } else {                                                                          // increment display index - this will change the display
          if (BackLightIndex == 2) {                                                      // if backlight is already high, do something, otherwise dont (backlight will go high because LastActivityTime was reset)
            if (LongPressFlag) {
              DisplayIndexShort = 0;                                                      // if long button has been pressed so straight back to summary screen
              LongPressFlag = false;                                                      // reset LongPressFlag
            }  else {
              DisplayIndexShort++;                                                        // but increment shortpress menu
              if (DisplayIndexShort > DisplayScreensShort) {
                DisplayIndexShort = 0;                                                    // if incremented too high then reset to 0 
              }
            }              
          }     
        }    
        LastActivityTime = millis();                                                      // button press so reset LastActivityTime
      }
    } else {                                                                              // if it was a long press reset long press flag to false and carry on
      IsLongPress = false;
    }
  }  

  LastButtonState = ButtonState;                                                          // used to track debounce
  
  //---------------------------------------------------------------------
  // Handle lcd backlight brightness
  //---------------------------------------------------------------------
  
  if (millis()-LastActivityTime < TIME_BACKLIGHT_HIGH) {                       // If time since last activity is less than the required delay, then keep backlight high
    if (BackLightIndex != 2) {                                                 // only set backlight if it is not already high
      SetBackLightHigh();  
    }      
  } else if (BackLightIndex == 2) {
    SetBackLightLow();                                                         // else if backlight is high then turn it down
  }

  //---------------------------------------------------------------------
  // Take sensor readings
  // - Detect contents of serial buffer and read if necessary
  // - Detect time since last read, and if over threshold then read other sensors
  //---------------------------------------------------------------------

  while (ss.available()) {                                                      // if there is data in the softwareserial buffer
    gps.encode(ss.read());                                                      // read it and send to tinygps++ for decoding
    LastDataTime = millis();                                                    // record the time at which data was last processed
  }
  
  if (millis() - LastDataTime > GPS_QUIET_TIME_THRESH && isQuietTime) {         // if no serial data has been received for a short period
    isQuietTime = false;                                                        //  set flag so we only do this loop once between GPS data bursts
    ReadVolts();                                                                // read engine voltages
    ReadTemps();                                                                // read temperatures 
    if (gps.time.age() < 100) {                                                 // if gps time is recent then sync
      now();
    }
    sensor[SPEED_MPH  ].value = PadInt(gps.speed.mph()      ,2," ");
    sensor[ALTITUDE   ].value = PadInt(gps.altitude.meters(),4," ");
    sensor[COURSE_DEGS].value = PadInt(gps.course.deg()     ,3,"0");
    DisplayValues();                                                            // update display
  }
  
  else {
   isQuietTime = true;                                                          // reset idleflag
  }
  
  ReadAccls();                                                                  // always read attitude to minimise integration time, dt
   
  //---------------------------------------------------------------------
  // Determine status of engine
  //---------------------------------------------------------------------

  if (IsEngineRunning && VoltsMain < CHARGING_VOLTS*10) {                      // engine is not running, but flag indicates it is then engine has just been turned off; set GPS timer 
    GpsOnTime = millis();   
  }                                                       
  if (VoltsMain > CHARGING_VOLTS*10) {
    IsEngineRunning = true;                                                    // otherwise if volts are high set flag to true
  } else {
    IsEngineRunning = false;                                                   // else set flag to false
  }
  
  //---------------------------------------------------------------------
  // Determine if Accl should be turned on or off
  //---------------------------------------------------------------------

/*   
  if (!IsEngineRunning && DisplayIndexShort != 4) {    // if engine is off and not on screen 4
    if (adxl.isPowerOn()) adxl.powerOff();             // turn off acccelerometers
  }
  else {                                               // otherwise...
   if (!adxl.isPowerOn()) adxl.powerOn();              // turn them on!
  }    
*/  
  //---------------------------------------------------------------------
  // Determine if GPS should be turned on or off - deactivated for debugging 
  //---------------------------------------------------------------------
/*  
  if (IsEngineRunning || (DisplayIndexShort == 1)) {         // if the engine is on OR engine is off but gps data is requested, make sure gps is on
    if (IsGpsOn == false) TurnOnGPS();
  }
  else {                                                // if engine is not on 
    if ((millis() - GpsOnTime) > DELAY_GPS_STANDBY) {   // and time is greater than defined standby time
      if (IsGpsOn) TurnOffGPS();                        // make sure gps is turned off
    }
  }
*/  

}                                                                           // close main loop subroutine

//================================================================================================================
// PROCESS TIMING
//----------------------------------------------------------------------------------------------------------------
// SoftwareSerial Buffer interrupt takes about 30ms and can kick in at any point in the loop
//
// 0 - 150ms (max) Read Serial Data from GPS (max value when reading two sentences - can be as little as 40ms)
// 150-200ms wait for 'quiet time'
// 200-205ms read volts (actually almost 0ms)
// 205-1005ms read temps (only done once  a minute)
// 1005ms-1020ms update display 
// 1020-1025ms read accls
//
// unless checking temps, pleaty of time for other processes between gps reads
// should only drop data if temps read and two sentence gps are recieved at the same time.
//================================================================================================================

