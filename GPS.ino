
//===============================================================================
//                           GPS COMMS
//===============================================================================

/*void TurnOffGPS() {
  ss.println("$PMTK161,0*28");  // turn off gps
  IsGpsOn = false;              // Set flag for reference
}*/

/*void TurnOnGPS() {
  ss.println(" ");              // turn on gps - just need to send a byte of informationt to wake it up
//  IsGpsOn = true;               // Set flag for reference
//  GpsOnTime = millis();         // record time 
}*/

void initGPS() {
  
//ss.println("$PMTK161,0*28");                       // wake gps up after standby
//ss.println("$PGCMD,33,1*6C");                      // request antenna connection status
  ss.println("$PMTK220,1000*1F");                    // set GPS update rate to 1Hz
  
  //---------------------------------------------------------------------
  // Define NMEA Sentence output
  //---------------------------------------------------------------------
  // Supported NMEA Sentences
  // 0 NMEA_SEN_GLL, // GPGLL interval - Geographic Position - Latitude longitude
  // 1 NMEA_SEN_RMC, // GPRMC interval - Recommended Minimum Specific GNSS Sentence
  // 2 NMEA_SEN_VTG, // GPVTG interval - Course over Ground and Ground Speed
  // 3 NMEA_SEN_GGA, // GPGGA interval - GPS Fix Data
  // 4 NMEA_SEN_GSA, // GPGSA interval - GNSS DOPS and Active Satellites
  // 5 NMEA_SEN_GSV, // GPGSV interval - GNSS Satellites in View
  // 6 //Reserved
  // 7 //Reserved
  // 13 //Reserved
  // 14 //Reserved
  // 15 //Reserved
  // 16 //Reserved
  // 17 //Reserved
  // 18 NMEA_SEN_MCHN, // PMTKCHN interval – GPS channel status 
  //---------------------------------------------------------------------
  // Use: http://www.hhhh.org/wiml/proj/nmeaxor.html to generate checksums
  //---------------------------------------------------------------------
  
//ss.println("$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29");    // only send RMC data once per update
//ss.println("$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28");    // only send RMC and GGA data once per update
  ss.println("$PMTK314,0,1,0,5,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*2C");    // only send RMC once per update and GGA data once per 5 updates  
   
}

//===============================================================================
//                           OTHER GPS 
//===============================================================================

boolean IsLocationInUK() {
  
  // function to determine timezone
  // works by using uk bounding box defined in parameter list above
  // if current lat/long is within box then GMT, else GMT+1
  
  int x_0 = gps.location.lng()*100.0;  // as per declared variables, x is longitude = E-W coordinate
  int y_0 = gps.location.lat()*100.0;  // and y is latitude = N-S coordinate

  if ( (x_0>x[0] && x_0<x[1] && y_0>y[0] && y_0<y[2]) || (x_0>x[1] && x_0<x[2] && y_0<y[2] && y_0>(y[1]-y[0])/(x[2]-x[1])*(x_0-x[1])+y[0]) ) {
    return true;
  }
  return false; 
}

char * strDegMinSec (double pos) {

  // function to convert decimal dgrees to degrees minutes seconds format as a string
  
  static char sdms[13] = "";
  char buff[4];
  
  unsigned int  intDegs;
  unsigned int  intMins;
  unsigned int  intSecs;
  unsigned int  intThou;
  
  // convert decimal format to degs  mins secs from decimal degrees
  if (pos < 0) pos = -pos;
  intDegs          = pos;
  intMins          = (   pos - intDegs) *60;
  intSecs          = ((( pos - intDegs) *60) - intMins) *60;
  intThou          = ((((pos - intDegs) *60) - intMins) *60 - intSecs) *1000;

  // construct string
  strcpy(sdms,PadInt(intDegs,2,"0"));
  strcat(sdms,"\337");                   //degrees
  strcat(sdms,PadInt(intMins,2,"0"));
  strcat(sdms,"\"");                     //minutes
  strcat(sdms,PadInt(intSecs,2,"0"));
  strcat(sdms,".");                      //decimal
  strcat(sdms,PadInt(intThou,3,"0"));
  strcat(sdms,"\'");                     //seconds
  
  return sdms;
}
