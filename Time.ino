
//===============================================================================
//-------------------------------------------------------------------------------
//
//                           TIME SUBROUTINES
//
//-------------------------------------------------------------------------------
//===============================================================================

time_t gpsTimeSync(){

  // subroutine adapted for tinygps++ from time library gps example
  // checks validity of gps time data and updates if ok calls gpsTimeToArduinoTime
  // if gps data is invalid or old, it returns 0 

 // if((gps.time.age() < 1000) && (gps.time.isValid())) {
    return gpsTime();
 // }

//  return 0;
  
}

time_t gpsTime(){
  
  // subroutine adapted for tinygps++ from time library gps example
  // returns time_t from gps date and time with the given offset hours

  int TimezoneOffset = 0;
  int Weekday;
  tmElements_t tm;
  
  tm.Month  = gps.date.month();
  tm.Day    = gps.date.day();
  tm.Hour   = gps.time.hour();
  tm.Minute = gps.time.minute();
  tm.Second = gps.time.second();
  tm.Year   = gps.date.year() - 1970;
  time_t time = makeTime(tm);

//  if ((tm.Month == 3  && (tm.Day - Weekday + 1) >= 25)  ||
//      (tm.Month >  3  && tm.Month < 10 )                ||
//      (tm.Month == 10 && (tm.Day - Weekday + 1)  < 25)  ){
    IsBritishSummerTime=true;
//  } 
  
  if (IsBritishSummerTime)  TimezoneOffset++;
  if (!IsLocationInUK())    TimezoneOffset++;

  return time + (TimezoneOffset * SECS_PER_HOUR);

}

char * strTime(){

  // function to return string containing time in hh:mm format
  
  static char st[6] = "";
  
  strcpy(st,PadInt(hour(),2,"0"));
  strcat(st,":");
  strcat(st,PadInt(minute(),2,"0"));
  
  return st;
}

char * strDate() {
  
  // function to return date in dd/mm/yyyy format
    
  static char sd[11]="";
 
  strcpy(sd,PadInt(day(),2,"0"));
  strcat(sd,"/");
  strcat(sd,PadInt(month(),2,"0"));
  strcat(sd,"/");
  strcat(sd,PadInt(year(),4,"0"));
 
  return sd;
}


char *strTimeZone() {

  // function to return string representing current timezone

    if (IsBritishSummerTime) {
      if (IsLocationInUK()) return "BST  ";
      else                  return "BST+1";
    }
    else {
      if (IsLocationInUK()) return "GMT  ";
      else                  return "GMT+1";
    }

}



