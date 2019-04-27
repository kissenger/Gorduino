//===============================================================================
//-------------------------------------------------------------------------------
//
//                   LCD DISPLAY SUBROUTINE
//
//-------------------------------------------------------------------------------
//===============================================================================

void DisplayValues() {
  
  // Loop time max 15ms (but doesnt execute each time called due to DELAY_SCREEN_UPDATE)

  if (IsDisplayChanged) {

   lcd.clear();
   
    // set all sensor values to not displayed
    int i=0;
    while (i < SENSOR_ARRAY_SIZE) {
      sensor[i].isdisplayed=false;
      i++;
    }
    
    if (DisplayIndexShort == 0) {
      if (IsEngineRunning) {
        if (gps.speed.mph() < ANGLES_DISPL_THRESH) {
          sensor[PITCH_ANGLE ].isdisplayed=true;
          sensor[ROLL_ANGLE  ].isdisplayed=true;      
          sensor[TIME        ].isdisplayed=true;
          sensor[ALTITUDE    ].isdisplayed=true;
          sensor[TEMP_IN_AVE ].isdisplayed=true;
          sensor[TEMP_OUT_AVE].isdisplayed=true;
          lcd.setCursor(0,0); lcd << "   \337          \337C";
          lcd.setCursor(0,1); lcd << "   \337      m   \337C";
        }
        else {
          sensor[SPEED_MPH   ].isdisplayed=true;
          sensor[COURSE_DEGS ].isdisplayed=true;      
          sensor[TIME        ].isdisplayed=true;
          sensor[ALTITUDE    ].isdisplayed=true;
          sensor[TEMP_IN_AVE ].isdisplayed=true;
          sensor[TEMP_OUT_AVE].isdisplayed=true;
          lcd.setCursor(0,0); lcd << "   mph        \337C";
          lcd.setCursor(0,1); lcd << "   \337      m   \337C";
        }
      }
      else {
        sensor[VOLTS_MAIN  ].isdisplayed=true;
        sensor[VOLTS_AUX   ].isdisplayed=true;      
        sensor[TIME        ].isdisplayed=true;
        sensor[TIME_ZONE   ].isdisplayed=true;
        sensor[TEMP_IN_AVE ].isdisplayed=true;
        sensor[TEMP_OUT_AVE].isdisplayed=true;
        lcd.setCursor(0,0); lcd << "    v         \337C";
        lcd.setCursor(0,1); lcd << "    v         \337C";
      }
    }
  }

//loop through each sensor reading and print to screen if updated and on current display
  
  if (DisplayValues) {
    int i=0;
    
    while (i < SENSOR_ARRAY_SIZE) {
      if (sensor[i].isdisplayed) {
        lcd.setCursor(sensor[i].col, sensor[i].row);
        lcd << sensor[i].value;
      }

      i++;
    }
  }
}  
//===============================================================================
//-------------------------------------------------------------------------------
//
//                    LCD RELATED SUBROUTINES
//
//-------------------------------------------------------------------------------
//===============================================================================

void SetBackLightHigh(){
  analogWrite(PIN_LCD_BACKLIGHT, LCD_LIGHT_HIGH); 
  BackLightIndex = 2;
}

void SetBackLightLow(){
  analogWrite(PIN_LCD_BACKLIGHT, LCD_LIGHT_LOW);
  BackLightIndex = 1;
}

