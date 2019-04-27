//===============================================================================
//-------------------------------------------------------------------------------
//
//                    UTILITY SUBROUTINES
//
//-------------------------------------------------------------------------------
//===============================================================================

char * PadInt(int num, const int wdth, char * pad) {
  
  // -------------------------------------------------------------------------  
  // function takes an integer and left pads to desired field width with requested char
  // int num - integer number to left-pad
  // int wdth - desired field width after padding (max 10 defined by zpi/buff array size)
  // char pad - character requested to pad with
  // -------------------------------------------------------------------------
  // eg: PadInt(56,5," ") returns "   56"
  // eg: PadInt(-12,4,"0") returns "0-12"
  // eg: PadInt(-12,4," ") returns " -12"
  // eg: PadInt(135,2," ") returns "13"
  // -------------------------------------------------------------------------

  static char zpi[10]="";                             //initialise zpi (zero-pad-integer) array
  char buff[10]="";                                   //initialise buffer array

  itoa(num,buff,10);                                  //convert the subject number to a string and store in buffer arrat
  strcpy(zpi,"");                                     // put nothing into zpi array                              

  if (strlen(buff) > wdth) strncat(zpi,buff,wdth);    // if buffer is longer than wdth requested then knock off end characters
  else {
    for (int i = 0; i < (wdth-strlen(buff)); i++) {   // for the number of characters in zpi not filled by the number in buff
      strcat(zpi,pad);                                // fill with pad character
    }
    strcat(zpi,buff);                                 // then copy buff onto the end 
  }
  
  return zpi;                                         // and return

}

