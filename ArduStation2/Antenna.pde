void SaveHomePosition() // Used to save lat/lon/altitude for tilt/pan computations
{
    Latitude_Home=latitude;
    Longitud_Home=longitude;
    Altitude_Home = altitude;
    eeprom_busy_wait();
    ee.write_float(0x00,Latitude_Home);
    eeprom_busy_wait();
    ee.write_float(0x04,Longitud_Home);
    eeprom_busy_wait();
    eeprom_write_dword((unsigned long*)0x08,(long)(Altitude_Home));
    
}

void RestoreHomePosition()
{
  eeprom_busy_wait();
  Latitude_Home=ee.read_float(0x00);
  eeprom_busy_wait();
  Longitud_Home=ee.read_float(0x04);
  eeprom_busy_wait();
  Altitude_Home=(long)eeprom_read_dword((unsigned long*)0x08);
  eeprom_busy_wait();
}

void SetAntennaPosition()
{
  if (pan_pos>180) 
         pan_pos=180;
     if (pan_pos < 0)
        pan_pos=0;
    Pan.write(pan_pos);
    Tilt.write(tilt_pos);
    lcd.clear();
    lcd_print_P(PSTR("Antenna Test"));
    lcd.setCursor(0,1);
    lcd_print_P(PSTR("Tilt~"));
    lcd.print(tilt_pos);
    lcd.setCursor(10,1);
    lcd_print_P(PSTR("Pan~"));
    lcd.print(pan_pos);
    
    redraw=0;   
}

void SetServoHardStop() // menu 2
{
 
     chg_angle=0;
     if (pan_pos>180) 
         pan_pos=180;
     if (pan_pos < 0)
        pan_pos=0;
     Pan.write(pan_pos);
     lcd.clear();
     lcd_print_P(PSTR("Servo Offset"));
     lcd.setCursor(0,1);
     lcd_print_P(PSTR("Tilt~"));
     lcd.print(tilt_pos);
     lcd.setCursor(10,1);
     lcd_print_P(PSTR("Pan~"));
     lcd.print(pan_pos);
     lcd.setCursor(0,2);
     lcd_print_P(PSTR("Compass="));
     lcd.print((180.0-pan_pos)*2.0);
     // Compute south offset for rotating servo limit point
     offset = pan_pos - 90.0;  
     redraw=0;   

//  char currentMode[3][4] = {"ACR","STB","NAV"};
//  char currentAxis[3][4] = {"RLL","PIT","YAW"};
//  char currentAxisNav[3][4] = {"LAT","LON","WP"};
//  char pids[3][2] = {"P","I","D"};
//  char currentPID[15];
//  lcd.clear();
//  lcd.setCursor(1,0);
//  lcd.print(currentMode[subMenu][0]);
//  if (subMenu == 1 || subMenu == 2)
//  {
//    lcd.setCursor(0,0);
//    lcd.write(3);
//  }
//  if (subMenu == 0 || subMenu == 1)
//  {
//    lcd.setCursor(19,0);
//    lcd.write(4);
//  }
//  for (int i=0; i<3; i++)
//  {
//    lcd.setCursor(0,i+1);
//    lcd.print(pids[i]);
//    lcd.setCursor(3 + (i*6),0);
//    if (subMenu == 2)
//      lcd.print(currentAxisNav[i]);
//    else
//      lcd.print(currentAxis[i]);
//    for (int j=0; j<3; j++)
//    {
//      strcpy(currentPID,currentMode[subMenu]);
//      strcat(currentPID,"_");
//      if (subMenu == 2)
//        strcat(currentPID,currentAxisNav[i]);
//      else
//        strcat(currentPID,currentAxis[i]);
//      strcat(currentPID,"_");
//      strcat(currentPID,pids[j]);
//      int loc = find_param(currentPID);
//      parameter temp;
//      EEPROM_readParameter(loc * sizeof(temp), temp);
//      lcd.setCursor(2 + (i*6),j+1);
//      lcd.print(temp.value);
//    }
//  }
//  redraw = 0;
}

void position_antenna()
{
    Distance_Home=calc_dist(Latitude_Home, Longitud_Home, latitude, longitude);
    Bearing_Home=calc_bearing(Latitude_Home, Longitud_Home, latitude, longitude);
    SvBearingHome=Bearing_Home;
    Angle_Home=ToDeg(atan((float)(altitude-Altitude_Home)/(float)Distance_Home));
    Bearing_Home = 180-(Bearing_Home/2.0);
    
    // Offset for servo limit 
    Bearing_Home = Bearing_Home + offset;
    if (Bearing_Home > 180.0)
       Bearing_Home = Bearing_Home - 180.0;
    else
    {
        if (Bearing_Home <0.0)
           Bearing_Home = Bearing_Home + 180.0;
    }
    Pan.write(Bearing_Home); //180-(Bearing_Home/2.0));
    Constrain_Angle_Home = constrain(Angle_Home,0,90);
    Constrain_Angle_Home = 90-Constrain_Angle_Home;
    
    int tilt_position;
    tilt_position = Constrain_Angle_Home + 50;
    
    if (tilt_position > tilt_pos_upper_limit) tilt_position = tilt_pos_upper_limit;
    
    if (tilt_position < tilt_pos_lower_limit) tilt_position = tilt_pos_lower_limit;
   
    Tilt.write(tilt_position);
   
}
