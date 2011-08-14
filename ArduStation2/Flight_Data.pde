void flight_data() // menu 1
{
  if (millis() - timer > 200)
  {
    timer = millis();
    lcd.clear();
    switch (subMenu)
    {
      case 0:
      {
        lcd.setCursor(0,0);
        lcd_print_P(PSTR("AT:"));
        lcd.print(altitude * DIST_CONV );

        lcd.setCursor(10,0); 
        lcd_print_P(PSTR("NSat:"));
        lcd.print(numSats);

        lcd.setCursor(19,0); 
         if (gpsfix == 2) {
        lcd_print_P(PSTR("2"));  }         
//           { lcd.write(1);}}
           else if (gpsfix == 3){
        lcd_print_P(PSTR("3"));}   
//               {lcd.write(2);}}
           else {
               {lcd_print_P(PSTR(" "));}}  

        lcd.setCursor(0,1); 
        lcd_print_P(PSTR("RL:"));
        lcd.print(roll);

        lcd.setCursor(10,1); 
        lcd_print_P(PSTR("LT:"));
        lcd.print(latitude,3);   

        lcd.setCursor(0,2); 
        lcd_print_P(PSTR("YW:"));
        lcd.print(yaw);
        
        lcd.setCursor(10,2); 
        lcd_print_P(PSTR("LN:"));
        lcd.print(longitude,3);

        lcd.setCursor(0,3); 
        lcd_print_P(PSTR("BR:"));
        lcd.print(Bearing_Home*2.0); 
        
        lcd.setCursor(10,3); 
        lcd_print_P(PSTR("BT:"));
        lcd.print(battery/1000);
        lcd_print_P(PSTR("V"));

        break;
      }
      case 1:
      {
      Distance_Home=calc_dist(Latitude_Home, Longitud_Home, latitude, longitude);
      Distance3D_Home= sqrt((Distance_Home*Distance_Home)+((altitude-Altitude_Home)*(altitude-Altitude_Home)));

      lcd.clear();
      
      lcd_print_P(PSTR("D2D~"));
      lcd.print(Distance_Home * DIST_CONV );
      lcd.setCursor(0, 1);
      
      lcd_print_P(PSTR("Angle~"));
      lcd.print(Angle_Home);
      lcd.setCursor(0, 2);
      
      lcd_print_P(PSTR("Alt~"));
      lcd.print(altitude * DIST_CONV );
      lcd.setCursor(0, 3);
      
      lcd_print_P(PSTR("Dir~"));
      lcd.print(SvBearingHome);
      lcd.setCursor(10, 0);
      
      lcd_print_P(PSTR("D3D~"));
      lcd.print(Distance3D_Home * DIST_CONV );
      lcd.setCursor(10, 1);
      break;
      }
      
//      case 2:
//      {
//      lcd.clear();
//      lcd_print_P(PSTR("Pan~"));
//      lcd.print(Bearing_Home);
//      lcd.setCursor(0, 1);
//      lcd_print_P(PSTR("Vert~"));
//      lcd.print(Angle_Home);
//      lcd.setCursor(0, 2);
//      lcd_print_P(PSTR("ALT~"));
//      lcd.print(altitude * DIST_CONV );
//      lcd.setCursor(0, 3);
//      lcd_print_P(PSTR("Dir~"));
//      lcd.print(Bearing_Home);
//      lcd.setCursor(10, 0);      
//      lcd_print_P(PSTR("Tilt~"));
//      lcd.print(Constrain_Angle_Home);
//      lcd.setCursor(10, 1);
//      lcd_print_P(PSTR("Ant~"));
//      lcd.print(SvBearingHome);
//      lcd.setCursor(10, 2);
//      break;
//      }
      case 2:
            {      
      lcd.clear(); //Printing all the stuff 
      lcd_print_P(PSTR("Set Home?"));
      lcd.setCursor(0, 1); 
      lcd.print(latitude,6);
      lcd.setCursor(10, 1);
      lcd.print(longitude,6);
      lcd.setCursor(0, 2);
      lcd_print_P(PSTR("Press OK to Save"));
       lcd.setCursor(0, 3);
      lcd.print(Latitude_Home,6);
      lcd.setCursor(10, 3);
      lcd.print(Longitud_Home,6);
      break;
      }
      
    }
  }
}
