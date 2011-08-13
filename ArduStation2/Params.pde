int get_Param_Key(char *buffer, int index)
{
  switch (index)
  {
    case 0: { strcpy_P(buffer, PSTR("RATE_RLL_P")); break; }
    case 1: { strcpy_P(buffer, PSTR("RATE_RLL_I")); break; }
    //case 2: { strcpy_P(buffer, PSTR("RATE_RLL_D")); break; }
    case 2: { strcpy_P(buffer, PSTR("RATE_RLL_IMAX")); break; }
    case 3: { strcpy_P(buffer, PSTR("RATE_PIT_P")); break; }
    case 4: { strcpy_P(buffer, PSTR("RATE_PIT_I")); break; }
    //case 6: { strcpy_P(buffer, PSTR("RATE_PIT_D")); break; } 
    case 5: { strcpy_P(buffer, PSTR("RATE_PIT_IMAX")); break; }
    case 6: { strcpy_P(buffer, PSTR("RATE_YAW_P")); break; }
    case 7: { strcpy_P(buffer, PSTR("RATE_YAW_I")); break; }
    //case 10: { strcpy_P(buffer, PSTR("RATE_YAW_D")); break; }
    case 8: { strcpy_P(buffer, PSTR("RATE_YAW_IMAX")); break; }
    case 9: { strcpy_P(buffer, PSTR("STB_RLL_P")); break; }
    case 10: { strcpy_P(buffer, PSTR("STB_RLL_I")); break; }
    //case 14: { strcpy_P(buffer, PSTR("STB_RLL_D")); break; }
    case 11: { strcpy_P(buffer, PSTR("STB_RLL_IMAX")); break; }
    case 12: { strcpy_P(buffer, PSTR("STB_PIT_P")); break; }
    case 13: { strcpy_P(buffer, PSTR("STB_PIT_I")); break; }
    //case 18: { strcpy_P(buffer, PSTR("STB_PIT_D")); break; }
    case 14: { strcpy_P(buffer, PSTR("STB_PIT_IMAX")); break; }
    case 15: { strcpy_P(buffer, PSTR("STB_YAW_P")); break; }
    case 16: { strcpy_P(buffer, PSTR("STB_YAW_I")); break; }
    //case 22: { strcpy_P(buffer, PSTR("STB_YAW_D")); break; }
    case 17: { strcpy_P(buffer, PSTR("STB_YAW_IMAX")); break; }
    case 18: { strcpy_P(buffer, PSTR("NAV_LAT_P")); break; }
    case 19: { strcpy_P(buffer, PSTR("NAV_LAT_I")); break; }
    case 20: { strcpy_P(buffer, PSTR("NAV_LAT_D")); break; }
    case 21: { strcpy_P(buffer, PSTR("NAV_LAT_IMAX")); break; }
    case 22: { strcpy_P(buffer, PSTR("NAV_LON_P")); break; }
    case 23: { strcpy_P(buffer, PSTR("NAV_LON_I")); break; }
    case 24: { strcpy_P(buffer, PSTR("NAV_LON_D")); break; }
    case 25: { strcpy_P(buffer, PSTR("NAV_LON_IMAX")); break; }
    case 26: { strcpy_P(buffer, PSTR("NAV_WP_P")); break; }
    case 27: { strcpy_P(buffer, PSTR("NAV_WP_I")); break; }
    case 28: { strcpy_P(buffer, PSTR("NAV_WP_D")); break; }
    case 29: { strcpy_P(buffer, PSTR("NAV_WP_IMAX")); break; }
    case 30: { strcpy_P(buffer, PSTR("MAG_ENABLE")); break; }
    default: { return -1; }
  }
  return 0;
}


int find_param(const char* key)
{
  char buffer[15];
  for (int i=0; i<TOTAL_PARAMS; i++)
  {
    get_Param_Key(buffer, i);
    if (strcmp(buffer,(const char*)key) == 0)
      return i;    
  }
  return -1;  
}

void get_params()
{
  if (paramsRecv >= TOTAL_PARAMS)
  {
    menu = MAIN_MENU;
    redraw = 1;
    return;
  }
  if (timeOut == GET_PARAMS_TIMEOUT)// || timeOut == 80 || timeOut == 60 || timeOut == 40 || timeOut == 20)  // request parameter every 2 seconds
  {
//      char buffer[15];
//      get_Param_Key(buffer, paramsRecv);
      mavlink_message_t msgp;
//      mavlink_msg_param_request_read_pack(127, 0, &msgp, 7, 1, (int8_t *)buffer, 0);
      mavlink_msg_param_request_list_pack(127, 0, &msgp, 7, 1);
      send_message(&msgp);
      timeOut--; //just so we don't send twice during the same cycle...
  }
  if (millis() - timer > 100)
  {
    timer = millis();
    timeOut--;
    redraw = 1;
  }
  if (redraw == 1)
  {
    lcd.clear();
    lcd_print_P(PSTR("Requesting params..."));
    lcd.setCursor(0,1);
    lcd_print_P(PSTR("Params received:"));
    lcd.setCursor(6,2);
    lcd.print(paramsRecv);
    lcd_print_P(PSTR(" of "));
    lcd.print(TOTAL_PARAMS);
    redraw = 0;
  }
  if (timeOut < 1)
  {
    lcd.clear();
    lcd.setCursor(0,3);
    lcd_print_P(PSTR("Timed out!"));
    delay(1000);
    menu = MAIN_MENU;
    redraw = 1;
  }
}

void list_params()
{
  lcd.clear();
  parameter temp;
  if (currentOption < 0)
    currentOption = TOTAL_PARAMS - 1;
  if (currentOption > TOTAL_PARAMS -1)
    currentOption = 0;
  for (int i=0; i<4; i++)
  {
    if (i + (currentOption/4 * 4) < TOTAL_PARAMS)
    {
      lcd.setCursor(0,i);
      EEPROM_readParameter((i + (currentOption/4 * 4)) * sizeof(temp), temp);
      if (strlen(temp.key)<10)
        lcd.print(temp.key);
      else
      {
        for (int i=0; i<9; i++)
          lcd.print(temp.key[i]);
      }
      lcd.setCursor(10,i);
      if (i == currentOption%4)
      {
        lcd.print(">");
        strcpy(editParm.key, temp.key);
        editParm.value = temp.value;
      }
      else
        lcd.print(" ");
      lcd.print(temp.value);
      if (i == currentOption%4)
        lcd.print("<");
    }
  }
  redraw = 0;
}

void edit_params()
{
  lcd.clear();
  if (strlen(editParm.key)<10)
    lcd.print(editParm.key);
  else
  {
    for (int i=0; i<9; i++)
      lcd.print(editParm.key[i]);
  }
  lcd.setCursor(10,0);
  lcd.print(">");
  lcd.print(editParm.value);
  lcd.print("<");
  lcd.setCursor(0,1);
  lcd_print_P(PSTR("Up/Down -> +/- .01"));
  lcd.setCursor(0,2);
  lcd_print_P(PSTR("Right/Left -> +/- 1"));
  lcd.setCursor(0,3);
  lcd_print_P(PSTR("Back->Quit OK->Send"));
  redraw = 0;
}

void save_param()
{
  if ((timeOut == 100 || timeOut == 80 || timeOut == 60 || timeOut == 40 || timeOut == 20) && waitingAck == 1)  // request parameter every 2 seconds
  {
      mavlink_message_t msgp;
      mavlink_msg_param_set_pack(127, 0, &msgp, 7, 1, (int8_t *)editParm.key, editParm.value);
      send_message(&msgp);
      timeOut--; //just so we don't send twice during the same cycle...
  }
  if (millis() - timer > 100)
  {
    timer = millis();
    timeOut--;
  }
  if (redraw == 1)
  {
    lcd.clear(); 
    lcd_print_P(PSTR("Sending new value..."));
    redraw = 0;
  }
  if (waitingAck == 0)
  {
    lcd.setCursor(0,2);
    lcd_print_P(PSTR("Value updated!"));
    delay(1000);
    menu = EDIT_VIEW_PARAMS;
    redraw = 1;
  }
  if (timeOut < 1)
  {
    lcd.setCursor(0,2);
    lcd_print_P(PSTR("Timed out!"));
    delay(1000);
    menu = EDIT_VIEW_PARAMS;
    redraw = 1;
    waitingAck = 0;
  }
}
