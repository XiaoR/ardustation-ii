void EEPROM_writeRevision()
{
  EEPROM.write(12,73);
  EEPROM.write(13,37);
  EEPROM.write(14,73);
/*  byte lowByte = ((paramCount >> 0) & 0xFF);
  byte highByte = ((paramCount >> 8) & 0xFF);
  EEPROM.write(3, lowByte);
  EEPROM.write(4, highByte);*/
}

int EEPROM_readRevision()
{
  if (EEPROM.read(12) == 73 && EEPROM.read(13) == 37 && EEPROM.read(14) == 73)
  {
    byte lowByte = EEPROM.read(3);
    byte highByte = EEPROM.read(4);
    return (0);
  }
  else
    return -1;
}

int EEPROM_writeParameter(int ee, parameter& value)
{
    ee += 15; //account for revision info
    const byte* p = (const byte*)(const void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
    {
	  EEPROM.write(ee++, *p++);
    }
    return i;
}

int EEPROM_writeFloat(int ee, float& value)
{
    ee += 15; //account for revision info
    const byte* p = (const byte*)(const void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
    {
	  EEPROM.write(ee++, *p++);
    }
    return i;
}

int EEPROM_readParameter(int ee, parameter& value)
{
    ee += 15; //account for revision info
    byte* p = (byte*)(void*)&value;
    int i;
    for (i = 0; i < sizeof(value); i++)
	  *p++ = EEPROM.read(ee++);
    return i;
}

void init_EEPROM()
{
  lcd.clear();
  lcd_print_P(PSTR("Init EEPROM..."));
  char buffer[15];
  for (int i=0; i<TOTAL_PARAMS; i++)
  {
    parameter temp;
    temp.value = 0;
    get_Param_Key(temp.key, i);
    EEPROM_writeParameter(i * sizeof(temp), temp);
  }
  //EEPROM_writeParamCount(TOTAL_PARAMS);
  delay(1000);
  menu = MAIN_MENU;
  redraw = 1;
}


