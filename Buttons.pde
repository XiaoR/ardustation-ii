unsigned char Button_0()
{
  static byte lock; 
  pinMode(13,LOW);
  pinMode(11, OUTPUT);
  digitalWrite(11, HIGH);
  pinMode(12, INPUT);

  if(digitalRead(12) == HIGH)
  return 1;
  else
  return 0;
}

//Button2: PORTB6 output, check PINB7
unsigned char Button_1()
{
  static byte lock;
  digitalWrite(11, LOW);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  pinMode(13, INPUT);

  if(digitalRead(13) == HIGH)
  return 1;
  else
  return 0;
}

//Button3: PORTB7 output, check PINB5
unsigned char Button_2()
{
  static byte lock;
  pinMode(12,LOW);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  pinMode(11, INPUT);
  if(digitalRead(11) == HIGH)
  return 1;
  else
  return 0;
}

//Button4: PORTB6 output, check PINB5
unsigned char Button_3()
{
  static byte lock;
  pinMode(13,LOW);
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);
  pinMode(11, INPUT);
  if(digitalRead(11) == HIGH)
  return 1;
  else
  return 0;
}

//Button5: PORTB7 output, check PINB6
unsigned char Button_4()
{
  static byte lock;
  pinMode(11,LOW);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  pinMode(12, INPUT);
  if(digitalRead(12) == HIGH)
  return 1;
  else
  return 0;
}

//Button1: PORTB6 output, check PINB7
unsigned char Button_5()
{
  pinMode(12,LOW);
  pinMode(11, OUTPUT);
  digitalWrite(11, HIGH);
  pinMode(13, INPUT);
  
  if(digitalRead(13)==HIGH)
  return 1;
  else
  return 0;
}

/*********************************************/

/*********************************************/

/*********************************************/

byte Lock_Button(byte button, byte inLock[1], int time)
{
  if(button == 1)
  {
    if(inLock[0] == 0)
    {
      inLock[0]=1;
      buzz(time,200);
      return 1;
    }
    else
    {
      return 0;
    }
  }
  else
  {
    inLock[0]=0;
    return 0; 
  }
}

void Check_Buttons(byte max_options) //Reading the buttons. 
{
  static byte lock[6];
  byte button_byte=0;

  if(Lock_Button(Button_0(), &lock[0], 200)) // UP
  {
    if (menu == MAIN_MENU)
    {
      if (currentOption > 0)
      {
        currentOption--;
        redraw = 1;
      }
    }
    else if (menu == EDIT_VIEW_PARAMS)
    {
      currentOption--;
      redraw = 1;
    }
    else if (menu == ANT_TEST)
    {          
            tilt_pos+=5;
            if (tilt_pos > tilt_pos_upper_limit) tilt_pos = tilt_pos_upper_limit;
            redraw = 1;
    }
    else if (menu == 10)
    {
      editParm.value += .01;
      redraw = 1;      
    }
  }

  if(Lock_Button(Button_1(), &lock[1], 200)) // BACK
  {
    if (menu == EDIT_VIEW_PARAMS)
      currentOption = lastOption;
    if (menu == FLIGHT_DATA || menu == ANTS || menu == GET_PARAMS || menu == EDIT_VIEW_PARAMS)
    {
      menu = MAIN_MENU;
      subMenu = 0;
    }
    else if (menu == ANT_TEST) 
    {
      pan_pos = original_pan_pos;
      tilt_pos = original_tilt_pos;
      chg_angle = original_chg_angle;

      menu = MAIN_MENU;
      subMenu = 0;
    }
    else if (menu == 10)
      menu = EDIT_VIEW_PARAMS;
    redraw = 1;
  }

  if(Lock_Button(Button_2(), &lock[2], 200)) // LEFT
  {
    if (menu == FLIGHT_DATA)// || menu == PIDS)
    {
      if (subMenu > 0)
      {
        subMenu--;
        redraw = 1;
      }
      else{
         subMenu=3;
         redraw=1;
      }
    }
    else if (menu == EDIT_PARAM)
    {
      editParm.value -= 1;
      redraw = 1;      
    }
    else if (menu == ANTS)
    {
            pan_pos-=10;
            redraw = 1;
    }
    else if (menu == ANT_TEST)
    {
            pan_pos+=5;
            redraw = 1;
    }
  }

  if(Lock_Button(Button_3(), &lock[3], 200)) // OK
  {
    if(menu == MAIN_MENU)
    {
      if (currentOption == 0) // start feeds
      {
        menu = START_FEEDS;
      }
      else if (currentOption == 1) // stop feeds
      {
        menu = STOP_FEEDS;
      }
      else if (currentOption == 2) // flight data
      {
        menu = FLIGHT_DATA;
        subMenu = 0;
        redraw = 1;
      }
      else if (currentOption == 3) // Antenna Stop
      {
        pan_pos = 90;
        tilt_pos = 90;
        menu = ANTS;
        subMenu = 0;
        redraw = 1;
      }
      else if (currentOption == 4) // get_params
      {
        menu = GET_PARAMS;
        paramsRecv = 0;
        timeOut = GET_PARAMS_TIMEOUT;
        redraw=1;
      }
      else if (currentOption == 5) // view/edit params
      {
        menu = EDIT_VIEW_PARAMS;
        subMenu = 0;
        lastOption = currentOption;
        currentOption = 0;        
        redraw = 1;
      }
      else if (currentOption == 6) // init_eeprom
      {
        menu = INIT_EEPROM;
        redraw = 1;
      }
      else if (currentOption == 7) // Antenna Test
      {
        original_pan_pos = pan_pos;
        original_tilt_pos = tilt_pos;
        original_chg_angle = chg_angle;
        pan_pos = 90;
        tilt_pos = 90;
        menu = ANT_TEST;
        subMenu = 0;
        redraw = 1;
      }
    }
    else if (menu == EDIT_VIEW_PARAMS) // edit single param based on current option
    {
      menu = 10; // edit single param
      redraw = 1;
    }
    else if (menu == 10) // save parameter / wait for ack
    {
      menu = 11; // save single param
      waitingAck = 1;
      timeOut = 100;
      redraw = 1;
    }
    else if ((menu==FLIGHT_DATA) && (subMenu==3))
    {
      SaveHomePosition();
    }
    else if (menu==ANTS)
    {
      menu=MAIN_MENU;
      redraw = 1;
    }
  }

  if(Lock_Button(Button_4(), &lock[4], 200)) // RIGHT
  {
    if (menu == FLIGHT_DATA)
    {
      if (subMenu < 3) {
        redraw=1;
        subMenu++;
      }
      else
      {
        subMenu=0;
        redraw=1;
      }
    }
    else if (menu == ANTS)
    {
//      if (subMenu < 2)
//      {
//        subMenu++;
//        redraw = 1;
//      }
          pan_pos+=10;
          redraw=1;
    }
     else if (menu == ANT_TEST)
    {
          pan_pos-=5;
          redraw=1;
    }
    else if (menu == 10)
    {
      editParm.value += 1;
      redraw = 1;      
    }
  }

  if(Lock_Button(Button_5(), &lock[5], 200)) // DOWN
  {
    if (menu == MAIN_MENU)
    {
      if (currentOption < 7)
      {
        redraw = 1;
        currentOption++;
      }
    }
    else if (menu == EDIT_VIEW_PARAMS)
    {
      currentOption++;
      redraw = 1;
    }
    else if (menu == ANT_TEST)
    {
      tilt_pos-=5;
      if (tilt_pos < tilt_pos_lower_limit) tilt_pos = tilt_pos_lower_limit;
      redraw = 1;
    }
    else if (menu == 10)
    {
      editParm.value -= .01;
      redraw = 1;      
    }
  } 
}
