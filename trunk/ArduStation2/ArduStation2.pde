// Ardustation 2
//
// This code is highly alpha quality, be aware...
//
// This is was original created by psmitty for APM and AC2 based upon orginal Ardustation code
//
// Modified to add tilt/pan functionality by Heino Pull (heyno@heino.com) 7/23/2011
// Used readymaderc.com tilt.pan solution with HITEC-485 servos. http://www.readymaderc.com/store/index.php?main_page=product_info&cPath=19_7&products_id=96
// for development and testing. 
// RAM is really tight so I had to remove the PID modification for Ardupilot Mega.
// 
// Modified to add battery monitoring, and manual antenna control by Hai Tran (hai@itsecurity.net.au) 4 August 2011
// Cleaned up references to PIDS menu and replaced with Antenna Stop menu (ANTS)
// Manual antenna control menu item is called ANT_TEST in the code, and called Antenna Test in the main menu
// User should set tilt_pos_upper_limit and tilt_pos_lower_limit to stop the tilt from overshooting. Start with 0 and 180 and when in the antenna testing menu find out what the min and max should be for your setup and set those values in the code.

#include <FastSerial.h>
#include <GCS_MAVLink.h>
#include <avr/pgmspace.h>
#include <LiquidCrystal.h>
#include <EEPROM.h>
#include <AP_EEPROMB.h> 		// ArduPilot Mega RC Library


#include <Servo.h>
#include "parameter.h"
AP_EEPROMB ee;
#undef PROGMEM 
#define PROGMEM __attribute__(( section(".progmem.data") )) 

#undef PSTR 
#define PSTR(s) (__extension__({static const char __c[] PROGMEM = (s); &__c[0];})) 

#define SERIAL_BAUD 57600
//#define BUZZERON // is the buzzer on?
FastSerialPort0(Serial);

// data streams active and rates
#define MAV_DATA_STREAM_POSITION_ACTIVE 1
#define MAV_DATA_STREAM_RAW_SENSORS_ACTIVE 1
#define MAV_DATA_STREAM_EXTENDED_STATUS_ACTIVE 1
#define MAV_DATA_STREAM_RAW_CONTROLLER_ACTIVE 1
#define MAV_DATA_STREAM_EXTRA1_ACTIVE 1

// update rate is times per second (hz)
#define MAV_DATA_STREAM_POSITION_RATE 1
#define MAV_DATA_STREAM_RAW_SENSORS_RATE 1
#define MAV_DATA_STREAM_EXTENDED_STATUS_RATE 1
#define MAV_DATA_STREAM_RAW_CONTROLLER_RATE 1
#define MAV_DATA_STREAM_EXTRA1_RATE 1

#define GET_PARAMS_TIMEOUT 200 //(20 seconds)

// store strings in flash memory
#define MAIN_MENU 0
#define START_FEEDS 1
#define STOP_FEEDS 2
#define FLIGHT_DATA 3
#define ANTS 4
#define GET_PARAMS 5
#define EDIT_VIEW_PARAMS 6
#define INIT_EEPROM 7
#define ANT_TEST 8
#define EDIT_PARAM 10
#define SAVE_PARAM 11

#define TOTAL_PARAMS 31

LiquidCrystal lcd(2, 3, 4, 5, 6, 7, 8); //Initailizing LCD object (this is real C++).

void lcd_print_P(const char *string)
{
  char c;
  while( (c = pgm_read_byte(string++)) )
    lcd.write(c);
}

#define toRad(x) (x*PI)/180.0
#define toDeg(x) (x*180.0)/PI

unsigned long hb_timer;
unsigned long timer;

#define SERVO_MAX 2600 //Range of servos pitch and roll
#define SERVO_MIN 400
#define TEST_PAN 0 //Test pan min and max (for calibration)
#define TEST_SOUTH 0 //test the south (just poing to south)
#define TEST_TILT 0 //test the tilt max and min (for calibration)
Servo Pan;
Servo Tilt;
// Servo Cal variables
int pan_pos = 90;
int tilt_pos = 90;
int chg_angle = 0;
int original_pan_pos = 90;
int original_tilt_pos = 90;
int original_chg_angle = 0;
#define tilt_pos_upper_limit 150 // Upper tilt limit (antenna points to the sky)
#define tilt_pos_lower_limit 50 //  Lower tilt limit (anntenna points straight ahead)


float Latitude_Home=0;
float Longitud_Home=0;
int Altitude_Home = 0;
float Distance_Home=0;
float Distance3D_Home=0;
int Angle_Home=0;
int Constrain_Angle_Home = 0;
float Bearing_Home=0;
float SvBearingHome = 0;

float offset = 0;
// flight data
int altitude=0;
float pitch=0;
float roll=0;
float yaw=0;
float longitude=0;
float latitude=0;
float velocity = 0;
int numSats=0;
int battery=0;
int currentSMode=0;
int currentNMode=0;

// menus etc
int menu=0;
int subMenu=0;
int currentOption=0;
int lastOption=0;
int timeOut=0;
int redraw=1;
int waitingAck=0;
parameter editParm;
int paramsRecv=0;
int beat=0;

byte heart[8] = {
  B00000,
  B01010,
  B11111,
  B11111,
  B01110,
  B00100,
  B00000,
  B00000,
};

//byte upArrow[8] = {
//  B00000,
//  B00100,
//  B01110,
//  B10101,
//  B00100,
//  B00100,
//  B00100,
//  B00000,
//};
//
//byte downArrow[8] = {
//  B00000,
//  B00100,
//  B00100,
//  B00100,
//  B10101,
//  B01110,
//  B00100,
//  B00000,
//};
//
//byte leftArrow[8] = {
//  B00000,
//  B00100,
//  B01000,
//  B11111,
//  B01000,
//  B00100,
//  B00000,
//  B00000,
//};
//
//byte rightArrow[8] = {
//  B00000,
//  B00100,
//  B00010,
//  B11111,
//  B00010,
//  B00100,
//  B00000,
//  B00000,
//};
    

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

// I'm doing this ugliness because for some reason fastserial breaks storing arrays in flash memory...
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

void gcs_update()
{
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;

    // process received bytes
    while(Serial.available())
    {
        uint8_t c = Serial.read();
        // Try to get a new message
        if(mavlink_parse_char(0, c, &msg, &status)) gcs_handleMessage(&msg);
    }
}

void gcs_handleMessageSysStatus(mavlink_message_t* msg)
{
  mavlink_sys_status_t packet;
  mavlink_msg_sys_status_decode(msg, &packet);
  currentSMode = packet.mode;
  currentNMode = packet.nav_mode;
  battery = packet.vbat;
  return;  
}

void gcs_handleMessage(mavlink_message_t* msg)
{
  switch (msg->msgid) {
    case MAVLINK_MSG_ID_HEARTBEAT:
    {
      beat = 1;
      break;
    }
   case MAVLINK_MSG_ID_ATTITUDE:
    {
      // decode
      mavlink_attitude_t packet;
      mavlink_msg_attitude_decode(msg, &packet);
      pitch = toDeg(packet.pitch);
      yaw = toDeg(packet.yaw);
      roll = toDeg(packet.roll);
      break;
    }
    case MAVLINK_MSG_ID_GPS_RAW:
    {
      // decode
      mavlink_gps_raw_t packet;
      mavlink_msg_gps_raw_decode(msg, &packet);
      latitude = packet.lat;
      longitude = packet.lon;
      velocity = packet.v;
      altitude = packet.alt;
      position_antenna();
      break;
    }
    case MAVLINK_MSG_ID_GPS_STATUS:
    {
      mavlink_gps_status_t packet;
      mavlink_msg_gps_status_decode(msg, &packet);        
      numSats = packet.satellites_visible;
      break;
    }
    case MAVLINK_MSG_ID_RAW_PRESSURE:
    {
      // decode
      mavlink_raw_pressure_t packet;
      mavlink_msg_raw_pressure_decode(msg, &packet);
      break;
    }
    case MAVLINK_MSG_ID_SYS_STATUS:
    {
      gcs_handleMessageSysStatus(msg);
      mavlink_sys_status_t packet;
      mavlink_msg_sys_status_decode(msg, &packet);
      currentSMode = packet.mode;
      currentNMode = packet.nav_mode;
      break;
    }
    case MAVLINK_MSG_ID_PARAM_VALUE:
    {
      // decode
      mavlink_param_value_t packet;
      mavlink_msg_param_value_decode(msg, &packet);
      const char * key = (const char*) packet.param_id;
      int loc = find_param(key);
      if (loc != -1)
      {
        float value;
        parameter temp;
        //strcpy(temp.key, key);
        //temp.value = packet.param_value;
        value = packet.param_value;
        EEPROM_writeFloat((loc * sizeof(temp))+sizeof(temp.key), value);         
        //EEPROM_writeParameter((loc * sizeof(temp))+sizeof(temp.key), temp);         
        paramsRecv++;
      }
      if (waitingAck == 1)
      {        
        if (strcmp(key, editParm.key) == 0)
        {
          waitingAck = 0;
        }
      }
      //else
      //  timeOut = 100; // each time we get another parameter reset the timeout
      //redraw = 1;
      break;
    }
  }
}

void lcdPrintString(int stringId, const char *table[])
{
    char buffer[21];
    strcpy_P(buffer, (char*)pgm_read_word(&(table[stringId])));
    lcd.print(buffer);
}

void send_message(mavlink_message_t* msg)
{
  Serial.write(MAVLINK_STX);
  Serial.write(msg->len);
  Serial.write(msg->seq);
  Serial.write(msg->sysid);
  Serial.write(msg->compid);
  Serial.write(msg->msgid);
  for(uint16_t i = 0; i < msg->len; i++)
  {
    Serial.write(msg->payload[i]);
  }
  Serial.write(msg->ck_a);
  Serial.write(msg->ck_b);
}

void setup()
{
  Serial.begin(SERIAL_BAUD);
  lcd.createChar(0, heart);
//  lcd.createChar(1, downArrow);
//  lcd.createChar(2, upArrow);
//  lcd.createChar(3, leftArrow);
//  lcd.createChar(4, rightArrow);
  
  //Servo Calibration!!!!
  Pan.attach(9,609,2255);//PAN Servos  //These numbers seem to work well for the HITEC-485 servos 
  Tilt.attach(10,591,2235);//Tilt Elevator
  
  Pan.write(pan_pos);
  Tilt.write(tilt_pos);

    
  delay(4000);
  
  
  lcd.begin(20, 4);
  pinMode(14,OUTPUT); //Pin mode as output to control buzzer (analog0)
  pinMode(11,OUTPUT);
  lcd.clear();
  lcd_print_P(PSTR("   ArduStation 2"));
  lcd.setCursor(0, 1);
  lcd_print_P(PSTR("   with MAVLink"));
  lcd.setCursor(0, 2);
  lcd_print_P(PSTR("       and"));
  lcd.setCursor(0,3);
  lcd_print_P(PSTR(" Antenna Tracking "));
  delay(2000);
  if (EEPROM_readRevision() != 0)
  {
    init_EEPROM();
    EEPROM_writeRevision();
  } 
  RestoreHomePosition(); 
  lcd.clear();
  lcd_print_P(PSTR("Starting xbee..."));
  Serial.println("...");
  delay(2000);
}

void loop()
{
  switch (menu)
  {
    case MAIN_MENU: // main menu
    {
      if (redraw == 1)
        main_menu();
      break;
    }
    case START_FEEDS: // start feeds
    {
      start_feeds();
      break;
    }
    case STOP_FEEDS: // stop feeds
    {
      stop_feeds();
      break;
    }  

    case FLIGHT_DATA: // flight data
    {
      flight_data();
      break;
    }
    case ANTS: // ANTS  - replaced PIDS adjustment with set servo hard stop menu
    {
      if (redraw == 1)
        SetServoHardStop();
      break;
    }
    case GET_PARAMS: // get params
    {
      get_params();
      //EEPROM_writeParamCount(totalParams);
      break;
    }
    case EDIT_VIEW_PARAMS: // view/edit params
    {
      if (redraw == 1)
        list_params();
      break; 
    }
    case INIT_EEPROM: // init eeprom
    {
      init_EEPROM();
      break;
    }
    case ANT_TEST: //Antenna Test
    {
      if (redraw == 1)
        SetAntennaPosition();
      break;
    }
    case 10:
    {
      if (redraw == 1)
        edit_params();
      break;
    }
    case 11:
    {
      save_param();
      break; 
    }
  }
  if (millis() - hb_timer > 500)
  {
    hb_timer = millis();
    lcd.setCursor(19,3);
    if (beat == 1)
    {
      beat = 0;
      lcd.write(0);
    }
    else
      lcd.print(" ");
  }
  gcs_update();
  Check_Buttons(4);
}

void main_menu() // menu 0
{
  lcd.clear();
  if (currentOption < 4)
  {
    lcd.setCursor(2,0);
    lcd_print_P(PSTR("Start Feeds"));
    lcd.setCursor(2,1);
    lcd_print_P(PSTR("Stop Feeds"));
    lcd.setCursor(2,2);
    lcd_print_P(PSTR("Flight Data"));
    lcd.setCursor(2,3);
    lcd_print_P(PSTR("Antenna Stop"));
    lcd.setCursor(18,3);
    lcd.write(1);
  }
  if (currentOption > 3 && currentOption < 8)
  {
    lcd.setCursor(18,0);
    lcd.write(2);
    lcd.setCursor(2,0);
    lcd_print_P(PSTR("Get Params"));
    lcd.setCursor(2,1);
    lcd_print_P(PSTR("Edit/View Params"));
    lcd.setCursor(2,2);
    lcd_print_P(PSTR("Init EEPROM"));
    lcd.setCursor(2,3);
    lcd_print_P(PSTR("Antenna Test"));
  }
  lcd.setCursor(0,currentOption%4);
  lcd.print(">>");
  redraw = 0;  
}

void start_feeds()
{
  lcd.clear();
  lcd_print_P(PSTR("Starting feeds!"));
  mavlink_message_t msg2;
  mavlink_msg_request_data_stream_pack(127, 0, &msg2, 7, 1, MAV_DATA_STREAM_RAW_SENSORS, MAV_DATA_STREAM_RAW_SENSORS_RATE, MAV_DATA_STREAM_RAW_SENSORS_ACTIVE);
  send_message(&msg2);
  delay(10);
  mavlink_message_t msg3;
  mavlink_msg_request_data_stream_pack(127, 0, &msg3, 7, 1, MAV_DATA_STREAM_EXTENDED_STATUS, MAV_DATA_STREAM_EXTENDED_STATUS_RATE, MAV_DATA_STREAM_EXTENDED_STATUS_ACTIVE);
  send_message(&msg3);
  delay(10);
  mavlink_message_t msg4;
  mavlink_msg_request_data_stream_pack(127, 0, &msg4, 7, 1, MAV_DATA_STREAM_RAW_CONTROLLER, MAV_DATA_STREAM_RAW_CONTROLLER_RATE, MAV_DATA_STREAM_RAW_CONTROLLER_ACTIVE);
  send_message(&msg4);
  delay(10);
  mavlink_message_t msg1;
  mavlink_msg_request_data_stream_pack(127, 0, &msg1, 7, 1, MAV_DATA_STREAM_POSITION, MAV_DATA_STREAM_POSITION_RATE, MAV_DATA_STREAM_POSITION_ACTIVE);
  send_message(&msg1);
  delay(10);
  mavlink_message_t msg5;
  mavlink_msg_request_data_stream_pack(127, 0, &msg5, 7, 1, MAV_DATA_STREAM_EXTRA1, MAV_DATA_STREAM_EXTRA1_RATE, MAV_DATA_STREAM_EXTRA1_ACTIVE);
  send_message(&msg5);
  delay(460);
  menu = MAIN_MENU;
  redraw = 1;
}

void stop_feeds()
{
  lcd.clear();
  lcd_print_P(PSTR("Stopping feeds!"));
  mavlink_message_t msg1;
  mavlink_msg_request_data_stream_pack(127, 0, &msg1, 7, 1, MAV_DATA_STREAM_ALL, 0, 0);
  send_message(&msg1);
  delay(500);
  menu = MAIN_MENU;
  redraw = 1;
}

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
        lcd_print_P(PSTR("NSat:"));
        lcd.print(numSats);
        lcd.setCursor(0,1);
        lcd_print_P(PSTR("RL:"));
        lcd.print(roll);
        lcd.setCursor(0,2);
        lcd_print_P(PSTR("YW:"));
        lcd.print(yaw);
        lcd.setCursor(0,3);
        lcd_print_P(PSTR("AT:"));
        lcd.print(altitude*3.280839);
        lcd.setCursor(10,0);
        lcd_print_P(PSTR("VL:"));
        lcd.print(velocity);   
        lcd.setCursor(10,1);
        lcd_print_P(PSTR("LT:"));
        lcd.print(latitude,3);   
        lcd.setCursor(10,2);
        lcd_print_P(PSTR("LN:"));
        lcd.print(longitude,3);
        lcd.setCursor(10,3);
         lcd_print_P(PSTR("BT:"));
        lcd.print(battery);
        break;
      }
      case 1:
      {
      lcd.clear();
      lcd_print_P(PSTR("Pan~"));
      lcd.print(Bearing_Home);
      lcd.setCursor(10, 0);
      lcd_print_P(PSTR("Tilt~"));
      lcd.print(Constrain_Angle_Home);
      lcd.setCursor(0, 1);
      lcd_print_P(PSTR("VertAng "));
      lcd.print(Angle_Home);

      lcd.setCursor(0, 2);
      lcd_print_P(PSTR("ALT~"));
      lcd.print(altitude*3.280839);
      lcd.setCursor(10, 2);
      lcd_print_P(PSTR("Ant~"));
      lcd.print(SvBearingHome);
      lcd.setCursor(10, 3);
      lcd_print_P(PSTR("Batt~"));
      lcd.print(battery);
      break;
    
      }
      case 2:
      {      
      Distance_Home=calc_dist(Latitude_Home, Longitud_Home, latitude, longitude);
      Distance3D_Home= sqrt((Distance_Home*Distance_Home)+((altitude-Altitude_Home)*(altitude-Altitude_Home)));
      
      
      lcd.clear();
      lcd_print_P(PSTR("Dist2D~"));
      lcd.print(Distance_Home*3.280839);
      lcd.setCursor(0, 1);
      lcd_print_P(PSTR("Dist3D~"));
      lcd.print(Distance3D_Home*3.280839);
      lcd.setCursor(0, 2);
      lcd_print_P(PSTR("Angle~"));
      lcd.print(Angle_Home);
      lcd.setCursor(10, 2);
      lcd_print_P(PSTR(" Alt~"));
      lcd.print(altitude*3.280839);
      lcd.setCursor(0, 3);
      lcd_print_P(PSTR("Dir~"));
      lcd.print(Bearing_Home);
      
    break;
      }
    case 3:
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

int availableMemory() {
  int size = 2048;
  byte *buf;

  while ((buf = (byte *) malloc(--size)) == NULL)
    ;

  free(buf);

  return size;
}
