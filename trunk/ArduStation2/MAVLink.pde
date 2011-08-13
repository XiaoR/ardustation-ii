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
      gpsfix = packet.fix_type;
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

      mavlink_sys_status_t packet;
      mavlink_msg_sys_status_decode(msg, &packet);
      currentSMode = packet.mode;
      currentNMode = packet.nav_mode;
      battery = packet.vbat;
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
