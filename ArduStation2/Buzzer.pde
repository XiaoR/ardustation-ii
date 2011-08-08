void buzz(int time, int freq)
{
  #ifdef BUZZERON
  cli();
  for(int c=0; c<time; c++)
  {
    digitalWrite(14,HIGH); 
    delayMicroseconds(freq);
    digitalWrite(14,LOW); 
    delayMicroseconds(freq);
  }
  sei();
  #endif
}
