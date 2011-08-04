/*************************************************************************
 * //Function to calculate the course between two waypoints
 * //I'm using the real formulas--no lookup table fakes!
 *************************************************************************/

float calc_bearing(float flat1, float flon1, float flat2, float flon2)
{
  float calc;
  float bear_calc;

  float x = 69.1 * (flat2 - flat1); 
  float y = 69.1 * (flon2 - flon1) * cos(flat1/57.3);
 
  calc=atan2(y,x);

  bear_calc= degrees(calc);

  if(bear_calc<=1){
    bear_calc=360+bear_calc; 
  }
  return bear_calc;
}
/*************************************************************************
 * //Function to calculate the distance between two waypoints
 * //I'm using  a really good approach
 *************************************************************************/
float calc_dist(float flat1, float flon1, float flat2, float flon2)
{
 float x = 69.1 * (flat2 - flat1); 
 float y = 69.1 * (flon2 - flon1) * cos(flat1/57.3);
 
 return (float)sqrt((float)(x*x) + (float)(y*y))*1609.344; 
}

float to_float_6(long value) 
{
  return (float)value/(float)1000000;
}
