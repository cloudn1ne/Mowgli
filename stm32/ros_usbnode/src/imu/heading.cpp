
#include <imu/heading.h>
#include <math.h>

Heading::Heading(void)
{
    m_max = (Heading::vector<int16_t>){-32767, -32767, -32767};
    m_min = (Heading::vector<int16_t>){+32767, +32767, +32767};
}


void Heading::setA(float x, float y, float z)
{
    a.x = x;
    a.y = y;
    a.z = z;
}

void Heading::setM(float x, float y, float z)
{
    m.x = x*10000*6842;
    m.y = y*10000*6842;
    m.z = z*10000*6842;
}




/*
Returns the angular difference in the horizontal plane between a
default vector and north, in degrees.

The default vector here is chosen to point along the surface of the
PCB, in the direction of the top of the text on the silkscreen.
This is the +X axis on the Pololu LSM303D carrier and the -Y axis on
the Pololu LSM303DLHC, LSM303DLM, and LSM303DLH carriers.
*/
float Heading::heading(void)
{
   return heading((vector<int>){1, 0, 0});
  //  return heading((vector<int>){0, -1, 0});  
}

void Heading::vector_normalize(vector<float> *a)
{
  float mag = sqrt(vector_dot(a, a));
  a->x /= mag;
  a->y /= mag;
  a->z /= mag;
}