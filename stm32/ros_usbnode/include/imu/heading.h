#ifndef HEADING_h
#define HEADING_h

#include <stdint.h>
#include <math.h>
#include "main.h"

#define MIN(a,b) (((a)<(b))?(a):(b))
#define MAX(a,b) (((a)>(b))?(a):(b))

class Heading
{
  public:
    template <typename T> struct vector
    {
      T x, y, z;
    };

    vector<int16_t> a; // accelerometer readings
    vector<int16_t> m; // magnetometer readings

    vector<int16_t> m_min; // minimum magnetometer readings
    vector<int16_t> m_max; // maximum magnetometer readings
    
    Heading(void);

    void setA(float x, float y, float z);
    void setM(float x, float y, float z);

    float heading(void);
    template <typename T> float heading(vector<T> from);

    // vector functions
    template <typename Ta, typename Tb, typename To> static void vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out);
    template <typename Ta, typename Tb> static float vector_dot(const vector<Ta> *a, const vector<Tb> *b);
    static void vector_normalize(vector<float> *a);

    
};

    
    /*
    Returns the angular difference in the horizontal plane between the
    "from" vector and north, in degrees.

    Description of heading algorithm:
    Shift and scale the magnetic reading based on calibration data to find
    the North vector. Use the acceleration readings to determine the Up
    vector (gravity is measured as an upward acceleration). The cross
    product of North and Up vectors is East. The vectors East and North
    form a basis for the horizontal plane. The From vector is projected
    into the horizontal plane and the angle between the projected vector
    and horizontal north is returned.
    */
    template <typename T> float Heading::heading(vector<T> from)
    {
        vector<int32_t> temp_m = {m.x, m.y, m.z};

        m_min.x =  MIN(m_min.x, m.x);
        m_min.y =  MIN(m_min.y, m.y);
        m_min.z =  MIN(m_min.z, m.z);

        m_max.x =  MAX(m_max.x, m.x);
        m_max.y=  MAX(m_max.y, m.y);
        m_max.z =  MAX(m_max.z, m.z);

        // subtract offset (average of min and max) from magnetometer readings
        temp_m.x -= ((int32_t)m_min.x + m_max.x) / 2;
        temp_m.y -= ((int32_t)m_min.y + m_max.y) / 2;
        temp_m.z -= ((int32_t)m_min.z + m_max.z) / 2;


        debug_printf("x: %d, y: %d, z: %d \r\n", m_min.x, m_min.y, m_min.z);
        debug_printf("x: %d, y: %d, z: %d \r\n", m_max.x, m_max.y, m_max.z);
        debug_printf("x: %d, y: %d, z: %d \r\n\r\n", temp_m.x, temp_m.y, temp_m.z);
        // compute E and N
        vector<float> E;
        vector<float> N;
        vector_cross(&temp_m, &a, &E);
        vector_normalize(&E);
        vector_cross(&a, &E, &N);
        vector_normalize(&N);

        // compute heading
        float heading = atan2(vector_dot(&E, &from), vector_dot(&N, &from)) * 180 / M_PI;
        if (heading < 0) heading += 360;
        return heading;
    }

    template <typename Ta, typename Tb, typename To> void Heading::vector_cross(const vector<Ta> *a, const vector<Tb> *b, vector<To> *out)
    {
        out->x = (a->y * b->z) - (a->z * b->y);
        out->y = (a->z * b->x) - (a->x * b->z);
        out->z = (a->x * b->y) - (a->y * b->x);
    }

    template <typename Ta, typename Tb> float Heading::vector_dot(const vector<Ta> *a, const vector<Tb> *b)
    {
        return (a->x * b->x) + (a->y * b->y) + (a->z * b->z);
    }


#endif