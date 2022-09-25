#ifndef _ROS_SERVICE_Led_h
#define _ROS_SERVICE_Led_h
#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"

namespace mowgli
{

static const char LED[] = "mowgli/Led";

  class LedRequest : public ros::Msg
  {
    public:
      typedef uint8_t _led_type;
      _led_type led;
      enum { LED_4H = 1 };
      enum { LED_6H = 2 };
      enum { LED_8H = 3 };
      enum { LED_10H = 4 };

    LedRequest():
      led(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->led >> (8 * 0)) & 0xFF;
      offset += sizeof(this->led);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->led =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->led);
     return offset;
    }

    virtual const char * getType() override { return LED; };
    virtual const char * getMD5() override { return "62899260de9e3cee72226266c0ba29e2"; };

  };

  class LedResponse : public ros::Msg
  {
    public:

    LedResponse()
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
     return offset;
    }

    virtual const char * getType() override { return LED; };
    virtual const char * getMD5() override { return "d41d8cd98f00b204e9800998ecf8427e"; };

  };

  class Led {
    public:
    typedef LedRequest Request;
    typedef LedResponse Response;
  };

}
#endif
