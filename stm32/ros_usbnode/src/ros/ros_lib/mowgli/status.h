#ifndef _ROS_mowgli_status_h
#define _ROS_mowgli_status_h

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include "ros/msg.h"
#include "ros/time.h"

namespace mowgli
{

  class status : public ros::Msg
  {
    public:
      typedef ros::Time _stamp_type;
      _stamp_type stamp;
      typedef bool _rain_detected_type;
      _rain_detected_type rain_detected;
      typedef bool _emergency_left_wheel_lifted_type;
      _emergency_left_wheel_lifted_type emergency_left_wheel_lifted;
      typedef bool _emergency_right_wheel_lifted_type;
      _emergency_right_wheel_lifted_type emergency_right_wheel_lifted;
      typedef bool _emergency_tilt_accel_triggered_type;
      _emergency_tilt_accel_triggered_type emergency_tilt_accel_triggered;
      typedef bool _emergency_tilt_mech_triggered_type;
      _emergency_tilt_mech_triggered_type emergency_tilt_mech_triggered;
      typedef bool _emergency_stopbutton_triggered_type;
      _emergency_stopbutton_triggered_type emergency_stopbutton_triggered;
      typedef float _v_charge_type;
      _v_charge_type v_charge;
      typedef float _v_battery_type;
      _v_battery_type v_battery;
      typedef float _i_charge_type;
      _i_charge_type i_charge;
      typedef uint16_t _charge_pwm_type;
      _charge_pwm_type charge_pwm;
      typedef bool _is_charging_type;
      _is_charging_type is_charging;
      typedef bool _blade_motor_ctrl_enabled_type;
      _blade_motor_ctrl_enabled_type blade_motor_ctrl_enabled;
      typedef bool _drive_motor_ctrl_enabled_type;
      _drive_motor_ctrl_enabled_type drive_motor_ctrl_enabled;
      typedef uint32_t _left_encoder_ticks_type;
      _left_encoder_ticks_type left_encoder_ticks;
      typedef uint32_t _right_encoder_ticks_type;
      _right_encoder_ticks_type right_encoder_ticks;
      typedef uint8_t _left_power_type;
      _left_power_type left_power;
      typedef uint8_t _right_power_type;
      _right_power_type right_power;
      typedef float _imu_temp_type;
      _imu_temp_type imu_temp;
      typedef bool _blade_motor_enabled_type;
      _blade_motor_enabled_type blade_motor_enabled;
      typedef uint8_t _sw_ver_maj_type;
      _sw_ver_maj_type sw_ver_maj;
      typedef uint8_t _sw_ver_bra_type;
      _sw_ver_bra_type sw_ver_bra;
      typedef uint8_t _sw_ver_min_type;
      _sw_ver_min_type sw_ver_min;

    status():
      stamp(),
      rain_detected(0),
      emergency_left_wheel_lifted(0),
      emergency_right_wheel_lifted(0),
      emergency_tilt_accel_triggered(0),
      emergency_tilt_mech_triggered(0),
      emergency_stopbutton_triggered(0),
      v_charge(0),
      v_battery(0),
      i_charge(0),
      charge_pwm(0),
      is_charging(0),
      blade_motor_ctrl_enabled(0),
      drive_motor_ctrl_enabled(0),
      left_encoder_ticks(0),
      right_encoder_ticks(0),
      left_power(0),
      right_power(0),
      imu_temp(0),
      blade_motor_enabled(0),
      sw_ver_maj(0),
      sw_ver_bra(0),
      sw_ver_min(0)
    {
    }

    virtual int serialize(unsigned char *outbuffer) const override
    {
      int offset = 0;
      *(outbuffer + offset + 0) = (this->stamp.sec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.sec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.sec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.sec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.sec);
      *(outbuffer + offset + 0) = (this->stamp.nsec >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->stamp.nsec >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->stamp.nsec >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->stamp.nsec >> (8 * 3)) & 0xFF;
      offset += sizeof(this->stamp.nsec);
      union {
        bool real;
        uint8_t base;
      } u_rain_detected;
      u_rain_detected.real = this->rain_detected;
      *(outbuffer + offset + 0) = (u_rain_detected.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->rain_detected);
      union {
        bool real;
        uint8_t base;
      } u_emergency_left_wheel_lifted;
      u_emergency_left_wheel_lifted.real = this->emergency_left_wheel_lifted;
      *(outbuffer + offset + 0) = (u_emergency_left_wheel_lifted.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->emergency_left_wheel_lifted);
      union {
        bool real;
        uint8_t base;
      } u_emergency_right_wheel_lifted;
      u_emergency_right_wheel_lifted.real = this->emergency_right_wheel_lifted;
      *(outbuffer + offset + 0) = (u_emergency_right_wheel_lifted.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->emergency_right_wheel_lifted);
      union {
        bool real;
        uint8_t base;
      } u_emergency_tilt_accel_triggered;
      u_emergency_tilt_accel_triggered.real = this->emergency_tilt_accel_triggered;
      *(outbuffer + offset + 0) = (u_emergency_tilt_accel_triggered.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->emergency_tilt_accel_triggered);
      union {
        bool real;
        uint8_t base;
      } u_emergency_tilt_mech_triggered;
      u_emergency_tilt_mech_triggered.real = this->emergency_tilt_mech_triggered;
      *(outbuffer + offset + 0) = (u_emergency_tilt_mech_triggered.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->emergency_tilt_mech_triggered);
      union {
        bool real;
        uint8_t base;
      } u_emergency_stopbutton_triggered;
      u_emergency_stopbutton_triggered.real = this->emergency_stopbutton_triggered;
      *(outbuffer + offset + 0) = (u_emergency_stopbutton_triggered.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->emergency_stopbutton_triggered);
      union {
        float real;
        uint32_t base;
      } u_v_charge;
      u_v_charge.real = this->v_charge;
      *(outbuffer + offset + 0) = (u_v_charge.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_v_charge.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_v_charge.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_v_charge.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->v_charge);
      union {
        float real;
        uint32_t base;
      } u_v_battery;
      u_v_battery.real = this->v_battery;
      *(outbuffer + offset + 0) = (u_v_battery.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_v_battery.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_v_battery.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_v_battery.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->v_battery);
      union {
        float real;
        uint32_t base;
      } u_i_charge;
      u_i_charge.real = this->i_charge;
      *(outbuffer + offset + 0) = (u_i_charge.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_i_charge.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_i_charge.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_i_charge.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->i_charge);
      *(outbuffer + offset + 0) = (this->charge_pwm >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->charge_pwm >> (8 * 1)) & 0xFF;
      offset += sizeof(this->charge_pwm);
      union {
        bool real;
        uint8_t base;
      } u_is_charging;
      u_is_charging.real = this->is_charging;
      *(outbuffer + offset + 0) = (u_is_charging.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->is_charging);
      union {
        bool real;
        uint8_t base;
      } u_blade_motor_ctrl_enabled;
      u_blade_motor_ctrl_enabled.real = this->blade_motor_ctrl_enabled;
      *(outbuffer + offset + 0) = (u_blade_motor_ctrl_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->blade_motor_ctrl_enabled);
      union {
        bool real;
        uint8_t base;
      } u_drive_motor_ctrl_enabled;
      u_drive_motor_ctrl_enabled.real = this->drive_motor_ctrl_enabled;
      *(outbuffer + offset + 0) = (u_drive_motor_ctrl_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->drive_motor_ctrl_enabled);
      *(outbuffer + offset + 0) = (this->left_encoder_ticks >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->left_encoder_ticks >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->left_encoder_ticks >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->left_encoder_ticks >> (8 * 3)) & 0xFF;
      offset += sizeof(this->left_encoder_ticks);
      *(outbuffer + offset + 0) = (this->right_encoder_ticks >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (this->right_encoder_ticks >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (this->right_encoder_ticks >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (this->right_encoder_ticks >> (8 * 3)) & 0xFF;
      offset += sizeof(this->right_encoder_ticks);
      *(outbuffer + offset + 0) = (this->left_power >> (8 * 0)) & 0xFF;
      offset += sizeof(this->left_power);
      *(outbuffer + offset + 0) = (this->right_power >> (8 * 0)) & 0xFF;
      offset += sizeof(this->right_power);
      union {
        float real;
        uint32_t base;
      } u_imu_temp;
      u_imu_temp.real = this->imu_temp;
      *(outbuffer + offset + 0) = (u_imu_temp.base >> (8 * 0)) & 0xFF;
      *(outbuffer + offset + 1) = (u_imu_temp.base >> (8 * 1)) & 0xFF;
      *(outbuffer + offset + 2) = (u_imu_temp.base >> (8 * 2)) & 0xFF;
      *(outbuffer + offset + 3) = (u_imu_temp.base >> (8 * 3)) & 0xFF;
      offset += sizeof(this->imu_temp);
      union {
        bool real;
        uint8_t base;
      } u_blade_motor_enabled;
      u_blade_motor_enabled.real = this->blade_motor_enabled;
      *(outbuffer + offset + 0) = (u_blade_motor_enabled.base >> (8 * 0)) & 0xFF;
      offset += sizeof(this->blade_motor_enabled);
      *(outbuffer + offset + 0) = (this->sw_ver_maj >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sw_ver_maj);
      *(outbuffer + offset + 0) = (this->sw_ver_bra >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sw_ver_bra);
      *(outbuffer + offset + 0) = (this->sw_ver_min >> (8 * 0)) & 0xFF;
      offset += sizeof(this->sw_ver_min);
      return offset;
    }

    virtual int deserialize(unsigned char *inbuffer) override
    {
      int offset = 0;
      this->stamp.sec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.sec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.sec);
      this->stamp.nsec =  ((uint32_t) (*(inbuffer + offset)));
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->stamp.nsec |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->stamp.nsec);
      union {
        bool real;
        uint8_t base;
      } u_rain_detected;
      u_rain_detected.base = 0;
      u_rain_detected.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->rain_detected = u_rain_detected.real;
      offset += sizeof(this->rain_detected);
      union {
        bool real;
        uint8_t base;
      } u_emergency_left_wheel_lifted;
      u_emergency_left_wheel_lifted.base = 0;
      u_emergency_left_wheel_lifted.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->emergency_left_wheel_lifted = u_emergency_left_wheel_lifted.real;
      offset += sizeof(this->emergency_left_wheel_lifted);
      union {
        bool real;
        uint8_t base;
      } u_emergency_right_wheel_lifted;
      u_emergency_right_wheel_lifted.base = 0;
      u_emergency_right_wheel_lifted.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->emergency_right_wheel_lifted = u_emergency_right_wheel_lifted.real;
      offset += sizeof(this->emergency_right_wheel_lifted);
      union {
        bool real;
        uint8_t base;
      } u_emergency_tilt_accel_triggered;
      u_emergency_tilt_accel_triggered.base = 0;
      u_emergency_tilt_accel_triggered.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->emergency_tilt_accel_triggered = u_emergency_tilt_accel_triggered.real;
      offset += sizeof(this->emergency_tilt_accel_triggered);
      union {
        bool real;
        uint8_t base;
      } u_emergency_tilt_mech_triggered;
      u_emergency_tilt_mech_triggered.base = 0;
      u_emergency_tilt_mech_triggered.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->emergency_tilt_mech_triggered = u_emergency_tilt_mech_triggered.real;
      offset += sizeof(this->emergency_tilt_mech_triggered);
      union {
        bool real;
        uint8_t base;
      } u_emergency_stopbutton_triggered;
      u_emergency_stopbutton_triggered.base = 0;
      u_emergency_stopbutton_triggered.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->emergency_stopbutton_triggered = u_emergency_stopbutton_triggered.real;
      offset += sizeof(this->emergency_stopbutton_triggered);
      union {
        float real;
        uint32_t base;
      } u_v_charge;
      u_v_charge.base = 0;
      u_v_charge.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_v_charge.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_v_charge.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_v_charge.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->v_charge = u_v_charge.real;
      offset += sizeof(this->v_charge);
      union {
        float real;
        uint32_t base;
      } u_v_battery;
      u_v_battery.base = 0;
      u_v_battery.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_v_battery.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_v_battery.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_v_battery.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->v_battery = u_v_battery.real;
      offset += sizeof(this->v_battery);
      union {
        float real;
        uint32_t base;
      } u_i_charge;
      u_i_charge.base = 0;
      u_i_charge.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_i_charge.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_i_charge.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_i_charge.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->i_charge = u_i_charge.real;
      offset += sizeof(this->i_charge);
      this->charge_pwm =  ((uint16_t) (*(inbuffer + offset)));
      this->charge_pwm |= ((uint16_t) (*(inbuffer + offset + 1))) << (8 * 1);
      offset += sizeof(this->charge_pwm);
      union {
        bool real;
        uint8_t base;
      } u_is_charging;
      u_is_charging.base = 0;
      u_is_charging.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->is_charging = u_is_charging.real;
      offset += sizeof(this->is_charging);
      union {
        bool real;
        uint8_t base;
      } u_blade_motor_ctrl_enabled;
      u_blade_motor_ctrl_enabled.base = 0;
      u_blade_motor_ctrl_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->blade_motor_ctrl_enabled = u_blade_motor_ctrl_enabled.real;
      offset += sizeof(this->blade_motor_ctrl_enabled);
      union {
        bool real;
        uint8_t base;
      } u_drive_motor_ctrl_enabled;
      u_drive_motor_ctrl_enabled.base = 0;
      u_drive_motor_ctrl_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->drive_motor_ctrl_enabled = u_drive_motor_ctrl_enabled.real;
      offset += sizeof(this->drive_motor_ctrl_enabled);
      this->left_encoder_ticks =  ((uint32_t) (*(inbuffer + offset)));
      this->left_encoder_ticks |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->left_encoder_ticks |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->left_encoder_ticks |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->left_encoder_ticks);
      this->right_encoder_ticks =  ((uint32_t) (*(inbuffer + offset)));
      this->right_encoder_ticks |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      this->right_encoder_ticks |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      this->right_encoder_ticks |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      offset += sizeof(this->right_encoder_ticks);
      this->left_power =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->left_power);
      this->right_power =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->right_power);
      union {
        float real;
        uint32_t base;
      } u_imu_temp;
      u_imu_temp.base = 0;
      u_imu_temp.base |= ((uint32_t) (*(inbuffer + offset + 0))) << (8 * 0);
      u_imu_temp.base |= ((uint32_t) (*(inbuffer + offset + 1))) << (8 * 1);
      u_imu_temp.base |= ((uint32_t) (*(inbuffer + offset + 2))) << (8 * 2);
      u_imu_temp.base |= ((uint32_t) (*(inbuffer + offset + 3))) << (8 * 3);
      this->imu_temp = u_imu_temp.real;
      offset += sizeof(this->imu_temp);
      union {
        bool real;
        uint8_t base;
      } u_blade_motor_enabled;
      u_blade_motor_enabled.base = 0;
      u_blade_motor_enabled.base |= ((uint8_t) (*(inbuffer + offset + 0))) << (8 * 0);
      this->blade_motor_enabled = u_blade_motor_enabled.real;
      offset += sizeof(this->blade_motor_enabled);
      this->sw_ver_maj =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sw_ver_maj);
      this->sw_ver_bra =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sw_ver_bra);
      this->sw_ver_min =  ((uint8_t) (*(inbuffer + offset)));
      offset += sizeof(this->sw_ver_min);
     return offset;
    }

    virtual const char * getType() override { return "mowgli/status"; };
    virtual const char * getMD5() override { return "15c620b7cc55c2285587da2dc5df3c34"; };

  };

}
#endif
