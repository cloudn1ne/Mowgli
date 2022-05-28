/*
 * cpp_main.c
 *
 */

#include <cpp_main.h>
#include "main.h"
#include "stm32f1xx_hal.h"
#include "ringbuffer.h"
#include "ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "nbt.h"
#include "geometry_msgs/Twist.h"

extern uint8_t RxBuffer[RxBufferSize];
struct ringbuffer rb;

ros::NodeHandle nh;

// std_msgs::String str_msg;
std_msgs::Float32 f32_battery_voltage_msg;
std_msgs::Float32 f32_charge_voltage_msg;
std_msgs::Int16 int16_charge_pwm_msg;

/*
 * PUBLISHERS
 */
// ros::Publisher chatter("version", &str_msg);
ros::Publisher pubBatteryVoltage("battery_voltage", &f32_battery_voltage_msg);
ros::Publisher pubChargeVoltage("charge_voltage", &f32_charge_voltage_msg);
ros::Publisher pubChargePWM("charge_pwm", &int16_charge_pwm_msg);

/*
 * SUBSCRIBERS
 */
extern "C" void CommandVelocityMessageCb(const geometry_msgs::Twist& msg);
ros::Subscriber<geometry_msgs::Twist> subCommandVelocity("cmd_vel", CommandVelocityMessageCb);

static nbt_t publish_nbt;
static nbt_t ros_nbt;


extern "C" void CommandVelocityMessageCb(const geometry_msgs::Twist& msg)
{
		//Fill subscriber
		debug_printf("x: %f  z: %f\r\n", msg.linear.x, msg.angular.z);
}

extern "C" void cdc_receive_put(uint8_t value)
{
	ringbuffer_putchar(&rb, value);
}

extern "C" void init_ROS()
{
	ringbuffer_init(&rb, RxBuffer, RxBufferSize);

	// Initialize ROS
	nh.initNode();

//	nh.advertise(chatter);
	nh.advertise(pubBatteryVoltage);
	nh.advertise(pubChargeVoltage);
	nh.advertise(pubChargePWM);
	nh.subscribe(subCommandVelocity);

	NBT_init(&publish_nbt, 1000);
	NBT_init(&ros_nbt, 10);	
}

extern "C" void chatter_handler()
{
	  if (NBT_handler(&publish_nbt))
	  {
		  /*
		  char version[] = "version: 0.1";
		  str_msg.data = version;
		  chatter.publish(&str_msg);
		  */
		  
		  f32_battery_voltage_msg.data = ADC_BatteryVoltage();
		  pubBatteryVoltage.publish(&f32_battery_voltage_msg);

		  f32_charge_voltage_msg.data = ADC_ChargeVoltage();
		  pubChargeVoltage.publish(&f32_charge_voltage_msg);

		  int16_charge_pwm_msg.data = chargecontrol_pwm_val;
		  pubChargePWM.publish(&int16_charge_pwm_msg);
	  }
}

extern "C" void spinOnce()
{
	  if (NBT_handler(&ros_nbt))
	  {
			nh.spinOnce();
	  }
}

