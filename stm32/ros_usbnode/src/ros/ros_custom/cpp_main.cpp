/*
 * cpp_main.c
 *
 */

#include <cpp_main.h>
#include "main.h"
#include "stm32f1xx_hal.h"
#include "ringbuffer.h"
#include "ros.h"
#include "ros/time.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "nbt.h"
#include "geometry_msgs/Twist.h"

#include "board.h"

#define MAX_MPS	  	0.6		 	// Allow maximum speed of 0.6 m/s 
#define PWM_PER_MPS 	300		// PWM value of 300 means 1 m/s bot speed

#define WHEEL_BASE  0.325		// The distance between the center of the wheels in meters
#define WHEEL_DIAMETER 0.198 	// The diameter of the wheels in meters

extern uint8_t RxBuffer[RxBufferSize];
struct ringbuffer rb;

// drive motor control
static uint8_t left_speed=0;
static uint8_t right_speed=0;
static uint8_t left_dir=0;
static uint8_t right_dir=0;

ros::NodeHandle nh;

// TF
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
char base_link[] = "/base_link";
char odom[] = "/odom";

double x = 1.0;
double y = 0.0;
double theta = 1.57;


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

/*
 * NON BLOCKING TIMERS
 */
static nbt_t ros_nbt;
static nbt_t publish_nbt;
static nbt_t drivemotors_nbt;
static nbt_t broadcast_nbt;


extern "C" void CommandVelocityMessageCb(const geometry_msgs::Twist& msg)
{
		//Fill subscriber
		debug_printf("x: %f  z: %f\r\n", msg.linear.x, msg.angular.z);

		// calculate twist speeds to add/substract 
		float left_twist_mps = -1.0 * msg.angular.z * WHEEL_BASE / WHEEL_DIAMETER / 10;
		float right_twist_mps = msg.angular.z * WHEEL_BASE / WHEEL_DIAMETER / 10;
    

		// add them to the linear speed 
		float left_mps = msg.linear.x + left_twist_mps;
        float right_mps = msg.linear.x + right_twist_mps;

		// cap left motor speed to MAX_MPS
		if (left_mps > MAX_MPS)
		{
			left_mps = MAX_MPS;
		}
		else if (left_mps < -1*MAX_MPS)
		{
			left_mps = -1*MAX_MPS;
		}
		// cap right motor speed to MAX_MPS
		if (right_mps > MAX_MPS)
		{
			right_mps = MAX_MPS;
		}
		else if (right_mps < -1*MAX_MPS)
		{
			right_mps = -1*MAX_MPS;
		}

		// set directions		
		left_dir = (left_mps >= 0)?1:0;
		right_dir = (right_mps >= 0)?1:0;

		// set drivemotors PWM values
		left_speed = abs(left_mps * PWM_PER_MPS);
		right_speed = abs(right_mps * PWM_PER_MPS);		

		debug_printf("left_mps: %f (%c)  right_mps: %f (%c)\r\n", left_mps, left_dir?'F':'R', right_mps, right_dir?'F':'R');
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

	// Initialize TF Broadcaster
	broadcaster.init(nh);

	// Initialize Pubs/Subs
//	nh.advertise(chatter);
	nh.advertise(pubBatteryVoltage);
	nh.advertise(pubChargeVoltage);
	nh.advertise(pubChargePWM);
	nh.subscribe(subCommandVelocity);

	// Initialize Timers
	NBT_init(&publish_nbt, 1000);
	NBT_init(&drivemotors_nbt, 100);
	NBT_init(&broadcast_nbt, 100);
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

		
		  HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);         // flash LED
	  }
}

extern "C" void drivemotors_handler()
{
	  if (NBT_handler(&drivemotors_nbt))
	  {	
		setDriveMotors(left_speed, right_speed, left_dir, right_dir);
	  }
}

extern "C" void broadcast_handler()
{
	  if (NBT_handler(&broadcast_nbt))
	  {
		double dx = 0.2;
		double dtheta = 0.18;

		x += cos(theta)*dx*0.1;
		y += sin(theta)*dx*0.1;
		theta += dtheta*0.1;
		if (theta > 3.14)
		  	theta = -3.14;
		t.header.frame_id = odom;
		t.child_frame_id = base_link;
		t.transform.translation.x = x;
		t.transform.translation.y = y;
		t.transform.rotation = tf::createQuaternionFromYaw(theta);
		t.header.stamp = nh.now();
		
		broadcaster.sendTransform(t);		
	  }
}

extern "C" void spinOnce()
{
	  if (NBT_handler(&ros_nbt))
	  {
			nh.spinOnce();
	  }
}

