/**
  ******************************************************************************
  * @file    cpp_main.c
  * @author  Georg Swoboda <cn@warp.at>
  * @date    21/09/2022
  * @version 1.0.0
  * @brief   ROS Node main C++ routines
  ******************************************************************************
  * Main ROS routines
  * Publish/Subscribe to Topics
  * Provide Services
  * Odometry (for DR)
  ******************************************************************************
  */


#include "board.h"

#include <cpp_main.h>
#include "main.h"
#include "panel.h"
#include "emergency.h"
#include "drivemotor.h"
#include "blademotor.h"
#include "spiflash.h"
#include "stm32f1xx_hal.h"
#include "ringbuffer.h"
#include "ros.h"
#include "ros/time.h"
#include "tf/tf.h"
#include "tf/transform_broadcaster.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Int16.h"
#include "std_msgs/UInt8.h"
#include "std_msgs/UInt16.h"
#include "std_msgs/UInt32.h"
#include "std_msgs/Int16MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "nbt.h"
#include "geometry_msgs/Twist.h"
#include "std_srvs/SetBool.h"
#include "std_srvs/Empty.h"

// IMU
#include "imu/imu.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/MagneticField.h"
#include "sensor_msgs/Temperature.h"
#include "mowgli/magnetometer.h"

// Flash Configuration Services
#include "mowgli/SetCfg.h"
#include "mowgli/GetCfg.h"
#include "mowgli/Led.h"

// Status message
#include "mowgli/status.h"


#define MAX_MPS	  	0.6		 	// Allow maximum speed of 0.6 m/s 
#define PWM_PER_MPS 300.0		// PWM value of 300 means 1 m/s bot speed

#define TICKS_PER_M 250.0		// Motor Encoder ticks per meter

//#define WHEEL_BASE  0.325		// The distance between the center of the wheels in meters
#define WHEEL_BASE  0.285		// The distance between the center of the wheels in meters
#define WHEEL_DIAMETER 0.198 	// The diameter of the wheels in meters

#define ODOM_NBT_TIME_MS   100 	// 200ms
#define IMU_NBT_TIME_MS    100  
#define MOTORS_NBT_TIME_MS 100
#define STATUS_NBT_TIME_MS 250

extern uint8_t RxBuffer[RxBufferSize];
struct ringbuffer rb;

ros::Time last_cmd_vel(0, 0);
ros::Time last_cmd_blade(0, 0);
uint32_t last_cmd_vel_age;		// age of last velocity command
uint32_t last_cmd_blade_age;		// age of last blade command

// drive motor control
static uint8_t left_speed=0;
static uint8_t right_speed=0;
static uint8_t left_dir=0;
static uint8_t right_dir=0;

// blade motor control
static uint8_t blade_on_off=0;

static uint8_t svcCfgDataBuffer[256];

ros::NodeHandle nh;

// odom message
geometry_msgs::Quaternion quat;
geometry_msgs::TransformStamped t;
tf::TransformBroadcaster broadcaster;
char base_link[] = "base_link_dr";
char odom[] = "odom_dr";

//double radius = 0.04;                              //Wheel radius, in m
//double wheelbase = 0.187;                          //Wheelbase, in m
double two_pi = 6.28319;
double speed_act_left = 0.0;
double speed_act_right = 0.0;

double distance_left = 0.0;
double distance_right = 0.0;
//double speed_req1 = 0.0;
//double speed_req2 = 0.0;
//double speed_dt = 0.0;
double x_pos = 0.0;
double y_pos = 0.0;
double theta = 0.0;
ros::Time current_time;
// ros::Time speed_time;
// ---
// double rate = 10.0;
double linear_scale_positive = 1.0;
double linear_scale_negative = 1.0;
double angular_scale_positive = 1.0;
double angular_scale_negative = 1.0;
bool publish_tf = true; // publish odom -> base_link transform
double dt = 0.0;
double dx = 0.0;
double dy = 0.0;
double dth = 0.0;
double dxy = 0.0;
double vx = 0.0;
double vy = 0.0;

int32_t left_encoder_ticks_old  = 0;
int32_t right_encoder_ticks_old = 0;

ros::Time odom_last_time = nh.now();	// this is 0 because we have no time upon startup
ros::Time odom_current_time;

float imu_onboard_temperature; // cached temp value, so we dont poll I2C constantly

// std_msgs::String str_msg;
//std_msgs::Float32 f32_battery_voltage_msg;
//std_msgs::Float32 f32_charge_voltage_msg;
//std_msgs::Float32 f32_charge_current_msg;
//std_msgs::Int16 int16_charge_pwm_msg;
//std_msgs::Bool bool_blade_state_msg;
//std_msgs::Bool bool_charging_state_msg;
std_msgs::Int16MultiArray buttonstate_msg;
nav_msgs::Odometry odom_msg;
//std_msgs::UInt32 left_encoder_ticks_msg;
//std_msgs::UInt32 right_encoder_ticks_msg;

// IMU
// external IMU (i2c)
sensor_msgs::Imu imu_msg;
sensor_msgs::MagneticField imu_mag_msg;
// onboard IMU (accelerometer and temp)
sensor_msgs::Imu imu_onboard_msg;
//sensor_msgs::Temperature imu_onboard_temp_msg;

//sensor_msgs::MagneticField imu_mag_calibration_msg;
mowgli::magnetometer imu_mag_calibration_msg;

// mowgli status message
mowgli::status status_msg;

/*
 * PUBLISHERS
 */
// ros::Publisher chatter("version", &str_msg);
// ros::Publisher pubBatteryVoltage("battery_voltage", &f32_battery_voltage_msg);
// ros::Publisher pubChargeVoltage("charge_voltage", &f32_charge_voltage_msg);
// ros::Publisher pubChargeCurrent("charge_current", &f32_charge_current_msg);
// ros::Publisher pubChargePWM("charge_pwm", &int16_charge_pwm_msg);
// ros::Publisher pubChargeingState("charging_state", &bool_charging_state_msg);
// ros::Publisher pubBladeState("blade_state", &bool_blade_state_msg);
ros::Publisher pubOdom("mowgli/odom", &odom_msg);
// ros::Publisher pubLeftEncoderTicks("left_encoder_ticks", &left_encoder_ticks_msg);
// ros::Publisher pubRightEncoderTicks("right_encoder_ticks", &right_encoder_ticks_msg);
ros::Publisher pubButtonState("buttonstate", &buttonstate_msg);
ros::Publisher pubStatus("mowgli/status", &status_msg);

// IMU onboard
ros::Publisher pubIMUOnboard("imu_onboard/data_raw", &imu_onboard_msg);
// ros::Publisher pubIMUOnboardTemp("imu_onboard/temp", &imu_onboard_temp_msg);

// IMU external
ros::Publisher pubIMU("imu/data_raw", &imu_msg);
ros::Publisher pubIMUMag("imu/mag", &imu_mag_msg);
ros::Publisher pubIMUMagCalibration("imu/mag_calibration", &imu_mag_calibration_msg);


/*
 * SUBSCRIBERS
 */
extern "C" void CommandVelocityMessageCb(const geometry_msgs::Twist& msg);
ros::Subscriber<geometry_msgs::Twist> subCommandVelocity("cmd_vel", CommandVelocityMessageCb);

// TODO ros::Subscriber<std_msgs::Bool> subLEDSet("cmd_panel_led_set", CommandLEDSetMessageCb);
// TODO ros::Subscriber<std_msgs::Bool> subLEDFlashSlow("cmd_panel_led_flash_slow", CommandLEDFlashSlowMessageCb);
// TODO ros::Subscriber<std_msgs::Bool> subLEDFlashFast("cmd_panel_led_flash_fast", CommandLEDFlashFastMessageCb);
// TODO ros::Subscriber<std_msgs::Bool> subLEDClear("cmd_panel_led_clear", CommandLEDClearMessageCb);

// SERVICES
void cbSetCfg(const mowgli::SetCfgRequest &req, mowgli::SetCfgResponse &res);
void cbGetCfg(const mowgli::GetCfgRequest &req, mowgli::GetCfgResponse &res);
void cbEnableMowerMotor(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
void cbReboot(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res);
void cbEnableTF(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
void cbSetLed(const mowgli::LedRequest &req, mowgli::LedResponse &res);
void cbClrLed(const mowgli::LedRequest &req, mowgli::LedResponse &res);

ros::ServiceServer<mowgli::SetCfgRequest, mowgli::SetCfgResponse> svcSetCfg("mowgli/SetCfg", cbSetCfg);
ros::ServiceServer<mowgli::GetCfgRequest, mowgli::GetCfgResponse> svcGetCfg("mowgli/GetCfg", cbGetCfg);
ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> svcEnableMowerMotor("mowgli/EnableMowerMotor", cbEnableMowerMotor);
ros::ServiceServer<std_srvs::Empty::Request, std_srvs::Empty::Response> svcReboot("mowgli/Reboot", cbReboot);
ros::ServiceServer<std_srvs::SetBool::Request, std_srvs::SetBool::Response> svcEnableTF("mowgli/EnableTF", cbEnableTF);
ros::ServiceServer<mowgli::LedRequest, mowgli::LedResponse> svcSetLed("mowgli/SetLed", cbSetLed);
ros::ServiceServer<mowgli::LedRequest, mowgli::LedResponse> svcClrLed("mowgli/ClrLed", cbClrLed);

/*
 * NON BLOCKING TIMERS
 */
static nbt_t ros_nbt;
static nbt_t publish_nbt;
static nbt_t motors_nbt;
static nbt_t panel_nbt;
static nbt_t odom_nbt;
static nbt_t imu_nbt;
static nbt_t status_nbt;

/*
 * reboot flag, if true we reboot after next publish_nbt
 */
static bool reboot_flag = false;

/*
 * receive and parse cmd_vel messages
 * actual driving (updating drivemotors) is done in the drivemotors_nbt
 */
extern "C" void CommandVelocityMessageCb(const geometry_msgs::Twist& msg)
{
		last_cmd_vel = nh.now();

		//	debug_printf("x: %f  z: %f\r\n", msg.linear.x, msg.angular.z);

		// calculate twist speeds to add/substract 
		float left_twist_mps = -1.0 * msg.angular.z * WHEEL_BASE * 0.5;
		float right_twist_mps =  msg.angular.z * WHEEL_BASE * 0.5;
    

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

	//	debug_printf("left_mps: %f (%c)  right_mps: %f (%c)\r\n", left_mps, left_dir?'F':'R', right_mps, right_dir?'F':'R');
}

extern "C" void cdc_receive_put(uint8_t value)
{
	ringbuffer_putchar(&rb, value);
}


/*
 * Update various chatters topics
 */
extern "C" void chatter_handler()
{
	  if (NBT_handler(&publish_nbt))
	  {
		  /*
		  char version[] = "version: 0.1";
		  str_msg.data = version;
		  chatter.publish(&str_msg);
		  */
		  /*
		  f32_battery_voltage_msg.data = battery_voltage;
		  pubBatteryVoltage.publish(&f32_battery_voltage_msg);

		  f32_charge_voltage_msg.data = charge_voltage;
		  pubChargeVoltage.publish(&f32_charge_voltage_msg);

		  f32_charge_current_msg.data = charge_current;
		  pubChargeCurrent.publish(&f32_charge_current_msg);

		  int16_charge_pwm_msg.data = chargecontrol_pwm_val;
		  pubChargePWM.publish(&int16_charge_pwm_msg);

		  bool_charging_state_msg.data =  chargecontrol_is_charging;
		  pubChargeingState.publish(&bool_charging_state_msg);
*/
 		  //bool_blade_state_msg.data = true; // TODO: read blade status
//		  pubBladeState.publish(&bool_blade_state_msg);

#ifdef IMU_ONBOARD_TEMP
		  imu_onboard_temperature = IMU_Onboard_ReadTemp();
#else
		  imu_onboard_temperature = -100;
#endif
/*
		  imu_onboard_temp_msg.variance = 0.5;		// 0.5°C resolution
		  imu_onboard_temp_msg.header.frame_id = base_link;
		  pubIMUOnboardTemp.publish(&imu_onboard_temp_msg);
*/
		  HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);         // flash LED

		  // reboot if set via cbReboot (mowgli/Reboot)
		  if (reboot_flag)
		  {
			nh.spinOnce();
			NVIC_SystemReset();
			// we never get here ...
		  }
	  }
}

/*
 *  Drive Motors handler
 *  Blade Motor handler
 */
extern "C" void motors_handler()
{
	  if (NBT_handler(&motors_nbt))
	  {
		if (Emergency_State())
		{			
			DRIVEMOTOR_SetSpeed(0,0,0,0);
			BLADEMOTOR_Set(0);
		}
		else {
			// if the last velocity cmd is older than 1sec we stop the drive motors
			last_cmd_vel_age = nh.now().sec - last_cmd_vel.sec;			
			if (last_cmd_vel_age > 1) {
				DRIVEMOTOR_SetSpeed(0, 0, left_dir, right_dir);
			}
			else {
				DRIVEMOTOR_SetSpeed(left_speed, right_speed, left_dir, right_dir);
			}

			// if the last blade cmd is older than 25sec we stop the motor
			last_cmd_blade_age = nh.now().sec - last_cmd_blade.sec;
			if (last_cmd_blade_age > 25) {
				blade_on_off = 0;				
			}			
			BLADEMOTOR_Set(blade_on_off);			
		}
	  }
}

/*
 *  Keyboard/LED Panel handler
 */
extern "C" void panel_handler()
{
	  if (NBT_handler(&panel_nbt))
	  {			  
		PANEL_Tick();
		if (buttonupdated == 1)
		{
			debug_printf("ROS: panel_nbt() - buttonstate changed\r\n");
			buttonstate_msg.data = (int16_t*) malloc(sizeof(int16_t) * PANEL_BUTTON_BYTES);
			buttonstate_msg.data_length = PANEL_BUTTON_BYTES;
			memcpy(buttonstate_msg.data,buttonstate,sizeof(int16_t) * PANEL_BUTTON_BYTES);
			pubButtonState.publish(&buttonstate_msg);		
			free(buttonstate_msg.data);
			buttonupdated=0;
		}
	  }
}

extern "C" void broadcast_handler()
{
	  if (NBT_handler(&imu_nbt))
	  {
		////////////////////////////////////////
		// IMU Messages
		////////////////////////////////////////		
		imu_msg.header.frame_id = "imu";
		
		// No Orientation in IMU message
		imu_msg.orientation.x = 0;
		imu_msg.orientation.y = 0;
		imu_msg.orientation.z = 0;
		imu_msg.orientation.w = 0;
		imu_msg.orientation_covariance[0] = -1;

		/**********************************/
		/* Exernal Accelerometer 		  */
		/**********************************/
#ifdef IMU_ACCELERATION
		// Linear acceleration		
		IMU_ReadAccelerometer(&imu_msg.linear_acceleration.x, &imu_msg.linear_acceleration.y, &imu_msg.linear_acceleration.z);		
		IMU_AccelerometerSetCovariance(imu_msg.linear_acceleration_covariance);	
#else
		imu_msg.linear_acceleration.x = imu_msg.linear_acceleration.y = imu_msg.linear_acceleration.z = 0;		
		imu_msg.linear_acceleration_covariance[0] = -1;
#endif

		/**********************************/
		/* Exernal Gyro					  */
		/**********************************/
#ifdef IMU_ANGULAR
		// Angular velocity
		IMU_ReadGyro(&imu_msg.angular_velocity.x, &imu_msg.angular_velocity.y, &imu_msg.angular_velocity.z);
		IMU_GyroSetCovariance(imu_msg.angular_velocity_covariance);	
#else
		imu_msg.angular_velocity.x = imu_msg.angular_velocity.y = imu_msg.angular_velocity.z = 0;		
		imu_msg.angular_velocity_covariance[0] = -1;
#endif		
		imu_msg.header.stamp = nh.now();
		pubIMU.publish(&imu_msg);

		/**********************************/
		/* Exernal Magnetometer Corrected */
		/**********************************/
		double x,y,z;	

		// Orientation (Magnetometer)
		imu_mag_msg.header.frame_id = "imu";								
	 	IMU_ReadMagnetometer(&x, &y, &z);
		imu_mag_msg.magnetic_field.x = x;
		imu_mag_msg.magnetic_field.y = y;
		imu_mag_msg.magnetic_field.z = z;

		// covariance is fixed for now
		imu_mag_msg.magnetic_field_covariance[0] = 1e-3;
		imu_mag_msg.magnetic_field_covariance[4] = 1e-3;
		imu_mag_msg.magnetic_field_covariance[8] = 1e-3;
		imu_mag_msg.header.stamp = nh.now();
		pubIMUMag.publish(&imu_mag_msg);

		/******************************************/
		/* Exernal Magnetometer RAW (Calibration) */
		/******************************************/
		IMU_ReadMagnetometerRaw(&x, &y, &z);
		imu_mag_calibration_msg.x = x;
		imu_mag_calibration_msg.y = y;
		imu_mag_calibration_msg.z = z;

		imu_mag_msg.header.stamp = nh.now();
		pubIMUMagCalibration.publish(&imu_mag_calibration_msg);

		/**********************************/
		/* Onboard (GForce) Accelerometer */
		/**********************************/
#ifdef IMU_ONBOARD_ACCELERATION
		IMU_Onboard_ReadAccelerometer(&imu_onboard_msg.linear_acceleration.x, &imu_onboard_msg.linear_acceleration.y, &imu_onboard_msg.linear_acceleration.z);		
		IMU_Onboard_AccelerometerSetCovariance(imu_onboard_msg.linear_acceleration_covariance);	
#else
		imu_onboard_msg.linear_acceleration.x = imu_onboard_msg.linear_acceleration.y = imu_onboard_msg.linear_acceleration.z = 0;		
#endif
		// no onboard gyro so angular velocities are always zero
		imu_onboard_msg.angular_velocity.x = imu_onboard_msg.angular_velocity.y = imu_onboard_msg.angular_velocity.z = 0;		
		imu_onboard_msg.angular_velocity_covariance[0] = -1;		// indicate *not valid* to EKF
		imu_onboard_msg.header.stamp = nh.now();
		pubIMUOnboard.publish(&imu_onboard_msg);		
	  } // if (NBT_handler(&imu_nbt))

  	  if (NBT_handler(&status_nbt))
	  {
		////////////////////////////////////////
		// mowgli/status Message
		////////////////////////////////////////		
		status_msg.stamp = nh.now();
		status_msg.rain_detected = RAIN_Sense();
		status_msg.emergency_tilt_mech_triggered = Emergency_Tilt();
		status_msg.emergency_tilt_accel_triggered = Emergency_LowZAccelerometer();
		status_msg.emergency_left_wheel_lifted = Emergency_WheelLiftBlue();
		status_msg.emergency_right_wheel_lifted = Emergency_WheelLiftRed();
		status_msg.emergency_stopbutton_triggered = Emergency_StopButtonYellow() || Emergency_StopButtonWhite();
		status_msg.left_encoder_ticks = left_encoder_ticks;
		status_msg.right_encoder_ticks = right_encoder_ticks;
		status_msg.v_charge = charge_voltage;
		status_msg.i_charge = charge_current;
		status_msg.v_battery = battery_voltage;
		status_msg.charge_pwm = chargecontrol_pwm_val;
		status_msg.is_charging = chargecontrol_is_charging;
		status_msg.imu_temp = imu_onboard_temperature;
		status_msg.blade_motor_ctrl_enabled = true;	// hardcoded for now
		status_msg.drive_motor_ctrl_enabled = true; // hardcoded for now
		status_msg.blade_motor_enabled = BLADEMOTOR_bActivated;	// set by feedback from blademotor	
		status_msg.left_power = left_power;	
		status_msg.right_power = right_power; 
		status_msg.sw_ver_maj = MOWGLI_SW_VERSION_MAJOR;
		status_msg.sw_ver_bra = MOWGLI_SW_VERSION_BRANCH;
		status_msg.sw_ver_min = MOWGLI_SW_VERSION_MINOR;
		pubStatus.publish(&status_msg);		
	  } // if (NBT_handler(&status_nbt))

	  if (NBT_handler(&odom_nbt))
	  {	
		/* first odom_nbt call sets our "odom_last_time" */
		if (odom_last_time.nsec == 0 )						
		{
			odom_last_time = nh.now();		
		}
		else		
		{	/* subsequent odom_nbts publish odom messages */		

			/* dynamic dt calculation */
			odom_current_time = nh.now();	
			uint32_t odom_current_time_msec = odom_current_time.sec*1000 + odom_current_time.nsec/1000000.0;
			uint32_t odom_last_time_msec = odom_last_time.sec*1000 + odom_last_time.nsec/1000000.0;			
			dt = (odom_current_time_msec - odom_last_time_msec) / 1000.0;


			speed_act_left = left_wheel_speed_val/PWM_PER_MPS;			// wheel speed in m/s
			speed_act_right = right_wheel_speed_val/PWM_PER_MPS;		// wheel speed in m/s			

			/* calculate distances from accumulating wheel encoder ticks */
			if (left_encoder_ticks>=left_encoder_ticks_old)
				distance_left = (left_encoder_ticks-left_encoder_ticks_old)/TICKS_PER_M;
			else 
				distance_left = -1.0 * (left_encoder_ticks_old-left_encoder_ticks)/TICKS_PER_M;
			
			if (right_encoder_ticks>right_encoder_ticks_old)			
				distance_right = (right_encoder_ticks-right_encoder_ticks_old)/TICKS_PER_M;
			else
				distance_right = -1.0 * (right_encoder_ticks_old-right_encoder_ticks)/TICKS_PER_M;

			// only continue with calculating an odom message if we are moving, 
			// or 0.1 sec are elapsed since the last odom message was published
			//if (distance_left != 0 || distance_right != 0 || dt > 0.05)
			if (1)
			{
				odom_last_time = nh.now();

				// debug_printf("left_encoder_ticks: (%d/%d) right_encoder_ticks: (%d/%d) distance_left: %f distance_right: %f\r\n", left_encoder_ticks, left_encoder_ticks_old, right_encoder_ticks, right_encoder_ticks_old, distance_left, distance_right);				
				// dt = (ODOM_NBT_TIME_MS/1000.0);			
				// 	debug_printf("left_encoder_val: %d right_encoder_val: %d dt: %f speed_act_left: %f speed_act_right: %f\r\n",left_encoder_val, right_encoder_val, dt, speed_act_left, speed_act_right);			
			
				dx = (distance_left+distance_right)/2.0;
				dy = 0;

				//	dxy = (speed_act_left+speed_act_right)*dt/2.0;
				//	dth = - 1.0 * ((speed_act_right-speed_act_left)*dt)/WHEEL_BASE;

				dth = - 1.0 * (distance_left-distance_right)/WHEEL_BASE;
				
				if (dth > 0) dth *= angular_scale_positive;
				if (dth < 0) dth *= angular_scale_negative;

				//if (dxy > 0) dxy *= linear_scale_positive;
				//if (dxy < 0) dxy *= linear_scale_negative;
				//	dx = cos(dth) * dxy;
				//  dy = sin(dth) * dxy;

				x_pos += (cos(theta) * dx - sin(theta) * dy);
				y_pos += (sin(theta) * dx + cos(theta) * dy);

				//	debug_printf("dx: %f x_pos: %f dth: %f\r\n", dx, x_pos, dth);		

				theta += dth;
				if(theta >= two_pi) theta -= two_pi;
				if(theta <= -two_pi) theta += two_pi;

				left_encoder_ticks_old = left_encoder_ticks;
				right_encoder_ticks_old = right_encoder_ticks;

				quat = tf::createQuaternionFromYaw(theta);
				current_time = nh.now(); 

				//////////////////////////////////////////////////
				// odom transform (optional)
				//////////////////////////////////////////////////
				if(publish_tf) {
					geometry_msgs::TransformStamped t;						
					t.header.frame_id = odom;
					t.child_frame_id = base_link;
					t.transform.translation.x = x_pos;
					t.transform.translation.y = y_pos;
					t.transform.translation.z = 0.0;
					t.transform.rotation = quat;
					t.header.stamp = current_time;					
					broadcaster.sendTransform(t);			
				}
				//////////////////////////////////////////////////
				// odom message
				//////////////////////////////////////////////////
				odom_msg.header.stamp = current_time; 	
				odom_msg.header.frame_id = odom;
				odom_msg.pose.pose.position.x = x_pos;
				odom_msg.pose.pose.position.y = y_pos;
				odom_msg.pose.pose.position.z = 0.0;
				odom_msg.pose.pose.orientation = quat;
				if (speed_act_left == 0 && speed_act_right == 0)
				{
					odom_msg.pose.covariance[0] = 1e-9;
					odom_msg.pose.covariance[7] = 1e-3;
					odom_msg.pose.covariance[8] = 1e-9;
					odom_msg.pose.covariance[14] = 1e6;
					odom_msg.pose.covariance[21] = 1e6;
					odom_msg.pose.covariance[28] = 1e6;
					odom_msg.pose.covariance[35] = 1e-9;

					odom_msg.twist.covariance[0] = 1e-9;
					odom_msg.twist.covariance[7] = 1e-3;
					odom_msg.twist.covariance[8] = 1e-9;
					odom_msg.twist.covariance[14] = 1e6;
					odom_msg.twist.covariance[21] = 1e6;
					odom_msg.twist.covariance[28] = 1e6;
					odom_msg.twist.covariance[35] = 1e-9;
				}
				else{
					odom_msg.pose.covariance[0] = 1e-1;
					odom_msg.pose.covariance[7] = 1e-1;
					odom_msg.pose.covariance[8] = 0.0;
					odom_msg.pose.covariance[14] = 1e6;
					odom_msg.pose.covariance[21] = 1e6;
					odom_msg.pose.covariance[28] = 1e6;
					odom_msg.pose.covariance[35] = 1e3;
					
					odom_msg.twist.covariance[0] = 1e-3;
					odom_msg.twist.covariance[7] = 1e-3;
					odom_msg.twist.covariance[8] = 0.0;
					odom_msg.twist.covariance[14] = 1e6;
					odom_msg.twist.covariance[21] = 1e6;
					odom_msg.twist.covariance[28] = 1e6;
					odom_msg.twist.covariance[35] = 1e3;
				}

				vx = (dt == 0)?  0 : (speed_act_left+speed_act_right)/2.0;
			//	vth = (dt == 0)? 0 : (speed_act_right-speed_act_left)/WHEEL_BASE;
				odom_msg.child_frame_id = base_link;
				odom_msg.twist.twist.linear.x = vx;
				odom_msg.twist.twist.linear.y = 0.0;
				odom_msg.twist.twist.angular.z = dth;
				pubOdom.publish(&odom_msg);

			
		   } // movement detected or timeout
		} // first odom_last_time > 0
	  } // if (NBT_handler(&broadcast_nbt))
}

/*
 *  callback for mowgli/GetCfg Service
 */
void cbGetCfg(const mowgli::GetCfgRequest &req, mowgli::GetCfgResponse &res) 
{	
    debug_printf("cbGetCfg:\r\n");	
	debug_printf(" name: %s\r\n", req.name);

	res.data_length = SPIFLASH_ReadCfgValue(req.name, &res.type, svcCfgDataBuffer);

	if (res.data_length > 0)
	{		
		res.data = (uint8_t*)&svcCfgDataBuffer;	
		res.status = 1;
	}
	else
	{
		res.status = 0;
	}
}

/// @brief Set Led and optionally reset all other Leds (0x40) + Chirp (0x80)
/// @param req req.led the LED number and any option flags
/// @param res 
void cbSetLed(const mowgli::LedRequest &req, mowgli::LedResponse &res)
{	
 //  debug_printf("cbSetLed:\r\n");
 //  debug_printf(" led: %d\r\n", req.led);
   uint8_t v=req.led;
   if ( (req.led & 0x40) == 0x40)	// clear all Leds
   {
		for (uint8_t i=0;i<LED_STATE_SIZE;i++)
		{
			PANEL_Set_LED(i, PANEL_LED_OFF);
		}
   }  
   if ( (req.led & 0x80) == 0x80)
   {
     do_chirp = 1;
   }

   // remove flag bits, turn on led
   v &= ~(1UL<<7);
   v &= ~(1UL<<6);
   PANEL_Set_LED(v, PANEL_LED_ON);
}

/// @brief Clear Led and optionally reset all other Leds (0x40) + Chirp (0x80)
/// @param req req.led the LED number and any option flags
/// @param res 
void cbClrLed(const mowgli::LedRequest &req, mowgli::LedResponse &res)
{	
 //  debug_printf("cbClrLed:\r\n");
 //  debug_printf(" led: %d\r\n", req.led);
   uint8_t v=req.led;
   if ( (req.led & 0x40) == 0x40)	// clear all Leds
   {
		for (uint8_t i=0;i<LED_STATE_SIZE;i++)
		{
			PANEL_Set_LED(i, PANEL_LED_OFF);
		}
   }  
   if ( (req.led & 0x80) == 0x80)
   {
     do_chirp = 1;
   }
 
   // remove flag bits, turn of led
   v &= ~(1UL<<7);
   v &= ~(1UL<<6);
   PANEL_Set_LED(v, PANEL_LED_OFF);
}


/*
 *  callback for mowgli/SetCfg Service
 */
void cbSetCfg(const mowgli::SetCfgRequest &req, mowgli::SetCfgResponse &res) {
	union {
		float f;
		uint8_t b[4];
	} float_val;

	union {
		double d;
		uint8_t b[8];
	} double_val;
	uint8_t i;

    debug_printf("cbSetCfg:\r\n");
	debug_printf(" type: %d\r\n", req.type);
	debug_printf(" len: %d\r\n", req.data_length);
	debug_printf(" name: %s\r\n", req.name);

	if (req.type == 0) // TYPE_INT32 (0)
	{
		int32_t int32_val = (req.data[0]) + (req.data[1]<<8) + (req.data[2]<<16) + (req.data[3]<<24);
		debug_printf("(int32) data: %d\r\n", int32_val);			
	}
	if (req.type == 1) // TYPE_UINT32 (1)
	{
		uint32_t uint32_val = (req.data[0]) + (req.data[1]<<8) + (req.data[2]<<16) + (req.data[3]<<24);
		debug_printf("(uint32) data: %d\r\n", uint32_val);			
	}
	if (req.type == 2) // TYPE_FLOAT (2)
	{		
		for (i=0;i<4;i++)
		{
			float_val.b[i] = req.data[i];
		}		
		debug_printf("(float) data: %f\r\n", float_val.f);					
	}
	if (req.type == 3)  // TYPE_DOUBLE (3)
	{		
		for (i=0;i<8;i++)
		{
			double_val.b[i] = req.data[i];
		}
		debug_printf("(double) data: %Lf\r\n", double_val.d);			
	}
	if (req.type == 4)	// TYPE_STRING (4)
	{	
		debug_printf("(string) data: '");				
		for (i=0;i<req.data_length;i++)
		{
			debug_printf("%c", req.data[i]);
		}
		debug_printf("'\r\n");	
	}
	if (req.type == 5)	// TYPE_BARRAY (5)
	{	
		debug_printf("(byte array) data: '");				
		for (i=0;i<req.data_length;i++)
		{
			debug_printf("0x%02x ", req.data[i]);
		}
		debug_printf("'\r\n");	
	}

	// debug print data[] array
	for (i=0;i<req.data_length;i++)
	{
		debug_printf(" data[%d]: %d\r\n", i, req.data[i]);	
	}		

	SPIFLASH_WriteCfgValue(req.name, req.type, req.data_length, req.data);
	res.status = 1;
}

/*
 *  callback for mowgli/EnableTF Service
 */
void cbEnableTF(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{
	publish_tf = req.data;
    if (req.data) {        		
        res.success = true;
        res.message = "cbEnableTF: publishing transform activated";
    }
    else {
        res.success = false;
        res.message = "cbEnableTF: publishing transform de-activated";
    }    
}

/*
 *  callback for mowgli/EnableMowerMotor Service
 */
void cbEnableMowerMotor(const std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
{	
	last_cmd_blade = nh.now();	// if the last blade cmd is older than 25sec the motor will be stopped !
	debug_printf("ROS: cbEnableMowerMotor(from %d to %d)", blade_on_off, req.data);	
	blade_on_off = req.data;	
    if (req.data) {        		
        res.success = true;
        res.message = "";
    }
    else {
        res.success = true;
        res.message = "";
    }    
	debug_printf("[DONE]\r\n");
}

/*
 *  callback for mowgli/Reboot Service
 */
void cbReboot(const std_srvs::Empty::Request &req, std_srvs::Empty::Response &res)
{
	//debug_printf("cbReboot:\r\n");
	reboot_flag = true;	
}

/*
 * ROS housekeeping
 */
extern "C" void spinOnce()
{
	  if (NBT_handler(&ros_nbt))
	  {
			nh.spinOnce();
	  }
}

/* 
 *  Initialize rosserial
 */
extern "C" void init_ROS()
{
	ringbuffer_init(&rb, RxBuffer, RxBufferSize);

	// Initialize ROS
	nh.initNode();

	// Initialize TF Broadcaster
	broadcaster.init(nh);

	// Initialize Pubs
	//nh.advertise(pubBatteryVoltage);
	//nh.advertise(pubChargeVoltage);
	//nh.advertise(pubChargeCurrent);
	//nh.advertise(pubChargePWM);
	nh.advertise(pubOdom);

	//nh.advertise(pubBladeState);
	//nh.advertise(pubChargeingState);
	//nh.advertise(pubLeftEncoderTicks);
	//nh.advertise(pubRightEncoderTicks);
	nh.advertise(pubButtonState);
	nh.advertise(pubIMU);
	nh.advertise(pubIMUMag);
	nh.advertise(pubIMUMagCalibration);
	nh.advertise(pubIMUOnboard);
	//nh.advertise(pubIMUOnboardTemp);
	nh.advertise(pubStatus);
	
	// Initialize Subscribers
	nh.subscribe(subCommandVelocity);

	// Initialize Services	
	nh.advertiseService(svcSetCfg);	  
	nh.advertiseService(svcGetCfg);	  
    nh.advertiseService(svcEnableMowerMotor);
	nh.advertiseService(svcReboot);
	nh.advertiseService(svcEnableTF);
    nh.advertiseService(svcSetLed);
	nh.advertiseService(svcClrLed);
	
	// Initialize Timers
	NBT_init(&publish_nbt, 1000);
	NBT_init(&panel_nbt, 100);	
	NBT_init(&status_nbt, STATUS_NBT_TIME_MS);
	NBT_init(&imu_nbt, IMU_NBT_TIME_MS);
	NBT_init(&motors_nbt, MOTORS_NBT_TIME_MS);
	NBT_init(&odom_nbt, ODOM_NBT_TIME_MS);
	NBT_init(&ros_nbt, 10);	
}