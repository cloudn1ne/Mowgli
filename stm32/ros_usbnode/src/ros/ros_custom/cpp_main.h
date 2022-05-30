/*
 * cpp_main.h
 * 
 */
#ifndef CPP_MAIN_H_
#define CPP_MAIN_H_

#ifdef __cplusplus
extern "C" {
#endif


void init_ROS();
void spinOnce();
void chatter_handler();
void motors_handler();
void panel_handler();
void broadcast_handler();

#ifdef __cplusplus
}
#endif

#endif /* CPP_MAIN_H_ */
