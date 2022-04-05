/**
 * @file micro_ros_cfg.cpp
 * @author Maciej Kurcius
 * @brief 
 * @version 0.1
 * @date 2022-04-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#include <micro_ros_cfg.h>

//ROS PUBLISHERS
rcl_publisher_t publisher;
rcl_publisher_t imu_publisher;
rcl_publisher_t motor_state_publisher;
//ROS SUBSCRIPTIONS
rcl_subscription_t subscriber;
rcl_subscription_t motors_cmd_subscriber;
//ROS MESSAGES
uint8_t ros_msgs_cnt = 0;
std_msgs__msg__String msgs;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__JointState motors_cmd_msg;
sensor_msgs__msg__JointState motors_response_msg;
//ROS
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

bool create_entities(void){
    allocator = rcl_get_default_allocator();
    /* MICRO ROS INIT */
    // create init_options
    RCCHECK(rclc_support_init(&support, 0, NULL, &allocator))
    // create node
    RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));
    // init timer
    RCCHECK(rclc_timer_init_default(&timer, &support, RCL_MS_TO_NS(10),
                                    timer_callback));
    ros_msgs_cnt++;
    if(BOARD_MODE_DEBUG) Serial.printf("Created timer\r\n");
    // Init subscribers
    RCCHECK(rclc_subscription_init_default(
        &motors_cmd_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "motors_cmd"));
    ros_msgs_cnt++;
    if(BOARD_MODE_DEBUG) Serial.printf("Created 'motors_cmd' subscriber\r\n");

    RCCHECK(rclc_publisher_init_default(
        &imu_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu/data_raw"));
    ros_msgs_cnt++;
    if(BOARD_MODE_DEBUG) Serial.printf("Created 'sensor_msgs/Imu' publisher.\r\n");

    RCCHECK(rclc_publisher_init_best_effort(
        &motor_state_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
        "motors_response"));
    ros_msgs_cnt++;
    if(BOARD_MODE_DEBUG) Serial.printf("Created 'motors_response' publisher.\r\n");

    // create executor
    RCCHECK(rclc_executor_init(&executor, &support.context, ros_msgs_cnt, &allocator));
    RCCHECK(rclc_executor_add_timer(&executor, &timer));
    RCCHECK(rclc_executor_add_subscription(&executor, &motors_cmd_subscriber, &motors_cmd_msg,
                                        &motors_cmd_callback, ON_NEW_DATA));                                 
    if(BOARD_MODE_DEBUG) Serial.printf("Executor started\r\n");
    RCCHECK(rmw_uros_sync_session(1000));
    if(BOARD_MODE_DEBUG) Serial.printf("Clocks synchronised\r\n");
    return false;
}

bool destroy_entities(void){
    rcl_publisher_fini(&publisher, &node);
	rcl_node_fini(&node);
	rcl_timer_fini(&timer);
	rclc_executor_fini(&executor);
	rclc_support_fini(&support);
    return false;
}

void executor_loop_handler(void){
    RCSOFTCHECK(rclc_executor_spin(&executor));
}