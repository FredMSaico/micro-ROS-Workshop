// --------------------------------------------------------------
// MicroROS Template for Arduino IDE
// Version: 1.0.0
// Created by: Alfredo Mamani Saico
// Email: amamanisai@unsa.edu.pe
// Date: July 30, 2023
// Last Modified: August 4, 2023
// Description: This template provides a basic setup for using MicroROS on ESP32 boards.
// Reference page: https://micro.ros.org/
// Repository: 
// Additional Contributors: 
//        - Flor Chacon (fchacong@unsa.edu.pe), 
//        - Marianela Choquepuma (mchoquepumac@unsa.edu.pe)
//        - Miguel Esquivel (mesquivelya@unsa.edu.pe)
// Requiriments: 
//        - Install MicroROS library from Arduino Library Manager.
//        - Setup ESP32 on Arduino IDE.
// Notes: This template is intended for educational purposes and may require additional 
//        configuration for specific hardware or ROS setups.
// --------------------------------------------------------------


#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

/*--------------Creación de entidades---------*/
rcl_publisher_t publisher;
rcl_subscription_t subscriber;

/*--------------Creación de mensajes---------*/
std_msgs__msg__Int32MultiArray pub_msg;
std_msgs__msg__Int32MultiArray sub_msg;

/*--------------Creación de objetos---------*/
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

/*--------------Configuracion de la red---------*/
char *ssid = "";
char *pwd = "";
char *host_ip = "";

/*--------------Función Callback---------*/
void subscription_callback(const void *raw_msg){
  std_msgs__msg__Int32MultiArray *msg = (std_msgs__msg__Int32MultiArray *)raw_msg;
  /*------------Lectura de mensajes----------*/   
}


void setup() {
  /*--------------Creación de objetos---------*/
  set_microros_wifi_transports(ssid, pwd, host_ip, 8888);

  /*--------------Inicialización del nodo---------*/
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "first_node", "", &support);
  
  /*--------------Creación de publicador---------*/
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "imu");

  /*--------------Creación de subscriptor---------*/
  rclc_subscription_init_default(
    &PWMsubscriber, 
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "pwm_topic" );

  /*--------------Creación de executor---------*/
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, 
                   &subscription_callback, ON_NEW_DATA);


}

void loop() {
  currentTime=millis();
  if ((currentTime - lastTime) >= 10){                        //Definir frecuencia de ejecución
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(200));
    
    /*--------Lectura de datos-------------*/

    rcl_publish(&publisher, &pub_msg, NULL);
  }
}
