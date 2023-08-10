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

#include <std_msgs/msg/int32_multi_array.h>
#include <geometry_msgs/msg/twist.h>

/*--------------Creación de entidades---------*/
rcl_publisher_t publisher;
rcl_subscription_t subscriber;

/*--------------Creación de mensajes---------*/
std_msgs__msg__Int32MultiArray pub_msg;
geometry_msgs__msg__Twist sub_msg;

/*--------------Creación de objetos---------*/
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

/*--------------Configuracion de la red---------*/
char *ssid = "Xiaomi_1A37";
char *pwd = "Ams200012";
char *host_ip = "192.168.31.52";
/*-----------------Variables------------*/
unsigned long lastTime = 0;
unsigned long currentTime = 0;

// Motor A
int motor1Pin1 = 5; 
int motor1Pin2 = 18; 
int enable1Pin = 15;

// Motor A
int motor2Pin1 = 19; 
int motor2Pin2 = 21; 
int enable2Pin = 2;

const int freq = 30000;
const int pwm1Channel = 0;
const int pwm2Channel = 0;
const int resolution = 8;

/*--------------Función Callback---------*/
void subscription_callback(const void *raw_msg) {
  geometry_msgs__msg__Twist *msg = (geometry_msgs__msg__Twist *)raw_msg;
  /*------------Lectura de mensajes----------*/
  float x = msg->linear.x;
  float z = msg->angular.z;   
  
  if (x > 0) { // Mover hacia adelante (todos los motores rotan en dirección adelante)
    Serial.print("Moving Forward: ");
    Serial.println(x);
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
    //ledcWrite(pwm1Channel, 100); 
  } else if (x < 0) { // Mover hacia atrás (todos los motores rotan en dirección atrás)
    Serial.print("Moving Backwards: ");
    Serial.println(x);
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW); 
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  }
  else {
    Serial.print("Stop: ");
    Serial.println(x);
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, LOW);    
    }

  if (z > 0) { // Mover a la izquierda
    Serial.print("Moving Left: ");
    Serial.println(z);
    digitalWrite(motor1Pin1, LOW);
    digitalWrite(motor1Pin2, HIGH);
    digitalWrite(motor2Pin1, HIGH);
    digitalWrite(motor2Pin2, LOW);
  } else if (z < 0) { // Mover a la derecha
    Serial.print("Moving Right: ");
    Serial.println(z);
    digitalWrite(motor1Pin1, HIGH);
    digitalWrite(motor1Pin2, LOW);
    digitalWrite(motor2Pin1, LOW);
    digitalWrite(motor2Pin2, HIGH);
  }
}

void setup() {
  /*--------------Creación de objetos---------*/
  set_microros_wifi_transports(ssid, pwd, host_ip, 8888);

  Serial.begin(115200);
  pinMode(motor1Pin1, OUTPUT);
  pinMode(motor1Pin2, OUTPUT);
  pinMode(enable1Pin, OUTPUT);
  pinMode(motor2Pin1, OUTPUT);
  pinMode(motor2Pin2, OUTPUT);
  pinMode(enable2Pin, OUTPUT);
  ledcSetup(pwm1Channel, freq, resolution);
  ledcAttachPin(enable1Pin, pwm1Channel);
  ledcSetup(pwm2Channel, freq, resolution);
  ledcAttachPin(enable1Pin, pwm2Channel);
  delay(10);
  digitalWrite(motor1Pin1, LOW);
  digitalWrite(motor1Pin2, LOW);
  digitalWrite(motor2Pin1, LOW);
  digitalWrite(motor2Pin2, LOW);
  delay(10);

  /*--------------Inicialización del nodo---------*/
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "first_node", "", &support);
  
  /*--------------Creación de publicador---------*/
  rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
    "pub_topic");

  /*--------------Creación de subscriptor---------*/
  rclc_subscription_init_default(
    &subscriber, 
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel" );

  /*--------------Creación de executor---------*/
  executor = rclc_executor_get_zero_initialized_executor();
  rclc_executor_init(&executor, &support.context, 2, &allocator);
  rclc_executor_add_subscription(&executor, &subscriber, &sub_msg, 
                   &subscription_callback, ON_NEW_DATA);
}

void loop() {
  currentTime=millis();
  if ((currentTime - lastTime) >= 10){                        //Definir frecuencia de ejecución
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(200));
    
    /*--------Lectura de datos-------------*/

    //rcl_publish(&publisher, &pub_msg, NULL);
  }
}
