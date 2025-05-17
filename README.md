# Simple_car_control_mege_2560
*This course project(SDM273) is a three-wheeled vehicle control system implemented based on the Arduino Mega 2560 development board, designed to achieve line following, stopping, and distance measurement functions.

* This project combines sensor technology, motor control algorithms, and distance measurement modules to enable the vehicle to autonomously identify paths, achieve precise stopping, and measure the distance to obstacles. It is a core practical component of the SDM273 course, covering key technologies such as embedded system development, sensor integration, and automatic control.  

## Main Features  
### Line-Following Function:  
The vehicle uses a line-following module to identify black paths and adjusts motor speed using a PID control algorithm to ensure it follows the predetermined trajectory.  

### Stopping Function:  
When a specific marker (black strip) is detected, the vehicle can achieve precise stopping, ensuring system stability.  

### Distance Measurement Function:  
Equipped with an ultrasonic sensor or infrared distance measurement module, the vehicle measures the distance to obstacles in real time and outputs the distance information via a Bluetooth module to a mobile app.  

## Hardware:  
Arduino Mega 2560 development board, DC motors, line-following sensors, ultrasonic sensors, TOF distance sensors, Hall encoders.  

## Software:  
Arduino IDE.
