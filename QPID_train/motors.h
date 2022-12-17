/*
 * @Author: Chenjie Wang
 * @LastEditTime: 2022-12-15 15:45:50
 * @Description: This file trains Q-learning agent. 
 * 
 * Copyright (c) 2022 Karry Zhang, Chenjie Wang All Rights Reserved. 
 */
#ifndef _MOTORS_H
#define _MOTORS_H
 
#define L_PWM_PIN 10
#define L_DIR_PIN 16
#define R_PWM_PIN 9
#define R_DIR_PIN 15

#define FWD LOW
#define REV HIGH

// Class to operate the motor(s).
class Motors_c {
  public:

    // Constructor, must exist.
    Motors_c() {
    } 

    // Use this function to initialise the pins and state of your motor(s).
    void initialise() {
      pinMode(L_PWM_PIN,OUTPUT);
      pinMode(L_DIR_PIN,OUTPUT);
      pinMode(R_PWM_PIN,OUTPUT);
      pinMode(R_DIR_PIN,OUTPUT);
      
      digitalWrite( L_DIR_PIN, HIGH );
      digitalWrite( R_DIR_PIN, HIGH );

      analogWrite( L_PWM_PIN, LOW );
      analogWrite( R_PWM_PIN, LOW );
    }

    // Write a function to operat your motor(s)
    void setMotorPower( float left_pwm, float right_pwm ) {
      //limit the range of power
      if (abs(left_pwm)>255 || abs(right_pwm)>255) {
        analogWrite( L_PWM_PIN, 255); 
        analogWrite( R_PWM_PIN, 0); 
      }
      //move forward,positive
      if (left_pwm >= 0) {
        digitalWrite(L_DIR_PIN, LOW);
        analogWrite( L_PWM_PIN, left_pwm);    
      }
      if (right_pwm >= 0) {
        digitalWrite(R_DIR_PIN, LOW);
        analogWrite( R_PWM_PIN, right_pwm);
      }
      //move back,nagivate
      if (left_pwm < 0) {
        digitalWrite(L_DIR_PIN, HIGH);
        analogWrite( L_PWM_PIN, -left_pwm);
      }
      if (right_pwm < 0) {
        digitalWrite(R_DIR_PIN, HIGH);
        analogWrite( R_PWM_PIN, -right_pwm);
      }
    }
};



#endif
