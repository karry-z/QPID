/*
 * @Author: Karry Zhang
 * @LastEditTime: 2022-12-15 16:57:10
 * @Description: This file trains Q-learning agent. 
 * 
 * Copyright (c) 2022 Karry Zhang, Chenjie Wang All Rights Reserved. 
 */


#include <EEPROM.h>
#include <ArduinoJson.h>

#include "motors.h"
#include "encoders.h"
#include "pid.h"
#include "qlearning.h"

#define lambda 1 // weight of reward for overshoot and settling time

StaticJsonDocument<200> doc;
double L_PWM = 0; // left motor pwm
double R_PWM = 0; // right motor pwm, not change in this experiment
double cpr = 358.3;  // counts per revolution of the encoder
volatile double speed;  // speed of the left motor
volatile long last_count; // count of the encoder in last interrupt
volatile long delta_count; // the difference count of the encoder between two interrupts
double speed_ref = 20; // target speed
long greedy = 10;   // greedy of ε-greedy policy, starts from 10 and increase to 90.
float max_speed = speed_ref;
unsigned long episode_start_time;
unsigned long episode_timeout = 10 * 1000; // 10 sec

PID_c l_pid(&speed, &L_PWM, &speed_ref, 0.35, 0.001, 0); // left pid controller
Motors_c motors;
QLearning_c rl;
unsigned long start_time;

void takeStep(double kp, double ki, bool *done, float *r);


void setup() {
    Serial.begin(9600);
    randomSeed(10);
    motors.initialise();
    delay(1); // wait for 30s to have enough time to prepare for the experiment.
    timerSetUP();
    setupEncoder0();
    setupEncoder1();
    motors.setMotorPower(0,0);
    rl.getQTableFromEEPROM(); // load the Q-table from EEPROM
    start_time = millis();
}

void loop() { // one timestep for each time calling loop()
    float s = speed; // get current state
    double dkp, dki;

    // choose action from the Q-table
    rl.chooseAction(s, &dkp, &dki);
    double kp = l_pid.getKp() + dkp;
    double ki = l_pid.getKi() + dki;

    // limit the range of kp and ki
    if(kp > 0.5){kp = 0.5;}
    else if (kp < 0.2){kp = 0.2;}
    if (ki > 0.001){ki = 0.001;}
    else if (ki < 0.00001){ki = 0.00001;}

    float r = 0;
    bool done = false; // check if the episode is done
    takeStep(kp, ki, &done, &r); // perform the action and get the reward and done flag

    // print the log of one timestep in json format.
    doc["time"] = millis() - start_time;
    doc["speed"] = s;
    doc["kp"] = kp;
    doc["ki"] = ki;
    doc["reward"] = r;
    serializeJson(doc, Serial);
    Serial.println();

    delay(1);
}


/**
 * @description: initialize timer3 to generate interrupt every 0.1s. 
 * @return {*}
 */
void timerSetUP(){
    noInterrupts();           // disable all interrupts
    TCCR3A = 0;
    TCCR3B = 0;

    TCNT3 = 65536-(int)(16000000/256*0.1);            // preload timer 65536-16MHz/256*0.1s
    TCCR3B |= (1 << CS12);    // 256 prescaler 
    TIMSK3 |= (1 << TOIE3);   // enable timer overflow interrupt
    interrupts();             // enable all interrupts
}

/**
 * @description: interrupt service routine for timer3. calculate the speed of motor every 0.1s.
 * @return {*}
 */
ISR(TIMER3_OVF_vect)        // interrupt service routine that wraps a user defined function supplied by attachInterrupt
{
    TCNT3 = 65536-(int)(16000000/256*0.1);            // preload timer
    delta_count = count_e1-last_count;
    last_count = count_e1;
    speed = 60.0 * delta_count / cpr / 0.1;   // rpm

    // update latest max_speed
    if (speed > max_speed){
        max_speed = speed;
    }
}


/**
 * @description: This function is an operation of agent in one timestep. The agent perform the actions (kp, ki) in the environment. The environment then transfer to next state and give back the reward and done flag. 
 * @param {double} kp: the parameter of pid controller.
 * @param {double} ki: the parameter of pid controller.
 * @param {bool *} done: the episode flag. When the episode is complete, the done will set to be true.
 * @param {float *} r: the reward recieved from environment.
 * @return {*}
 */
void takeStep(double kp, double ki, bool *done, float *r){
    // perform actions to the environment
    l_pid.setTunings(kp, ki, 0); 
    l_pid.compute();
    motors.setMotorPower(L_PWM, R_PWM);
    
    // calculate the time elapsed since the start of the episode
    unsigned long episode_elapse = millis() - episode_start_time; 

    // calculate reward
    if (episode_elapse < episode_timeout){ // check if the episode has timed out
        *done = false;
        *r = - (float)episode_elapse * 0.01; 
        
        // check if the target speed has been reached in range of 30% * speed_ref
        if (abs(speed - speed_ref) < 0.3 * speed_ref){ 
            delay(100); // wait for 100 milliseconds and check again
            if(abs(speed - speed_ref) < 0.3 * speed_ref){
                *done = true; 
                float overshoot = max_speed - speed_ref; // calculate the overshoot
                *r = 80 - (overshoot +  lambda *(float)episode_elapse * 0.01); 
            }
        }
    }
    else{ // timeout
        *done = true;
        *r = - (float)episode_elapse*0.1;
    }
}