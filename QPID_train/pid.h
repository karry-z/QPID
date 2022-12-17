/*
 * @Author: Karry Zhang
 * @LastEditTime: 2022-12-15 16:07:58
 * @Description: This file trains Q-learning agent. 
 * 
 * Copyright (c) 2022 Karry Zhang, Chenjie Wang All Rights Reserved. 
 */
#ifndef _PID_H
#define _PID_H


class PID_c {
    double kp, ki, kd; // gains of PID controller
    double i_val_sum = 0; // sum of integral value
    double p_val=0, i_val=0, d_val=0; // proportional, integral, derivative value
    double err = 0; // error
    double last_err = 0; // last error
    double *control_variable; // the output of the system being controlled. in this case, it is the speed of the motor.
    double *output; // the output of PID controller 
    double *ref; // the reference value of control variable
    double out_min, out_max; // limit of output of PID controller
    unsigned long min_sample_time; // to avoid the PID controller to update too frequently
    unsigned long last_time; // the last time the PID controller updates

    public:
    /**
     * @description: Initialize the PID controller
     * @param {double} *control_variable
     * @param {double} *output
     * @param {double} *ref
     * @param {double} kp
     * @param {double} ki
     * @param {double} kd
     * @return {*}
     */    
    PID_c(double *control_variable, double *output, double *ref, double kp=0, double ki=0, double kd=0){
        this->control_variable = control_variable;
        this->output = output;
        this->ref = ref;
        last_time = 0;
        min_sample_time = 0.1;
        this->out_min = 0;
        this->out_max = 255;
        setTunings(kp, ki, kd);
    }

    /**
     * @description: This function computes the output of PID controller once. It should be called in loop().
     * @return {bool} It's true when the output of PID controller is updated.
     */    
    bool compute(){
        unsigned long now = millis();
        unsigned long time_elaspe = (now - last_time);
        if (time_elaspe >= min_sample_time) {
            err = *ref - *control_variable;
            // calculate proportional
            p_val = kp * err;
            
            // calculate integral
            i_val += ki * err;
            if (i_val > 50){
                i_val = 50;
            }
            else if(i_val < 0){
                i_val = 0;
            }

            // calculate derivative
            d_val = kd * (err - last_err); 

            *output = p_val + i_val + d_val;

            // set limit
            if (*output > out_max)
                *output = out_max;
            else if (*output < out_min)
                *output = out_min;
            
            // update last_err and last_time
            last_err = err;
            last_time = now;
            return true;
        }
        return false; // not time to update the output of PID controller
    }

    /**
     * @description: This function sets the parameters of PID controller in progress. It allows to change the parameters by Q-learning agent in real time.
     * @param {double} kp
     * @param {double} ki
     * @param {double} kd
     * @return {*}
     */
    void setTunings(double kp, double ki, double kd){
        this->kp = kp;
        this->ki = ki;
        this->kd = kd;
    }

    /**
     * @description: reset pid controller to allow to start a new episode.
     * @return {*}
     */    
    void resetPID(){
        err = 0;
        last_err = 0; 
        p_val = 0; 
        i_val = 0; 
        d_val = 0; 
        last_time = 0;
    }

    /* here are some getters used to display. */ 
    double getKp() { return kp; }
    double getKi() { return ki; }
    double getKd() { return kd; }
    double getPVal() { return p_val; }
    double getIVal() { return i_val; }
    double getDVal() { return d_val; }
    double getErr() { return err; }
    double getLastErr() { return last_err; }
};


#endif
