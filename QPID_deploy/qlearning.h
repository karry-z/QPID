/*
 * @Author: Karry Zhang
 * @LastEditTime: 2022-12-15 16:21:20
 * @Description: This file trains Q-learning agent. 
 * 
 * Copyright (c) 2022 Karry Zhang, Chenjie Wang All Rights Reserved. 
 */

#ifndef _Q_LEARNING_H
#define _Q_LEARNING_H

#define NS 25 // num of states
#define NA 9 // num of elements in action vector
#define kp_base 0.001
#define ki_base 0.00001

class QLearning_c{
    public:
    float gamma; // discount factor, default 0.9
    float lr; // learning rate, default 0.1
    double q_table[NS][NA];
    double action_list[NA][2] = {{kp_base, ki_base}, {kp_base, 0}, {kp_base, -ki_base},
                              {0, ki_base}, {0, 0}, {0, -ki_base},
                              {-kp_base, ki_base}, {-kp_base, 0}, {-kp_base, -ki_base}}; // actions are in forms of {dkp, dki}
    QLearning_c(float lr=0.1, float gamma=0.9); 
    int getActionIndex(double dkp, double dki);
    int getStateIndex(float s);
    void chooseAction(float s, double *kp, double *ki);
    void learn(float s, double dkp, double dki, float s_, float r, bool done);
    void getQTableFromEEPROM();
};


/**
 * @description: initialize Q-learning agent
 * @param {float} lr: learning rate, default 0.1
 * @param {float} gamma: discount factor, default 0.9
 * @return {*}
 */
QLearning_c::QLearning_c(float lr=0.1, float gamma=0.9){
    this->lr = lr;
    this->gamma = gamma;
    // init q_table
    for (int i = 0; i < NS; i++){
        for (int j = 0; j < NA; j++){
            q_table[i][j] = 0.0;
        }
    }
}


/**
 * @description: get action_index with action={dkp, dki}
 * @param {double} dkp: an element in action_list. The increment of kp.
 * @param {double} dki: an element in action_list. The increment of ki.
 * @return {int} action_index: the index of action_list
 */
int QLearning_c::getActionIndex(double dkp, double dki){
    int action_index = 0;
    if (abs(dkp-kp_base) <= 1e-6){ // dkp == kp_base
        action_index = 0;
        if (abs(dki-ki_base) <= 1e-6){ // dki == ki_base
            action_index += 0;
        }
        else if (abs(dki-0) <= 1e-6){ // dki == 0
            action_index += 1;
        }
        else{ // dki == -ki_base
            action_index += 2;
        }
    }
    else if (abs(dkp-0) <= 1e-6){ // dkp == 0
        action_index = 3;
        if (abs(dki-ki_base) <= 1e-6){ // dki == ki_base
            action_index += 0;
        }
        else if (abs(dki-0) <= 1e-6){ // dki == 0
            action_index += 1;
        }
        else{ // dki == -ki_base
            action_index += 2;
        }
    }
    else{ // dkp == -kp_base
        action_index = 6;
        if (abs(dki-ki_base) <= 1e-6){ // dki == ki_base
            action_index += 0;
        }
        else if (abs(dki-0) <= 1e-6){ // dki == 0
            action_index += 1;
        }
        else{ // dki == -ki_base
            action_index += 2;
        }
    }
    return action_index;
}


/**
 * @description: get state_index with state s
 * @param {float} s: state, which is the current speed of motor in this system.
 * @return {int} state_index: the state index that can be used in q_table.
 */
int QLearning_c::getStateIndex(float s){
    int state_idx = int(s);
    if (state_idx > NS - 1){
        state_idx = NS - 1;
    }
    else if (state_idx < 0){
        state_idx = 0;
    }
    return state_idx;
}


/**
 * @description: choose action from q_table
 * @param {float} s: state, which is the current speed of motor in this system.
 * @param {double} *dkp: the 1st output action.
 * @param {double} *dki: the 2rd output action.
 * @return {*}
 */
void QLearning_c::chooseAction(float s, double *dkp, double *dki){
    int state_idx = getStateIndex(s);

    // get action_idx which max q value in line s from q-table
    float max_q = q_table[state_idx][0];
    int max_idx = 0;
    for (int i = 1; i < NA; i++){
        if (q_table[state_idx][i] > max_q){
            max_q = q_table[state_idx][i];
            max_idx = i;
        }
    }
    // choose action with action index (max_idx)
    *dkp = action_list[max_idx][0];
    *dki = action_list[max_idx][1];
}


/**
 * @description: agent learning process, which update q_table with bellmen equation. 
 * @param {float} s: the state of current timestep.
 * @param {double} dkp: the actions agent perform in the env at current timestep.
 * @param {double} dki: the actions agent perform in the env at current timestep.
 * @param {float} s_: the state of next timestep, which is transfer from s after take actions in the step.
 * @param {float} r: the reward from s transfer to s_
 * @param {bool} done: the flag to chech whether this episode is done.
 * @return {*}
 */
void QLearning_c::learn(float s, double dkp, double dki, float s_, float r, bool done){
    int state_idx = getStateIndex(s);
    int action_idx = getActionIndex(dkp, dki);
    float q_predict = q_table[state_idx][action_idx];
    float q_target;
    if (done){
        q_target = r;
    }
    else{ // next state is not terminal
        int next_state_idx = getStateIndex(s_);
        
        // get max q value in line s_ from q_table
        float max_q = q_table[next_state_idx][0];
        for (int i = 1; i < NA; i++){
            if (q_table[next_state_idx][i] > max_q){
                max_q = q_table[next_state_idx][i];
            }
        }
        
        q_target = r + gamma * max_q;
    }
    q_table[state_idx][action_idx] += lr * (q_target - q_predict); // update q_table 
}


/**
 * @description: get q_table from eeprom and print it.
 * @return {*}
 */
void QLearning_c::getQTableFromEEPROM(){
    int address = 0;
    for (int i = 0; i < NS; i++){
        for (int j = 0; j < NA; j++){
            EEPROM.get(address, q_table[i][j]);
            address += sizeof(float);
        }
    }
    for (int i = 0; i < NS; i++){
        for (int j = 0; j < NA; j++){
            Serial.println(q_table[i][j]);
        }
    }
}


#endif
