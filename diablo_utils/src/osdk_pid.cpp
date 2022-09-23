#include <math.h>
#include "diablo_utils/diablo_tools/osdk_pid.hpp"

PositionPID::PositionPID(float target,float max,float min,float dt,float k_p,float k_i,float k_d,bool l_limit)
{
    this->dt = dt;
    this->target = target;
    this->k_p = k_p;
    this->k_i = k_i;
    this->k_d = k_d;
    this->_max = max;
    this->_min = min;
    this->l_limit = l_limit;
}


float PositionPID::calculate(float cur_val){
    float error = this->target - cur_val;
    //比例项
    float p_out = this->k_p * error;
    //积分项
    this->_integral += (error * this->dt);
    float i_out = this->k_i * this->_integral;

    //微分项
    float derivative = (error - this->_pre_error) / this->dt;
    float d_out = this->k_d * derivative;
    // t 时刻pid输出
    float output = p_out + i_out + d_out;

    // 限制输出值
    if(output > this->_max)
        output = this->_max;
    else if(output < this->_min)
        output = this->_min;
    
    this->_pre_error = error;
    cur_val = output;

    if(l_limit){
        if(cur_val >= 0){
            if (fabs(cur_val) < 0.1){
                cur_val = 0.1;
            }
        }else{
            if (fabs(cur_val) < 0.1){
                cur_val = -0.1;
            }
        }
    }
    return cur_val;
}


DeltaPID::DeltaPID(float target,float dt,float p,float i,float d)
{
    this->dt = dt;  //循环时间间隔
    this->k_p = p;  //比例系数
    this->k_i = i;  //积分系数
    this->k_d = d;  //微分系数

    this->target = target;  //目标值
    this->_pre_error = 0;  //t-1 时刻误差值
    this->_pre_pre_error = 0;  //t-2 时刻误差值
}

float DeltaPID::calculate(float cur_val){
    float error = this->target - cur_val;
    float p_change = this->k_p * (error - this->_pre_error);
    float i_change = this->k_i * error;
    float d_change = this->k_d * (error - 2 * this->_pre_error + this->_pre_pre_error);
    float delta_output = p_change + i_change + d_change;  //本次增量
    cur_val += delta_output;  //计算当前位置

    this->_pre_error = error;
    this->_pre_pre_error = this->_pre_error;
    return cur_val;
}
