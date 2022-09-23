#pragma once
class PositionPID
{
private:
    float  dt;   //循环时间间隔
    float _max;  // 最大输出限制，规避过冲
    float _min;  // 最小输出限制
    float k_p;  // 比例系数
    float k_i;  // 积分系数
    float k_d;  // 微分系数

    bool l_limit;
    float target; // 目标值
    float cur_val;  //算法当前PID位置值，第一次为设定的初始位置
    float _pre_error = 0;  // t-1 时刻误差值
    float _integral = 0;  // 误差积分值

public:

    PositionPID(float target,float max,float min,float dt,float k_p,float k_i,float k_d,bool l_limit=true);
    float calculate(float cur_val);
};



class DeltaPID
{
private:
    float dt;   //循环时间间隔
    float k_p;  //比例系数
    float k_i;  //积分系数
    float k_d;  //微分系数

    float target;  //目标值
    float cur_val;  //算法当前PID位置值
    float _pre_error = 0;  //t-1 时刻误差值
    float _pre_pre_error = 0;  //t-2 时刻误差值

public:
    DeltaPID(float target,float dt,float p,float i,float d);
    float calculate(float cur_val);
};

