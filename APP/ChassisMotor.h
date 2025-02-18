#ifndef _CHASSISMOTOR_H
#define _CHASSISMOTOR_H

#include "main.h"
#include "can.h"
#include "CloudMotor.h"
#include "Encoder.h"
#include "pid.h"
#include "rc.h"
#include "Remote_Control.h"
#include "user_lib.h"
#include "arm_math.h"
#include "judgement_info.h"

#define CHASSIS_MAXSPEED 1550
//前后的遥控器通道号码
#define CHASSIS_X_CHANNEL 3
//左右的遥控器通道号码
#define CHASSIS_Y_CHANNEL 2
//在特殊模式下，可以通过遥控器控制旋转
#define CHASSIS_WZ_CHANNEL 0

//选择底盘状态 开关通道号
#define MODE_CHANNEL 0
//遥控器前进摇杆（max 660）转化成车体前进速度（m/s）的比例
#define CHASSIS_VX_RC_SEN 0.006f
//遥控器左右摇杆（max 660）转化成车体左右速度（m/s）的比例
#define CHASSIS_VY_RC_SEN 0.005f
//跟随底盘yaw模式下，遥控器的yaw遥杆（max 660）增加到车体角度的比例
#define CHASSIS_ANGLE_Z_RC_SEN 0.000002f
//不跟随云台的时候 遥控器的yaw遥杆（max 660）转化成车体旋转速度的比例
#define CHASSIS_WZ_RC_SEN 0.005f//0.01f

#define CHASSIS_ACCEL_X_NUM 0.1666666667f//0.0833333333//0.1666666667f
#define CHASSIS_ACCEL_Y_NUM 0.3333333333f//0.1666666666//0.3333333333f

#define CHASSIS_RC_DEADLINE 10

#define MOTOR_SPEED_TO_CHASSIS_SPEED_VX 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_VY 0.25f
#define MOTOR_SPEED_TO_CHASSIS_SPEED_WZ 0.25f

#define MOTOR_DISTANCE_TO_CENTER 0.2f

//底盘任务控制间隔 1ms
#define CHASSIS_CONTROL_TIME_MS 1
//底盘任务控制间隔 0.001s
#define CHASSIS_CONTROL_TIME 0.001
//底盘任务控制频率，尚未使用这个宏
#define CHASSIS_CONTROL_FREQUENCE 1000.0f
//底盘3508最大can发送电流值
#define MAX_MOTOR_CAN_CURRENT 6000//16000.0f
//底盘摇摆按键
#define SWING_KEY KEY_PRESSED_OFFSET_CTRL
//底盘前后左右控制按键
#define CHASSIS_FRONT_KEY KEY_PRESSED_OFFSET_W
#define CHASSIS_BACK_KEY KEY_PRESSED_OFFSET_S
#define CHASSIS_LEFT_KEY KEY_PRESSED_OFFSET_A
#define CHASSIS_RIGHT_KEY KEY_PRESSED_OFFSET_D

//m3508转化成底盘速度(m/s)的比例，做两个宏 是因为可能换电机需要更换比例
#define M3508_MOTOR_RPM_TO_VECTOR 0.000415809748903494517209f
#define CHASSIS_MOTOR_RPM_TO_VECTOR_SEN M3508_MOTOR_RPM_TO_VECTOR

//底盘电机最大速度
#define MAX_WHEEL_SPEED 4.0f
//底盘运动过程最大前进速度
#define NORMAL_MAX_CHASSIS_SPEED_X 3.0f
//底盘运动过程最大平移速度
#define NORMAL_MAX_CHASSIS_SPEED_Y 2.9f
//底盘设置旋转速度，设置前后左右轮不同设定速度的比例分权 0为在几何中心，不需要补偿
#define CHASSIS_WZ_SET_SCALE 0.1f

//摇摆原地不动摇摆最大角度(rad)
#define SWING_NO_MOVE_ANGLE 0.7f
//摇摆过程底盘运动最大角度(rad)
#define SWING_MOVE_ANGLE 0.31415926535897932384626433832795f


//底盘电机速度环PID
#define M3508_MOTOR_SPEED_PID_KP 15000.0f
#define M3508_MOTOR_SPEED_PID_KI 10.0f
#define M3508_MOTOR_SPEED_PID_KD 0.0f
#define M3508_MOTOR_SPEED_PID_MAX_OUT MAX_MOTOR_CAN_CURRENT
#define M3508_MOTOR_SPEED_PID_MAX_IOUT 2000.0f

//底盘旋转跟随PID
#define CHASSIS_FOLLOW_GIMBAL_PID_KP 40.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KI 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_KD 0.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_OUT 6.0f
#define CHASSIS_FOLLOW_GIMBAL_PID_MAX_IOUT 0.2f

//轮子轴心长宽
#define CHASSIS_LENGTH 0.415f
#define CHASSIS_WIDTH  0.39f
typedef enum
{
    CHASSIS_VECTOR_FOLLOW_GIMBAL_YAW,
    CHASSIS_VECTOR_FOLLOW_CHASSIS_YAW,
    CHASSIS_VECTOR_NO_FOLLOW_YAW,
    CHASSIS_VECTOR_RAW,

    //  CHASSIS_AUTO,
    //  CHASSIS_FOLLOW_YAW,
    //  CHASSIS_ENCODER,
    //  CHASSIS_NO_ACTION,
    //  CHASSIS_RELAX,
} chassis_mode_e;

typedef struct
{
    //const motor_measure_t *chassis_motor_measure;
    fp32 accel;
    fp32 speed;
    fp32 speed_set;
    int16_t give_current;
} Chassis_Motor_t;


typedef struct
{
    const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针
    const Gimbal_Motor_t *chassis_yaw_motor;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角
    const Gimbal_Motor_t *chassis_pitch_motor; //底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
    const fp32 *chassis_INS_angle;             //获取陀螺仪解算出的欧拉角指针
    chassis_mode_e chassis_mode;               //底盘控制状态机
    chassis_mode_e last_chassis_mode;          //底盘上次控制状态机
    Chassis_Motor_t motor_chassis[4];          //底盘电机数据
    PidTypeDef motor_speed_pid[4];             //底盘电机速度pid 0:LF 1:RF 2:LB 3:RB
    PidTypeDef chassis_angle_pid;              //底盘跟随角度pid

    first_order_filter_type_t chassis_cmd_slow_set_vx;
    first_order_filter_type_t chassis_cmd_slow_set_vy;

    fp32 vx;                         //底盘速度 前进方向 前为正，单位 m/s
    fp32 vy;                         //底盘速度 左右方向 左为正  单位 m/s
    fp32 wz;                         //底盘旋转角速度，逆时针为正 单位 rad/s
    fp32 vx_set;                     //底盘设定速度 前进方向 前为正，单位 m/s
    fp32 vy_set;                     //底盘设定速度 左右方向 左为正，单位 m/s
    fp32 wz_set;                     //底盘设定旋转角速度，逆时针为正 单位 rad/s
    fp32 chassis_relative_angle;     //底盘与云台的相对角度，单位 rad/s
    fp32 chassis_relative_angle_set; //设置相对云台控制角度
    fp32 chassis_yaw_set;

    fp32 vx_max_speed;  //前进方向最大速度 单位m/s
    fp32 vx_min_speed;  //前进方向最小速度 单位m/s
    fp32 vy_max_speed;  //左右方向最大速度 单位m/s
    fp32 vy_min_speed;  //左右方向最小速度 单位m/s
    fp32 chassis_yaw;   //陀螺仪和云台电机叠加的yaw角度
    fp32 chassis_pitch; //陀螺仪和云台电机叠加的pitch角度
    fp32 chassis_roll;  //陀螺仪和云台电机叠加的roll角度

} chassis_move_t;

extern chassis_move_t chassis_move;

void chassis_init(void);
void chassis_control_loop(void);


extern const Gimbal_Motor_t *get_yaw_motor_point(void);
extern const Gimbal_Motor_t *get_pitch_motor_point(void);
//extern void chassis_task(void);
extern void chassis_rc_to_control_vector(fp32 *vx_set, fp32 *vy_set, chassis_move_t *chassis_move_rc_to_vector);
#endif //_CHASSISMOTOR_H
