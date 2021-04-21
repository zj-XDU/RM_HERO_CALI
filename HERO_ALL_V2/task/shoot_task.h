/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      ������ܡ�
  * @note       
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Dec-26-2018     RM              1. ���
  *
  @verbatim
  ==============================================================================

  ==============================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2019 DJI****************************
  */

#ifndef SHOOT_H
#define SHOOT_H
#include "struct_typedef.h"

#include "bsp_can.h"
#include "gimbal_task.h"
#include "remote_control.h"
#include "user_lib.h"



#define trigle_pid_kp               1000.0f
#define trigle_pid_ki               100.0f
#define trigle_pid_kd               0.0f
#define trigle_pid_max              8000.0f
#define trigle_pid_maxi             3000.0f


#define trigle_pid_speed_kp               1000.0f
#define trigle_pid_speed_ki               60.0f
#define trigle_pid_speed_kd               0.0f
#define trigle_pid_speed_max              9000.0f
#define trigle_pid_speed_maxi             500.0f


#define TRIGLE_ANGLE_ADD				  8

//������俪��ͨ������
#define SHOOT_RC_MODE_CHANNEL       1
//��̨ģʽʹ�õĿ���ͨ��

#define SHOOT_CONTROL_TIME          GIMBAL_CONTROL_TIME

#define SHOOT_FRIC_PWM_ADD_VALUE    100.0f

//���Ħ���ּ���� �ر�
#define SHOOT_ON_KEYBOARD           KEY_PRESSED_OFFSET_Q
#define SHOOT_OFF_KEYBOARD          KEY_PRESSED_OFFSET_E

//�����ɺ� �ӵ�����ȥ���ж�ʱ�䣬�Է��󴥷�
#define SHOOT_DONE_KEY_OFF_TIME     15
//��곤���ж�
#define PRESS_LONG_TIME             80
//ң����������ش��µ�һ��ʱ��� ���������ӵ� �����嵥
#define RC_S_LONG_TIME              2000
//Ħ���ָ��� ���� ʱ��
#define UP_ADD_TIME                 80
//�����������ֵ��Χ
#define HALF_ECD_RANGE              4096
#define ECD_RANGE                   8191
//���rmp �仯�� ��ת�ٶȵı���
#define MOTOR_RPM_TO_SPEED          0.00290888208665721596153948461415f
#define MOTOR_ECD_TO_ANGLE          0.000021305288720633905968306772076277f
#define FULL_COUNT                  16
//�����ٶ�
#define TRIGGER_SPEED               800.0f
#define CONTINUE_TRIGGER_SPEED      15.0f
#define READY_TRIGGER_SPEED         5.0f

#define KEY_OFF_JUGUE_TIME          500
#define SWITCH_TRIGGER_ON           0
#define SWITCH_TRIGGER_OFF          1

//����ʱ�� �Լ���תʱ��
#define BLOCK_TRIGGER_SPEED         -1100.0f
#define BLOCK_TIME                  700
#define REVERSE_TIME                100
#define REVERSE_SPEED_LIMIT         13.0f

#define PI_FOUR                     0.78539816339744830961566084581988f
#define PI_TEN                      0.314f

//�����ֵ��PID
#define TRIGGER_ANGLE_PID_KP        800.0f
#define TRIGGER_ANGLE_PID_KI        0.5f
#define TRIGGER_ANGLE_PID_KD        0.0f

#define TRIGGER_BULLET_PID_MAX_OUT  10000.0f
#define TRIGGER_BULLET_PID_MAX_IOUT 9000.0f

#define TRIGGER_READY_PID_MAX_OUT   16000.0f
#define TRIGGER_READY_PID_MAX_IOUT  7000.0f


#define SHOOT_HEAT_REMAIN_VALUE     80



//Ħ����ת��������λ
#define SHOOT_FRIC_SPEED_LOW		5000
#define SHOOT_FRIC_SPEED_MID		7000
#define	SHOOT_FRIC_SPEED_HIG		10000

typedef enum
{
    SHOOT_STOP = 0,
    SHOOT_READY_FRIC,       //Ħ���ֿ�ʼת��
    //SHOOT_READY_BULLET,	//ת�ٴﵽĿ��ת��֮������ӵ�Ԥװ��
    //SHOOT_READY,			//
    SHOOT_BULLET,
    //SHOOT_CONTINUE_BULLET,
    SHOOT_DONE,
} shoot_mode_e;


typedef struct
{
    shoot_mode_e shoot_mode;
    const RC_ctrl_t *shoot_rc;
	const motor_measure_t *trigle_motor_meaure;
    const motor_measure_t *shoot_motor_measure0;
	const motor_measure_t *shoot_motor_measure1;
    pid_type_def trigger_motor_pid;			//����PID�ṹ��
	pid_type_def trigger_motor_speed_pid;			//����PID�ṹ��
	
	pid_type_def fric_motor0_pid;			//Ħ����0PID�ṹ��
	pid_type_def fric_motor1_pid;			//Ħ����1PID�ṹ��
	
	
    fp32 trigger_speed_set;
    fp32 speed;
    fp32 speed_set;
    fp32 angle;
    fp32 set_angle;
    int16_t given_current;
    int8_t ecd_count;

    bool_t press_l;
    bool_t press_r;
    bool_t last_press_l;
    bool_t last_press_r;
    uint16_t press_l_time;
    uint16_t press_r_time;
    uint16_t rc_s_time;

    uint16_t block_time;
    uint16_t reverse_time;
    bool_t move_flag;

    bool_t key;
    uint8_t key_time;

    uint16_t heat_limit;
    uint16_t heat;
	
	int16_t fric0_speed_set;
	int16_t fric1_speed_set;
	
	fp32 ecd_now;
	fp32 ecd_last;
	fp32 ecd_add;
	fp32 ecd_set;
	fp32 ecd_det;
	
	
	int fric_flag;
	int16_t shoot_current[3];
} shoot_control_t;

//�����������̨ʹ��ͬһ��can��id��Ҳ�����������̨������ִ��
extern void shoot_init(void);
extern void shoot_task(void const *pvParameters);

extern shoot_control_t shoot_control;          //�������

#endif
