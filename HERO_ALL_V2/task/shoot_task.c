/**
  ****************************(C) COPYRIGHT 2019 DJI****************************
  * @file       shoot.c/h
  * @brief      �������.
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

#include "shoot_task.h"
#include "main.h"

#include "cmsis_os.h"

//#include "bsp_laser.h"
//#include "bsp_fric.h"
#include "arm_math.h"
#include "user_lib.h"
#include "referee.h"

#include "bsp_can.h"
#include "gimbal_behaviour.h"
#include "detect_task.h"
#include "pid.h"

#define shoot_fric1_on(pwm) fric1_on((pwm)) //Ħ����1pwm�궨��
#define shoot_fric2_on(pwm) fric2_on((pwm)) //Ħ����2pwm�궨��
#define shoot_fric_off()    fric_off()      //�ر�����Ħ����

#define shoot_laser_on()    laser_on()      //���⿪���궨��
#define shoot_laser_off()   laser_off()     //����رպ궨��
//΢������IO
#define BUTTEN_TRIG_PIN HAL_GPIO_ReadPin(BUTTON_TRIG_GPIO_Port, BUTTON_TRIG_Pin)




/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void);
/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void);

/**
  * @brief          ��ת��ת����
  * @param[in]      void
  * @retval         void
  */
static void trigger_motor_turn_back(void);

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void);

static void shoot_set_control();

shoot_control_t shoot_control;          //�������


/**
  * @brief          �����ʼ������ʼ��PID��ң����ָ�룬���ָ��
  * @param[in]      void
  * @retval         ���ؿ�
  */
void shoot_init(void)
{


    shoot_control.shoot_mode = SHOOT_STOP;		//���������ʼ״̬Ϊֹͣ������Ϊ
    //ң����ָ��
    shoot_control.shoot_rc = get_remote_control_point();
    //���ָ��
    shoot_control.shoot_motor_measure0 = get_shoot_motor_measure_point(0);		//��ȡĦ���ֵ������ָ��
	shoot_control.shoot_motor_measure1 = get_shoot_motor_measure_point(1);		//��ȡĦ���ֵ������ָ��
	shoot_control.trigle_motor_meaure =  get_trigle_motor_measure_point();		//��ȡ���̵������ָ��

    //��������
    shoot_control.angle = shoot_control.shoot_motor_measure0->ecd * MOTOR_ECD_TO_ANGLE;
    shoot_control.given_current = 0;
    shoot_control.move_flag = 0;
    shoot_control.set_angle = shoot_control.angle;
    shoot_control.speed = 0.0f;
    shoot_control.speed_set = 0.0f;
    shoot_control.key_time = 0;
	shoot_control.fric0_speed_set = 0;
	shoot_control.fric1_speed_set = 0;
	shoot_control.ecd_now = (fp32)shoot_control.trigle_motor_meaure->ecd * 3.14 / 8191;			//��¼��ʼʱ�̵ĽǶ�ֵ
	shoot_control.ecd_last = (fp32)shoot_control.trigle_motor_meaure->last_ecd* 3.14 /8191;
	shoot_control.ecd_set = shoot_control.ecd_now;			//��¼��ʼʱ�̵ĽǶ�ֵ

	 shoot_control.shoot_mode = SHOOT_STOP;
	
	
	
	
}

/**
  * @brief          ���ѭ��
  * @param[in]      void
  * @retval         ������
  */
void shoot_task(void const *pvParameters)
{

	portTickType shoot_task_pre_tick = 0;
	shoot_init();
  while(1)
  {
	fp32 temp_ecd=0;
    shoot_set_mode();        //����״̬��
	shoot_set_control();	 //���ݵ�ǰ״̬���ÿ��Ʊ���
    
   shoot_feedback_update(); //��������
   osDelayUntil(&shoot_task_pre_tick,1);
   
 }
}

/**
  * @brief          ���״̬�����ã�ң�����ϲ�һ�ο��������ϲ��رգ��²�1�η���1�ţ�һֱ�����£���������䣬����3min׼��ʱ�������ӵ�
  * @param[in]      void
  * @retval         void
  */
static void shoot_set_mode(void)
{
    static int8_t last_s = RC_SW_UP;



	if((shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_Q) || (shoot_control.shoot_mode ==SHOOT_READY_FRIC) ||(shoot_control.shoot_mode == SHOOT_DONE) || shoot_control.shoot_mode == SHOOT_BULLET )
		shoot_control.fric_flag = 1;
	else if( (shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_E) || (shoot_control.shoot_mode ==SHOOT_STOP) )
		shoot_control.fric_flag = 0;



//�ж�Ħ�����Ƿ�ʼת��
    //�ϲ��жϣ� һ�ο������ٴιر�
    if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode == SHOOT_STOP))
    {
       shoot_control.shoot_mode = SHOOT_READY_FRIC;
     }
    else if ((switch_is_up(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_up(last_s) && shoot_control.shoot_mode != SHOOT_STOP))
    {
        shoot_control.shoot_mode = SHOOT_STOP;
    }



    //�����е��� ����ʹ�ü��̿���Ħ����
    if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_ON_KEYBOARD) && shoot_control.shoot_mode == SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_READY_FRIC;
    }
    //�����е��� ����ʹ�ü��̹ر�Ħ����
    else if (switch_is_mid(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && (shoot_control.shoot_rc->key.v & SHOOT_OFF_KEYBOARD) && shoot_control.shoot_mode != SHOOT_STOP)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
		shoot_control.press_l = 0;
    }








//�жϲ����Ƿ�ת��
	//Ħ���ֿ�ʼת��֮����뼴�ɽ���Ԥ����ģʽ
	if(shoot_control.shoot_mode == SHOOT_READY_FRIC)
    {
        //�²�һ�λ�������������һ�Σ��������״̬
        if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s) )|| ( shoot_control.press_l))//&& shoot_control.last_press_l == 0)  || (shoot_control.shoot_rc->key.v & KEY_PRESSED_OFFSET_R))//|| (shoot_control.press_r && shoot_control.last_press_r == 0))
        {
            shoot_control.shoot_mode = SHOOT_BULLET;
			 //shoot_control.press_l = 0;
        }
    }
	//ÿ�η������֮����뾭��һ��ʱ����ܽ�����һ�η���
	else if(shoot_control.shoot_mode == SHOOT_DONE)
	{	 
	    if ((switch_is_down(shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL]) && !switch_is_down(last_s)) || (shoot_control.press_l) )// && shoot_control.last_press_l == 0) )//|| (shoot_control.press_r && shoot_control.last_press_r == 0))
        {
            shoot_control.shoot_mode = SHOOT_BULLET;			
			//shoot_control.press_l = 0;
        }
		shoot_control.ecd_set = shoot_control.ecd_now;
			if(shoot_control.move_flag == 0)
				shoot_control.press_l_time++;
			//if(shoot_control.press_l_time == PRESS_LONG_TIME)
				
		
	}
	




	
 
//ϵͳ���Ͼ��ߺ�ǹ����������

	//ǹ����������
	//��ȡӢ�ۻ�����ǹ��ÿ����ȴֵ
    get_shoot_heat1_limit_and_heat1(&shoot_control.heat_limit, &shoot_control.heat);
	
	//����ϵͳ���ߣ����ߵ�ǰ��������ʣ��������������
    if(!toe_is_error(REFEREE_TOE) && (shoot_control.heat + SHOOT_HEAT_REMAIN_VALUE > shoot_control.heat_limit))
    {	
		//�������ϣ������ǰ���ڷ���״̬���˻ص���һ״̬����Ħ��������״̬��ȡ������ʹ�ܣ�
        if(shoot_control.shoot_mode == SHOOT_BULLET )//|| shoot_control.shoot_mode == SHOOT_CONTINUE_BULLET)
        {
        //    shoot_control.shoot_mode =SHOOT_STOP;
        }
    }
    //�����̨״̬�� ����״̬���͹ر����
    if (gimbal_cmd_to_shoot_stop())
    {
        //shoot_control.shoot_mode = SHOOT_STOP;
    }

    last_s = shoot_control.shoot_rc->rc.s[SHOOT_RC_MODE_CHANNEL];
}
/**
  * @brief          ������ݸ���
  * @param[in]      void
  * @retval         void
  */
static void shoot_feedback_update(void)
{



//������갴������Ƶ�ʣ�����
		if((shoot_control.shoot_rc->mouse.press_l)&&(!shoot_control.press_l)&&((shoot_control.shoot_mode == SHOOT_READY_FRIC ) || (shoot_control.shoot_mode == SHOOT_DONE&& shoot_control.press_l_time >= PRESS_LONG_TIME)  ))
		{
			shoot_control.press_l = 1;
			shoot_control.press_l_time = 0;
		}	



//���Ȧ�����ã� ��Ϊ�������תһȦ�� �������ת 36Ȧ������������ݴ������������ݣ����ڿ��������Ƕ�
    if (shoot_control.trigle_motor_meaure->ecd - shoot_control.trigle_motor_meaure->last_ecd > HALF_ECD_RANGE)
    {
       shoot_control.ecd_count--;
    }
    else if (shoot_control.trigle_motor_meaure->ecd - shoot_control.trigle_motor_meaure->last_ecd < -HALF_ECD_RANGE)
    {
        shoot_control.ecd_count++;
    }

    if (shoot_control.ecd_count == FULL_COUNT)
    {
        shoot_control.ecd_count = -(FULL_COUNT - 1);
    }
    else if (shoot_control.ecd_count == -FULL_COUNT)
    {
        shoot_control.ecd_count = FULL_COUNT - 1;
    }

    //���������Ƕ�
    shoot_control.angle = (shoot_control.ecd_count * ECD_RANGE + shoot_control.trigle_motor_meaure->ecd) * MOTOR_ECD_TO_ANGLE;
    
	
	
 
//��Ȧ��¼
	if(shoot_control.trigle_motor_meaure->ecd > shoot_control.trigle_motor_meaure->last_ecd )
		{if(shoot_control.trigle_motor_meaure->speed_rpm < 0)
			shoot_control.ecd_now = shoot_control.ecd_now + ((fp32)shoot_control.trigle_motor_meaure->ecd - (fp32)shoot_control.trigle_motor_meaure->last_ecd - 8191) *3.14 / 8191;
		 else
			shoot_control.ecd_now = shoot_control.ecd_now + ((fp32)shoot_control.trigle_motor_meaure->ecd - (fp32)shoot_control.trigle_motor_meaure->last_ecd) *3.14/8191;
		
		}
	else if(shoot_control.trigle_motor_meaure->ecd < shoot_control.trigle_motor_meaure->last_ecd )
	{
		if(shoot_control.trigle_motor_meaure->speed_rpm <= 0)
			shoot_control.ecd_now = shoot_control.ecd_now + ((fp32)shoot_control.trigle_motor_meaure->ecd - (fp32)shoot_control.trigle_motor_meaure->last_ecd ) *3.14 /8191;
		 else
			shoot_control.ecd_now = shoot_control.ecd_now + ((fp32)shoot_control.trigle_motor_meaure->ecd - (fp32)shoot_control.trigle_motor_meaure->last_ecd + 8191) * 3.14 /8191;
	}
	else if(shoot_control.trigle_motor_meaure->ecd == shoot_control.trigle_motor_meaure->last_ecd )	
			shoot_control.ecd_now = shoot_control.ecd_now;
  


}




//���̿�����ת��Ᵽ��
static void trigger_motor_turn_back(void)
{
    if( shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.trigger_speed_set = -TRIGGER_SPEED;
    }
    else
    {
        shoot_control.trigger_speed_set = TRIGGER_SPEED;
    }

    if(fabs(shoot_control.speed) > BLOCK_TRIGGER_SPEED && shoot_control.block_time < BLOCK_TIME)
    {
        shoot_control.block_time++;
        shoot_control.reverse_time = 0;
    }
    else if (shoot_control.block_time == BLOCK_TIME && shoot_control.reverse_time < REVERSE_TIME)
    {
        shoot_control.reverse_time++;
    }
    else
    {
        shoot_control.block_time = 0;
    }
}

/**
  * @brief          ������ƣ����Ʋ�������Ƕȣ����һ�η���
  * @param[in]      void
  * @retval         void
  */
static void shoot_bullet_control(void)
{
	

	//���÷�������ƶ������������ǰ���¹ر�Ħ���ְ������ͽ��뷢��ֹͣ״̬
	if(shoot_control.key == SHOOT_OFF_KEYBOARD)
    {
        shoot_control.shoot_mode = SHOOT_STOP;
		return ;
    }

    //ÿ��ת��һ���Ƕȣ�����һ���ӵ�
    if( (shoot_control.move_flag == 0) ) 
    {

		shoot_control.ecd_set -= TRIGLE_ANGLE_ADD;					//��С��ǰ��Ԥ��Ƕȣ�ָʾ����ת��
        shoot_control.move_flag = 1;
    }
	
			//����Ƕ��ж�
    if ( shoot_control.ecd_set > shoot_control.ecd_now)
    {
        //û����һֱ������ת�ٶ�
        //shoot_control.trigger_speed_set = -TRIGGER_SPEED;
		
		shoot_control.move_flag = 0;
		
    }


}


//�������״̬�����������
static void shoot_set_control()
{
//���̿��Ƶ���״̬��
    if(shoot_control.shoot_mode == SHOOT_STOP)
    {	
		//ֹͣģʽ��Ħ���ֺͲ��̿��Ƶ�����Ϊ0
		 shoot_control.shoot_current[2] = 0;
		 shoot_control.move_flag = 0;
		 shoot_control.press_l = 0;
		 
		 
    }
    else if(shoot_control.shoot_mode  == SHOOT_READY_FRIC)
    {
		 shoot_control.shoot_current[2] = 0;
	}	 
		//Ħ����׼��û�����ʱ���̺�Ħ���ֵ������Ϊ0
    else if(shoot_control.shoot_mode == SHOOT_BULLET)
        {
          
		  //����������̽Ƕȿ���
		   shoot_bullet_control();
		   trigger_motor_turn_back();
		  //���ٶȻ����ƣ��ı��ٶȿ��Կ�����Ƶ
		   
		
			
		   if(shoot_control.move_flag == 0)
		   {
				shoot_control.press_l = 0;
				shoot_control.shoot_mode = SHOOT_DONE;
				
		   }
				

        }
		
	


}


