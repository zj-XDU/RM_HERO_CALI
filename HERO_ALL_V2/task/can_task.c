#include "can_task.h"
#include "cmsis_os.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "detect_task.h"
#include "shoot_task.h"

void can_task(void const *pvParameters)
{

    portTickType can_task_pre_tick = 0;

    while(1)
    {



/*******************************************��������CAN�շ�********************************************/
	    //make sure  one motor is online at least, so that the control CAN message can be received
        //ȷ������һ��������ߣ� ����CAN���ư����Ա����յ�
        if (!(toe_is_error(CHASSIS_MOTOR1_TOE) && toe_is_error(CHASSIS_MOTOR2_TOE) && toe_is_error(CHASSIS_MOTOR3_TOE) && toe_is_error(CHASSIS_MOTOR4_TOE)))
        {
            //when remote control is offline, chassis motor should receive zero current. 
            //��ң�������ߵ�ʱ�򣬷��͸����̵�������.
            if (toe_is_error(DBUS_TOE))//||toe_is_error(INS_TOE))
			//if (toe_is_error(INS_TOE))
            {
                CAN_cmd_chassis(0, 0, 0, 0);
            }
            else
            {
                //send control current
                //���Ϳ��Ƶ���
                CAN_cmd_chassis(chassis_move.motor_chassis[0].give_current, chassis_move.motor_chassis[1].give_current,
                                chassis_move.motor_chassis[2].give_current, chassis_move.motor_chassis[3].give_current);
            }
        }

		osDelay(1);
/*******************************************��̨����CAN�շ�********************************************/

        if (!(toe_is_error(YAW_GIMBAL_MOTOR_TOE) && toe_is_error(PITCH_GIMBAL_MOTOR_TOE) ))//&& toe_is_error(INS_TOE)))
        {
            if (toe_is_error(DBUS_TOE)||toe_is_error(INS_TOE))
            {
                CAN_cmd_gimbal(0, 0, 0, 0);
				CAN1_cmd_gimbal(0, 0, 0, 0);//
            }
            else
            {//gimbal_control.yaw_current_set
			
			
					 CAN_cmd_gimbal(gimbal_control.yaw_current_set, gimbal_control.pit_current_set, shoot_control.shoot_current[0], shoot_control.shoot_current[1]);
					 CAN1_cmd_gimbal(shoot_control.shoot_current[2], 0, 0, 0);			//gimbal_control.trigle_current_set	
            }
        }
		osDelay(1);
//gimbal_control.yaw_current_set

		osDelayUntil(&can_task_pre_tick,4);
    
    }




}