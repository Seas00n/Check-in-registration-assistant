#include "algorithm.h"
#include "pros.h"
#include "time_measure.h"
#include "ads1256_spi.h"
#include "motor.h"
#include <stdlib.h>
#include "main.h"
#include "main.h"
#include "math.h"

/***********************************************************************                                                                 1

	int *get_Positon_Velocity(void)
    返回ELMO驱动器所读的位置速度
    使用方法    int *p；
               p=get_Positon_Velocity；
		       knee_position=*(get_Positon_Velocity);
			   knee_velocity=*(get_Positon_Velocity+1);
		编码器信息：
		            膝关节   7500
					踝关节   2500
*************************************************************************/
float *get_Positon_Velocity(void) {
    //static float Posi_Vel[4];                                        //关节数据读取
    static float Posi_Vel[8];
	  motorSendSync();
//      HAL_Delay(1);	//发送同步帧
    Posi_Vel[0] = (float) motor_2.positionActual / 8192.0 / 50.0 * 360.0;              //knee position    7500
    Posi_Vel[1] = (float) motor_2.velocityActual / 8192.0 / 50.0 * 360.0;              //knee velocity    7500
    Posi_Vel[2] = (float) motor_1.positionActual / 8192.0 / 50.0 * 360.0;              //ankle position   2500
    Posi_Vel[3] = (float) motor_1.velocityActual / 8192.0 / 50.0 * 360.0;              //ankle velocity   2500
    Posi_Vel[4] = (float) motor_2.currentActual;
	  Posi_Vel[5] = (float) motor_2.torqueActual;
	  Posi_Vel[6] = (float) motor_1.currentActual;
	  Posi_Vel[7] = (float) motor_1.torqueActual;
	  return Posi_Vel;                                                 //返回数组首地址
}


/***********************************************************************                                                                   2
    void Impedance_control(JointparaTypeDef *Knee,JointparaTypeDef *Ankle)
    输出膝关节和踝关节的阻抗控制，未加延时
    阻抗控制方程：
                   k*(angle-angle_equ)+b*angle_velocity
             其中：
                   k-----------------------刚度系数
                   b-----------------------阻尼系数
                   angle-------------------当前角度
                   angle_equ---------------平衡角度
                   angle_velocity--------当前角速度
    使用方法：  Impedance_control(JointparaTypeDef *Knee,JointparaTypeDef *Ankle)
*************************************************************************/
/*void Impedance_control(JointparaTypeDef *Knee, JointparaTypeDef *Ankle, float *Posi_Vel, uint16_t *a, uint16_t *b)      //入口参数膝关节和踝关节的阻抗控制参数，两个结构体
{
    int I_ankle, I_knee, com_knee, com_ankle;
    //knee摩擦力补偿
    if (*(Posi_Vel + 1) > 1)
        com_knee = 20;
    else if (*(Posi_Vel + 1) < -1)
        com_knee = -20;
    else
        com_knee = 0;
    //ankle摩擦力补偿
    if (*(Posi_Vel + 3) > 1.5)
        com_ankle = 20;
    else if (*(Posi_Vel + 3) < -1.5)
        com_ankle = -20;
    else
        com_ankle = 0;

    I_knee = (int) (Knee->k * (Knee->q_d - (*Posi_Vel)) + Knee->b * (Knee->qv_d - (*(Posi_Vel + 1))) +
                    com_knee);//+com_knee;              //计算膝关节电流+com_knee
		
		
		//I_knee & 0xFFFF //低16
		//I_knee & 0xFFFF0000  //高16   
		// uint16_t
    I_ankle = (int) (Ankle->k * (Ankle->q_d - (*(Posi_Vel + 2))) + Ankle->b * (Ankle->qv_d - *(Posi_Vel + 3)) +
                     com_ankle);//+com_ankle;      //计算踝关节电流
		//if(I_ankle < 0)  a |= 0b1(15*0)
		//uint16_t  a =  (uint16_t)I_ankle;
		//

    if (I_knee >= 3000)         //判断输出电流是否过大
    {
        I_knee = 3000;
    }
    if (I_knee <= -3000)         //判断输出电流是否过大
    {
        I_knee = -3000;
    }
    if (I_ankle >= 3000)         //判断输出电流是否过大
    {
        I_ankle = 3000;
    }
    if (I_ankle <= -3000)         //判断输出电流是否过大
    {
        I_ankle = -3000;
    }
		uint16_t i_knee = (uint16_t) (I_knee+3000);
		uint16_t i_ankle = (uint16_t) (I_ankle+3000);
		*a = i_knee;
		*b = i_ankle;
    setMotorCurrent(motor_2.deviceId, I_knee);      //踝关节电流指令
    setMotorCurrent(motor_1.deviceId, I_ankle);     //膝关降电流指令
}*/
void Impedance_control(JointparaTypeDef *Knee, JointparaTypeDef *Ankle, float *Posi_Vel)      //入口参数膝关节和踝关节的阻抗控制参数，两个结构体
{
    int I_ankle, I_knee, com_knee, com_ankle;
    //knee摩擦力补偿
    if (*(Posi_Vel + 1) > 1)
        com_knee = 20;
    else if (*(Posi_Vel + 1) < -1)
        com_knee = -20;
    else
        com_knee = 0;
    //ankle摩擦力补偿
    if (*(Posi_Vel + 3) > 1.5)
        com_ankle = 20;
    else if (*(Posi_Vel + 3) < -1.5)
        com_ankle = -20;
    else
        com_ankle = 0;

    I_knee = (int) (Knee->k * (Knee->q_d - (*Posi_Vel)) + Knee->b * (Knee->qv_d - (*(Posi_Vel + 1))) +
                    com_knee);//+com_knee;              //计算膝关节电流+com_knee
		
		
		//I_knee & 0xFFFF //低16
		//I_knee & 0xFFFF0000  //高16   
		// uint16_t
    I_ankle = (int) (Ankle->k * (Ankle->q_d - (*(Posi_Vel + 2))) + Ankle->b * (Ankle->qv_d - *(Posi_Vel + 3)) +
                     com_ankle);//+com_ankle;      //计算踝关节电流
		//if(I_ankle < 0)  a |= 0b1(15*0)
		//uint16_t  a =  (uint16_t)I_ankle;
		//

    if (I_knee >= 3000)         //判断输出电流是否过大
    {
        I_knee = 3000;
    }
    if (I_knee <= -3000)         //判断输出电流是否过大
    {
        I_knee = -3000;
    }
    if (I_ankle >= 3000)         //判断输出电流是否过大
    {
        I_ankle = 3000;
    }
    if (I_ankle <= -3000)         //判断输出电流是否过大
    {
        I_ankle = -3000;
    }
    setMotorCurrent(motor_2.deviceId, I_knee);      //踝关节电流指令
    setMotorCurrent(motor_1.deviceId, I_ankle);     //膝关降电流指令
}