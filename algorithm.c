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
    ����ELMO������������λ���ٶ�
    ʹ�÷���    int *p��
               p=get_Positon_Velocity��
		       knee_position=*(get_Positon_Velocity);
			   knee_velocity=*(get_Positon_Velocity+1);
		��������Ϣ��
		            ϥ�ؽ�   7500
					�׹ؽ�   2500
*************************************************************************/
float *get_Positon_Velocity(void) {
    //static float Posi_Vel[4];                                        //�ؽ����ݶ�ȡ
    static float Posi_Vel[8];
	  motorSendSync();
//      HAL_Delay(1);	//����ͬ��֡
    Posi_Vel[0] = (float) motor_2.positionActual / 8192.0 / 50.0 * 360.0;              //knee position    7500
    Posi_Vel[1] = (float) motor_2.velocityActual / 8192.0 / 50.0 * 360.0;              //knee velocity    7500
    Posi_Vel[2] = (float) motor_1.positionActual / 8192.0 / 50.0 * 360.0;              //ankle position   2500
    Posi_Vel[3] = (float) motor_1.velocityActual / 8192.0 / 50.0 * 360.0;              //ankle velocity   2500
    Posi_Vel[4] = (float) motor_2.currentActual;
	  Posi_Vel[5] = (float) motor_2.torqueActual;
	  Posi_Vel[6] = (float) motor_1.currentActual;
	  Posi_Vel[7] = (float) motor_1.torqueActual;
	  return Posi_Vel;                                                 //���������׵�ַ
}


/***********************************************************************                                                                   2
    void Impedance_control(JointparaTypeDef *Knee,JointparaTypeDef *Ankle)
    ���ϥ�ؽں��׹ؽڵ��迹���ƣ�δ����ʱ
    �迹���Ʒ��̣�
                   k*(angle-angle_equ)+b*angle_velocity
             ���У�
                   k-----------------------�ն�ϵ��
                   b-----------------------����ϵ��
                   angle-------------------��ǰ�Ƕ�
                   angle_equ---------------ƽ��Ƕ�
                   angle_velocity--------��ǰ���ٶ�
    ʹ�÷�����  Impedance_control(JointparaTypeDef *Knee,JointparaTypeDef *Ankle)
*************************************************************************/
/*void Impedance_control(JointparaTypeDef *Knee, JointparaTypeDef *Ankle, float *Posi_Vel, uint16_t *a, uint16_t *b)      //��ڲ���ϥ�ؽں��׹ؽڵ��迹���Ʋ����������ṹ��
{
    int I_ankle, I_knee, com_knee, com_ankle;
    //kneeĦ��������
    if (*(Posi_Vel + 1) > 1)
        com_knee = 20;
    else if (*(Posi_Vel + 1) < -1)
        com_knee = -20;
    else
        com_knee = 0;
    //ankleĦ��������
    if (*(Posi_Vel + 3) > 1.5)
        com_ankle = 20;
    else if (*(Posi_Vel + 3) < -1.5)
        com_ankle = -20;
    else
        com_ankle = 0;

    I_knee = (int) (Knee->k * (Knee->q_d - (*Posi_Vel)) + Knee->b * (Knee->qv_d - (*(Posi_Vel + 1))) +
                    com_knee);//+com_knee;              //����ϥ�ؽڵ���+com_knee
		
		
		//I_knee & 0xFFFF //��16
		//I_knee & 0xFFFF0000  //��16   
		// uint16_t
    I_ankle = (int) (Ankle->k * (Ankle->q_d - (*(Posi_Vel + 2))) + Ankle->b * (Ankle->qv_d - *(Posi_Vel + 3)) +
                     com_ankle);//+com_ankle;      //�����׹ؽڵ���
		//if(I_ankle < 0)  a |= 0b1(15*0)
		//uint16_t  a =  (uint16_t)I_ankle;
		//

    if (I_knee >= 3000)         //�ж���������Ƿ����
    {
        I_knee = 3000;
    }
    if (I_knee <= -3000)         //�ж���������Ƿ����
    {
        I_knee = -3000;
    }
    if (I_ankle >= 3000)         //�ж���������Ƿ����
    {
        I_ankle = 3000;
    }
    if (I_ankle <= -3000)         //�ж���������Ƿ����
    {
        I_ankle = -3000;
    }
		uint16_t i_knee = (uint16_t) (I_knee+3000);
		uint16_t i_ankle = (uint16_t) (I_ankle+3000);
		*a = i_knee;
		*b = i_ankle;
    setMotorCurrent(motor_2.deviceId, I_knee);      //�׹ؽڵ���ָ��
    setMotorCurrent(motor_1.deviceId, I_ankle);     //ϥ�ؽ�����ָ��
}*/
void Impedance_control(JointparaTypeDef *Knee, JointparaTypeDef *Ankle, float *Posi_Vel)      //��ڲ���ϥ�ؽں��׹ؽڵ��迹���Ʋ����������ṹ��
{
    int I_ankle, I_knee, com_knee, com_ankle;
    //kneeĦ��������
    if (*(Posi_Vel + 1) > 1)
        com_knee = 20;
    else if (*(Posi_Vel + 1) < -1)
        com_knee = -20;
    else
        com_knee = 0;
    //ankleĦ��������
    if (*(Posi_Vel + 3) > 1.5)
        com_ankle = 20;
    else if (*(Posi_Vel + 3) < -1.5)
        com_ankle = -20;
    else
        com_ankle = 0;

    I_knee = (int) (Knee->k * (Knee->q_d - (*Posi_Vel)) + Knee->b * (Knee->qv_d - (*(Posi_Vel + 1))) +
                    com_knee);//+com_knee;              //����ϥ�ؽڵ���+com_knee
		
		
		//I_knee & 0xFFFF //��16
		//I_knee & 0xFFFF0000  //��16   
		// uint16_t
    I_ankle = (int) (Ankle->k * (Ankle->q_d - (*(Posi_Vel + 2))) + Ankle->b * (Ankle->qv_d - *(Posi_Vel + 3)) +
                     com_ankle);//+com_ankle;      //�����׹ؽڵ���
		//if(I_ankle < 0)  a |= 0b1(15*0)
		//uint16_t  a =  (uint16_t)I_ankle;
		//

    if (I_knee >= 3000)         //�ж���������Ƿ����
    {
        I_knee = 3000;
    }
    if (I_knee <= -3000)         //�ж���������Ƿ����
    {
        I_knee = -3000;
    }
    if (I_ankle >= 3000)         //�ж���������Ƿ����
    {
        I_ankle = 3000;
    }
    if (I_ankle <= -3000)         //�ж���������Ƿ����
    {
        I_ankle = -3000;
    }
    setMotorCurrent(motor_2.deviceId, I_knee);      //�׹ؽڵ���ָ��
    setMotorCurrent(motor_1.deviceId, I_ankle);     //ϥ�ؽ�����ָ��
}