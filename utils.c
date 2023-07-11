#include "string.h"
#include "utils.h"
#include "ads1256_spi.h"
#include "algorithm.h"
#include "uart_debug.h"
#include "arm_math.h"
#include "stdlib.h"

float *pos_vel;      //joint angle and angular velocity: enum_q_knee, dq_knee, enum_q_ankle, dq_ankle
uint16_t state_vec[8]; // pos_vel
//uint16_t state_vec[6] = {0, 0, 0, 0, 0, 0}; // pos_vel
float angle_paras[8] = {1, 0, -1, 0, 40, 5, 50, 6};//angle parameters of knee and ankle: knee_paras.q_d, knee_paras.qv_d, ankle_paras.q_d, ankle_paras.qv_d
float impedance_paras[8] = {1, 0, 180, 0.6, -1, 0, 120, 0.8};//impedance parameters of knee and ankle: knee_paras.q_d, knee_paras.qv_d, knee_paras.k, knee_paras.b,
// ankle_paras.q_d, ankle_paras.qv_d, ankle_paras.k, ankle_paras.b
float k_float_2_int = 100;
float b_float_2_int = 30000;
uint16_t *a;
uint16_t *b;

JointparaTypeDef knee_paras, ankle_paras;

void init_states(){
    size_t n = sizeof(state_vec) / sizeof(uint16_t);
    for (int i = 0; i < n; i++) {
        state_vec[i] = 0;
    }
    for (int i = 0; i < 4; i++) {
        *(pos_vel+i) = 0;
    }
    update_impedance_paras();
}

void update_states()
{
    pos_vel = get_Positon_Velocity();   //read joint angle and angular velocity: enum_q_knee, dq_knee, enum_q_ankle, dq_ankle
    for (int r = 0; r < 8; r++) {
        state_vec[r] =  (uint16_t) (*(pos_vel+r) * k_float_2_int + b_float_2_int);
    }
//    state_vec[0] =  (uint16_t) (knee_paras.q_d * k_float_2_int + b_float_2_int);
//    state_vec[1] =  (uint16_t) (knee_paras.qv_d * k_float_2_int + b_float_2_int);
//    state_vec[2] =  (uint16_t) (ankle_paras.q_d * k_float_2_int + b_float_2_int);
//    state_vec[3] =  (uint16_t) (ankle_paras.qv_d * k_float_2_int + b_float_2_int);
}


void printf_states(){
    update_states();
    printf_time_ms();
    size_t n = sizeof(state_vec) / sizeof(float);
    for (int i = 0; i < n; i++) {
        debugPrintPending("%.2f,", state_vec[i]);
    }
    debugPrintPending("\r\n");
}

void update_impedance_paras() {
    knee_paras.q_d  =impedance_paras[0];
    knee_paras.qv_d =impedance_paras[1];
    knee_paras.k    =impedance_paras[2];
    knee_paras.b    =impedance_paras[3];
    ankle_paras.q_d =impedance_paras[4];
    ankle_paras.qv_d=impedance_paras[5];
    ankle_paras.k   =impedance_paras[6];
    ankle_paras.b   =impedance_paras[7];
}

void update_desired_angle_and_velocity(uint16_t *q_qv_d_100_vec) {
		/*int j=0;
    for (int i = 0; i < 4; i++) {
			if(*(q_qv_d_100_vec+(4+j))!=0||*(q_qv_d_100_vec+(5+j))!=0){
				if(i>1){
						j=2;
				}
        k_float_2_int = *(q_qv_d_100_vec+(4+j));
				b_float_2_int = *(q_qv_d_100_vec+(5+j));//更新K和B,发送对于一组K,B一个不为0认为有效，进行更新
			}else{ 
				k_float_2_int = 100;
			  k_float_2_int = 30000;
			}//反之不更新，采用原固定值。
				angle_paras[i] = (((float) *(q_qv_d_100_vec+i)) - b_float_2_int) / k_float_2_int;
    }*/
		for (int i = 0; i < 8; i++){
			angle_paras[i] = (((float) *(q_qv_d_100_vec+i)) - b_float_2_int) / k_float_2_int;
		}
    knee_paras.q_d  =angle_paras[0];
    knee_paras.qv_d =angle_paras[1];
    ankle_paras.q_d =angle_paras[2];
    ankle_paras.qv_d=angle_paras[3];
		knee_paras.k  =angle_paras[4];
    knee_paras.b =angle_paras[5];
    ankle_paras.k =angle_paras[6];
    ankle_paras.b=angle_paras[7];
}

void printf_time_ms(){
    int time_ms = HAL_GetTick();
    time_ms = time_ms % 1000;
    debugPrintPending("Time: %d ms,", time_ms);
}


void update_parameter(uint16_t *q_qv_d_100_vec){

}//更新参数

uint16_t *joint_impedance_control(uint16_t *q_qv_d_100_vec) {
    update_states();
    update_desired_angle_and_velocity(q_qv_d_100_vec);
    Impedance_control(&knee_paras, &ankle_paras, pos_vel);
	  //Impedance_control(&knee_paras, &ankle_paras, pos_vel, a, b);
	  //state_vec[4] = *a;
	  //state_vec[5] = *b;
    update_states();
    return state_vec;
}


int compare_floats(const void *a, const void *b) {
    return ( *(float *)a - * (float *)b);
}

float calc_median(float *signal, int size) {
    qsort(signal, size, sizeof(float), compare_floats);
    int idx = (int)((size + 1)/2 - 1);
    return signal[idx];
}


float calc_min_val(float *signal, int size) {
    float32_t min_val = 0.0;
    uint32_t min_idx = 0;
    arm_min_f32(signal,size, &min_val, &min_idx);
    return min_val;
}

float calc_max_val(float *signal, int size) {
    float32_t max_val = 0.0;
    uint32_t max_idx = 0;
    arm_max_f32(signal,size, &max_val, &max_idx);
    return max_val;
}