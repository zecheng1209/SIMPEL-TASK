#include "Run.h"
#include "usb_trans.h"
#include "usbd_cdc_if.h"
#include "PID.h"
#include "math.h"
extern uint8_t ready;

Joint_t Joint[5];
int16_t can_buf[4] = {0};

uint8_t enable_Joint[5] = {1,1,1,1,1};//1,1,1,1,1

TaskHandle_t Motor_Drive_Handle;

RobStride_t rs02={.hcan=&hcan2,.motor_id=0x03,.type= RobStride_02, };//注意02为03id
RobStride_t rs03={.hcan=&hcan2,.motor_id=0x02,.type= RobStride_03, };
//float rs03_torque =-0.0f,rs03_rad=-0.2f;

float M0 = 7.0f; //rs03水平位置前馈
float rs02_torque=0.0f,rs02_rad=0.3f;//初始角
float rs03_torque=5.0f,rs03_rad=4.15f;//初始角
///float angle = 0.0f;//目标角度，角度制



//      rs03_rad = angle / 57.3f+rs03_rad;

//float rs03_kp=170.0f,rs03_kd=34.0f,rs03_omega=0.2f;   //MotionControl
PID rs02_vel_pid;  // 速度环PID
PID rs02_pos_pid;  // 位置环PID
PID rs03_vel_pid;  // 速度环PID
PID rs03_pos_pid;  // 位置环PID

//////typedef enum {
//////    RS03_MOVING,
//////    RS03_HOLD
//////} rs03_mode_t;

//////rs03_mode_t rs03_mode = RS03_MOVING;

void rs02_PID_Init() {
  
	rs02_vel_pid.Kp = 1.0f;
	rs02_vel_pid.Ki = 0.01f;
	rs02_vel_pid.Kd = 0.0f;
	rs02_vel_pid.limit = 4.0f;
	rs02_vel_pid.output_limit = 4.0f;
	
	
	rs02_pos_pid.Kp = 5.0f;
	rs02_pos_pid.Ki = 0.0f;
	rs02_pos_pid.Kd = 1.0f;
	rs02_pos_pid.limit = 5.0f;
	rs02_pos_pid.output_limit = 2.0f;
	

}

void rs03_PID_Init() {
  
	rs03_vel_pid.Kp = 11.0f;
	rs03_vel_pid.Ki = 0.01f;
	rs03_vel_pid.Kd = 0.0f;
	rs03_vel_pid.limit = 20.0f;
	rs03_vel_pid.output_limit = 35.0f;
	
	
	rs03_pos_pid.Kp = 75.0f;
	rs03_pos_pid.Ki = 0.0f;
	rs03_pos_pid.Kd = 1.0f;
	rs03_pos_pid.limit = 15.0f;
	rs03_pos_pid.output_limit = 5.0f;
	
}


void Motor_Drive(void *param)
{

	TickType_t Last_wake_time = xTaskGetTickCount();
	

	rs02_PID_Init();
	rs03_PID_Init();
	RobStrideResetAngle(&rs03);
  ///////////////RobStrideResetAngle(&rs03);//重置角度

	for(;;)
	{
      float curr_rad02 = rs02.state.rad;
			float curr_omega02 = rs02.state.omega;

			float curr_rad03 = rs03.state.rad;
			float curr_omega03 = rs03.state.omega;
		
		  float M1 = M0 * cos(rs03.state.rad- 4.15);//rs03初始水平的角度
		  
//////		 if (rs03_mode == RS03_MOVING)
//////		{
//////			if (fabs(curr_rad - rs03_rad) < 0.007f &&
//////        fabs(curr_omega) < 0.01f)
//////			{
//////        rs03_mode = RS03_HOLD;
//////			}
		

	  	PID_Control(curr_rad02, rs02_rad, &rs02_pos_pid);
			PID_Control(curr_rad03, rs03_rad, &rs03_pos_pid);
      float target_omega02 = rs02_pos_pid.pid_out;
			float target_omega03 = rs03_pos_pid.pid_out;
			PID_Control(curr_omega02, 0 , &rs02_vel_pid);	//target_omega02
			PID_Control(curr_omega03, 0 , &rs03_vel_pid);	//target_omega03		
      rs02_torque = 0;
			rs03_torque = 0;
///	    rs03_torque = M1+rs03_vel_pid.pid_out;
		//	rs03_torque = 0;
//////		}	
//////		else
//////		{
//////       if (fabs(curr_rad - rs03_rad) > 0.02f)
//////       {
//////         rs03_mode = RS03_MOVING;
//////       }
//////			float K_hold = 80.0f;   
//////			float tau_hold = K_hold * (rs03_rad - curr_rad);

//////			rs03_torque = M1 + tau_hold;
//////		}
//////		
		
		
		
		
		// for(uint8_t i = 0; i < 3; i++)
		// {
		// 	PID_Control(Joint[i].Rs_motor.state.rad, Joint[i].exp_rad + Joint[i].pos_offset, &Joint[i].pos_pid);
		// 	PID_Control(Joint[i].Rs_motor.state.omega, Joint[i].pos_pid.pid_out + Joint[i].exp_omega, &Joint[i].vel_pid);
//		 	RobStrideTorqueControl(&Joint[i].Rs_motor, Joint[i].vel_pid.pid_out * enable_Joint[i]);
//		 }
		// vTaskDelay(1);
		// PID_Control(Joint[3].Rs_motor.state.rad, 	Joint[3].exp_rad + Joint[3].pos_offset, &Joint[3].pos_pid);
		// PID_Control(Joint[3].Rs_motor.state.omega, Joint[3].pos_pid.pid_out + Joint[3].exp_omega, &Joint[3].vel_pid);
       RobStrideTorqueControl(&rs03,rs03_torque);
			 RobStrideTorqueControl(&rs02,rs02_torque);
  //   RobStrideMotionControl(&rs03, 0x02, rs03_torque, rs03_rad, rs03_omega, rs03_kp, rs03_kd);

		// PID_Control(Joint[4].RM_motor.actual_pos, Joint[4].exp_rad - Joint[4].pos_offset, &Joint[4].RM_motor.pos_pid);
		// PID_Control(Joint[4].RM_motor.motor.Speed, Joint[4].RM_motor.pos_pid.pid_out, &Joint[4].RM_motor.vel_pid);
		// can_buf[0] = Joint[4].RM_motor.vel_pid.pid_out * enable_Joint[4];
		// MotorSend(&hcan2 ,0x200, can_buf);
		
		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(2));
	}
}

Arm_t arm_t;
TaskHandle_t MotorSendTask_Handle;

SemaphoreHandle_t cdc_recv_semphr;
Arm_t arm_Rec_t;
uint16_t cur_recv_size;

void CDC_Recv_Cb(uint8_t *src, uint16_t size)
{
	if(!ready)
		return;
	cur_recv_size=size;
	if(((Arm_t *)src )->pack_type == 0x01)
	{
		memcpy(&arm_Rec_t, src, sizeof(arm_Rec_t));
		  
		  
		  
		  
		  
			Joint[0].exp_rad = arm_Rec_t.joints[0].rad;
			Joint[0].exp_omega = arm_Rec_t.joints[0].omega;
			Joint[0].exp_torque = arm_Rec_t.joints[0].torque;
			
			Joint[1].exp_rad = arm_Rec_t.joints[1].rad;
			Joint[1].exp_omega = arm_Rec_t.joints[1].omega;
			Joint[1].exp_torque = arm_Rec_t.joints[1].torque;
			
			Joint[2].exp_rad = arm_Rec_t.joints[2].rad * Joint[2].inv_motor;
			Joint[2].exp_omega = arm_Rec_t.joints[2].omega * Joint[2].inv_motor;
			Joint[2].exp_torque = arm_Rec_t.joints[2].torque * Joint[2].inv_motor;
			
			Joint[3].exp_rad = arm_Rec_t.joints[3].rad * Joint[3].inv_motor;
			Joint[3].exp_omega = arm_Rec_t.joints[3].omega * Joint[3].inv_motor;
			Joint[3].exp_torque = arm_Rec_t.joints[3].torque * Joint[3].inv_motor;
			
			Joint[4].exp_rad =( arm_Rec_t.joints[4].rad / 6.28319f * 36.0f * 8192.0f);
			Joint[4].exp_omega = arm_Rec_t.joints[4].omega;
			Joint[4].exp_torque = arm_Rec_t.joints[4].torque;
	}
}

void MotorSendTask(void *param)// 将电机的数据发送到PC上
{
	TickType_t Last_wake_time = xTaskGetTickCount();
	USB_CDC_Init(CDC_Recv_Cb, NULL, NULL);
	arm_t.pack_type = 1;
	
	for(;;)
	{
		for (uint8_t i = 0; i < 4; i++)
		{
			arm_t.joints[i].rad = (Joint[i].Rs_motor.state.rad -  Joint[i].pos_offset) * Joint[i].inv_motor;
			arm_t.joints[i].omega = Joint[i].Rs_motor.state.omega;
			arm_t.joints[i].torque = Joint[i].Rs_motor.state.torque;
		}
		
		arm_t.joints[4].rad =(((Joint[4].RM_motor.actual_pos + Joint[4].pos_offset)/ 8192.0f / 36.0f) * 2.0f * 3.1415926f) * Joint[4].inv_motor;
		arm_t.joints[4].omega = Joint[4].RM_motor.motor.Speed / 36.0f * 3.1415926f /30.0f;
		
		CDC_Transmit_FS((uint8_t*)&arm_t, sizeof(arm_t));
		vTaskDelayUntil(&Last_wake_time, pdMS_TO_TICKS(20));
	}
}

uint8_t count = 0; 
TaskHandle_t MotorRecTask_Handle;
void MotorRecTask(void *param)// 从PC接收电机的期望值
{
	TickType_t last_wake_time = xTaskGetTickCount();

	cdc_recv_semphr = xSemaphoreCreateBinary();
  xSemaphoreTake(cdc_recv_semphr, 0);

	for (;;)
	{
		if(xSemaphoreTake(cdc_recv_semphr, pdMS_TO_TICKS(200)) == pdTRUE)
		{
			count ++;
			
		}
	}
} 
uint8_t buf[8];
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan)
{

	{
		
		uint32_t ID = CAN_Receive_DataFrame(hcan, buf);
		
		RobStrideRecv_Handle(&rs03, &hcan2, ID, buf);
		RobStrideRecv_Handle(&rs02, &hcan2, ID, buf);
//		RobStrideRecv_Handle(&Joint[1].Rs_motor, &hcan1, ID, buf);
//		RobStrideRecv_Handle(&Joint[2].Rs_motor, &hcan1, ID, buf);
//		RobStrideRecv_Handle(&Joint[3].Rs_motor, &hcan1, ID, buf);
	}
}
		uint16_t ID;
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	//if (hcan->Instance == CAN2)
	{
		uint8_t buf[8];
		ID = CAN_Receive_DataFrame(&hcan2, buf);
//		Motor2006Recv(&Joint[4].RM_motor, &hcan2, ID, buf);
	}
}


