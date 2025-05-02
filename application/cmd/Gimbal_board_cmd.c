#include "Gimbal_board_cmd.h"
#include "Task_Init.h"
#include "tim.h"
// module
#include "remote.h"
#include "pub_sub.h"
 // bsp
#include "bsp_dwt.h"
#include "can_comm.h"
#include "CANDrive.h"
#include "ins_task.h"

//extern Gimbal_upload_t gimbal_feedback_data;
static CANCommInstance *can_cmd_comm;
static uint16_t cnt = 0;
float Mouse_Sensitivity = 0.25f;
// cmd的private函数
void soft_rest(void);
void stop_mode_update(void);                //机器人停止模式更新函数
void remote_mode_update(void);              //机器人遥控器模式更新函数
void mouse_key_mode_update(void);           //机器人键鼠模式更新函数
void send_cmd_and_data(void);               //发布指令和板间通信
void mousekey_GimbalChassis_default(void);  //底盘和云台的默认状态

// 其他功能函数
static float   get_offset_angle(short init_forward, short now_encoder);  // 获取云台朝向与底盘正前的夹角
static void    Gimbal_Refreeget(CANInstance *instance);
static	Robot_Status_e robot_state;                       // 机器人整体工作状态
static	Robot_Status_e last_robot_state;                  // 机器人整体工作状态
/* 外设  todo: 添加蜂鸣器 PC虚拟串口 */
static RC_Ctl_t *rc_data;   // 遥控器数据指针
static attitude_t *ins;

   /* 云台部分 */
static Publisher_t *gimbal_cmd_pub;            // 云台控制消息发布者
static Subscriber_t *gimbal_upload_sub;        // 云台反馈信息订阅者
static Gimbal_ctrl_cmd_t gimbal_control;             // 传递给云台的控制信息
static Gimbal_upload_t   gimbal_upload_data;   // 从云台获取的反馈信息
    
   /* 发射部分 */
static Publisher_t *shoot_cmd_pub;           // 发射控制消息发布者
static Subscriber_t *shoot_upload_sub; 	     // 发射反馈信息订阅者
static Shoot_ctrl_cmd_t shoot_control;       // 传递给发射的控制信息
static Shoot_upload_t   shoot_upload_data;   // 从发射获取的反馈信息
  /* PC接收部分 */
    
	/* 双板通信 */
static	Gimbal_board_send_t    send_data;
static	Gimbal_board_send_UI_t send_UI;
static  Chassis_board_send_t  *receive_data;
static  Gimbal_data_t 		   Gimbal_data_send;
static  Gimbal_action_t        Communication_Action_Tx;
    /* 自瞄模式 */
Aim_Action_e Aim_Action = AIM_STOP;   // 手动自瞄开关
extern Aim_Data_t Aim_Ref;
/* 舵机云台    倍镜       图传*/
//ServoInstance *LES_Servo, *Image_Servo;
void Gimbal_Cmd_Init()
{	

	// 定义发布者订阅者
	gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_ctrl_cmd_t) );
	gimbal_upload_sub = SubRegister("gimbal_upload", sizeof(Gimbal_upload_t) );
	shoot_cmd_pub = PubRegister("shoot_cmd", sizeof(Shoot_ctrl_cmd_t) );
	shoot_upload_sub = SubRegister("shoot_upload", sizeof(Shoot_upload_t));

	/* 上下板通信 */
	CANComm_Init_Config_s comm_conf = {
		.can_config = {
			.can_handle = &hcan2,
			.tx_id = 0x110,
			.rx_id = 0x101,
			.can_module_callback = Gimbal_Refreeget,
		},
		.recv_data_len = sizeof(Chassis_board_send_t),
		.send_data_len = sizeof(Gimbal_board_send_t),
		.dog_config = {
			.Max_num = 10,
			.dog_name = "UPBOARD",
		},
	};
	can_cmd_comm = CANCommInit(comm_conf);
    
	robot_state = ROBOT_READY; //     //初始化为ready
    last_robot_state = ROBOT_STOP;
		
	receive_data = RMLIB_MALLOC(sizeof(Chassis_board_send_t));
    rc_data = get_remote_control_point();
	ins     = INS_Init();

	/** 初始化舵机 **/
//	Servo_Init_Config_s les_config = {
//		.servo_type = PWM_Servo,
//	    .pwm_init_config = {
//		    .htim = &htim1,
//			.channel = TIM_CHANNEL_1,
//			.period = 0.02f,
//			.dutyratio = 0.50f,
//		},
//	}, 
//	image_config = {
//		.servo_type = PWM_Servo,
//	    .pwm_init_config = {
//		    .htim = &htim1,
//			.channel = TIM_CHANNEL_2,
//			.period = 0.02f,
//			.dutyratio = 0.50f,
//		},
//	};
////	LES_Servo = ServoInit(&les_config);
//	Image_Servo = ServoInit(&image_config);
}

/**    **/
float slope_thete = 0.0;
void Gimbal_board_CMD_Update()
{    
	static uint16_t tim; // 用于云台归中	
    if (can_cmd_comm->Dog->state != Dog_Online) {
		 gimbal_control.rotate_feedforward = 0;  // 底盘小陀螺前馈
		  receive_data->chassis_slop = 0;
	} else {
		 gimbal_control.rotate_feedforward = (float )receive_data->chassis_gyro/200.0f;
		 deadline_limit(gimbal_control.rotate_feedforward, 0.05f);
		 shoot_control.heat_limit_remain = receive_data->shoot_referee_data.heat_limit_remain;
	}
	slope_thete = receive_data->chassis_slop/200.0f;
	// 判断云台模块  todo : 与PC通信
	SubGetMessage(gimbal_upload_sub, (void *)&gimbal_upload_data);
	  /* 发射部分 */
	  // 获取实际弹速
	SubGetMessage(shoot_upload_sub, (void* )&shoot_upload_data);
	  /* 数据通信丢失处理 */
    if (shoot_upload_data.shoot_status != Device_Online)
		shoot_control.mode = SHOOT_STOP;
    if (gimbal_upload_data.gimbal_status != Device_Online)
		gimbal_control.mode = GIMBAL_ZERO_FORCE;
	  
	// 遥控器判断
	if ( rc_data->Remote_dog->state != Dog_Online)
	    robot_state = ROBOT_STOP;
	if ( rc_data->Remote_dog->state == Dog_Online && robot_state == ROBOT_STOP)  // 恢复任务
		    robot_state = ROBOT_READY;
	// 遥控器离线模式
	 if(robot_state == ROBOT_STOP)
		 stop_mode_update();
     else if(robot_state == ROBOT_READY) {
		 if(gimbal_upload_data.gimbal_status == Device_Online) 
		  {
			tim++;
			if (tim < 30)
			{
	#if   Yaw_Mid_Right < Yaw_Mid_Left
			if ( (gimbal_upload_data.yaw_encorder <= Yaw_Mid_Left) && (gimbal_upload_data.yaw_encorder >= Yaw_Mid_Right) )
	#elif Yaw_Mid_Right > Yaw_Mid_Left
			if ( (gimbal_upload_data.yaw_encorder <= Yaw_Mid_Left) || (gimbal_upload_data.yaw_encorder >= Yaw_Mid_Right) )
	#endif
				gimbal_control.Mid_mode = FRONT;
			else
				gimbal_control.Mid_mode = BACK;
			gimbal_control.mode = GIMBAL_ZERO_FORCE;
			gimbal_control.Mech_Ref.Pitch = 0;
			} else if (tim < 1000 ) {
				gimbal_control.mode = GIMBAL_MIDDLE;  // 云台归中 归中时先使用机械模式  后续考虑使用陀螺仪
			} else {
			    tim = 0;
			    robot_state = Robot_RUNNING;
			    gimbal_control.mode = GIMBAL_GYRO_MODE; // 跟随模式
			    gimbal_control.Gyro_Ref.Yaw   = ins->ContinuousYaw;  /* 陀螺仪模式 */ 
			    gimbal_control.Gyro_Ref.Pitch = ins->Pitch;
				gimbal_control.Mech_Ref.Yaw = gimbal_upload_data.yaw_encorder + gimbal_upload_data.yaw_round * 8192;
		    }
	     }   
	 } else if(robot_state == Robot_RUNNING) {
		/* 向底盘发送角度差 */
		switch (rc_data->RemoteMode) {
			case REMOTE_INPUT:        	// 遥控器控制模式
				remote_mode_update(); send_data.Close_flag = 0; send_data.Shift_flag = 0; 
			break;
			case KEY_MOUSE_INPUT:
				mouse_key_mode_update(); send_data.Close_flag = 0; send_data.Shift_flag = 0; break;
			case STOP :    // 急停  todo : 修改为期望速度为零
				gimbal_control.mode  = GIMBAL_ZERO_FORCE;
				shoot_control.mode   = SHOOT_ZERO_FORCE;
				send_UI.chassis_mode = CHASSIS_ZERO_FORCE;
			    gimbal_control.Gyro_Ref.Yaw   = ins->ContinuousYaw;  /* 陀螺仪模式 */
			    gimbal_control.Gyro_Ref.Pitch = ins->Pitch;
			    send_data.Close_flag = 1;
				send_data.Shift_flag = 0;
			 break;
		}
	 }
	 if(send_UI.chassis_mode == CHASSIS_ZERO_FORCE) {
		  send_data.Close_flag = 1;
		  send_data.Shift_flag = 0;
	 }
	 /* 裁判系统UI数据 */
//	 send_data.gimbal_mode = gimbal_upload_data.mode;
//	 send_data.shoot_mode  = shoot_upload_data.mode;
//	 send_data.bullet_mode = shoot_upload_data.bullet_mode;
//	 send_data.now_robot_mode = robot_state;
	 
	 // 根据底盘模式进行计算底盘的旋转速度 获取云台补偿角度
	 gimbal_control.chassis_mode = send_UI.chassis_mode;
	 gimbal_control.shoot_mode = shoot_control.mode;
	// 发布数据
    PubPushMessage(shoot_cmd_pub, (void *)&shoot_control); 
    PubPushMessage(gimbal_cmd_pub, (void *)&gimbal_control);
	gimbal_control.last_mode = gimbal_control.mode;
	 /**  底盘信息发射  **/
#if CHASSIS_MOVE == 0
send_data.Close_flag = 1;
send_data.vx = 0;
send_data.vy = 0;
send_data.rotate = 0;
#endif
	CAN_Send_StdDataFrame(&hcan2, 0x110, (uint8_t *)&send_data);
    // 向底盘发送UI信息
	Gimbal_data_send.Offset_Angle = (int16_t)((gimbal_upload_data.yaw_encorder - Yaw_Mid_Front) / 1303.80f * 100);
	Gimbal_data_send.Pitch_angle = (int16_t)(-ins->Pitch * 100);
	Gimbal_data_send.Yaw_angle = (int16_t)(ins->ContinuousYaw * 100);
	Gimbal_data_send.Fric_Speed = (int16_t)(shoot_upload_data.fire_speed);
					
	Communication_Action_Tx.Gimbal_status.Pitch = (uint8_t)gimbal_upload_data.pitch_status;
	Communication_Action_Tx.Gimbal_status.Yaw   = (uint8_t)gimbal_upload_data.gimbal_status;
	
	switch(shoot_upload_data.mode) {
		case 0: case 2 :case SHOOT_STUCKING : Communication_Action_Tx.shoot_mode = 0;break;
		case SHOOT_AIM : Communication_Action_Tx.shoot_mode = 3;break;
		case SHOOT_READY : Communication_Action_Tx.shoot_mode = 1;break;
		case SHOOT_NOMAL : Communication_Action_Tx.shoot_mode = 2;break;
		default: Communication_Action_Tx.shoot_mode = 0;break;
	}
	
	switch(send_UI.chassis_mode) {
		case 0: Communication_Action_Tx.move_status = 0;break;
		case CHASSIS_FOLLOW: Communication_Action_Tx.move_status = 1;break;
		case CHASSIS_SPIN: Communication_Action_Tx.move_status = 2;break;
		
		default: Communication_Action_Tx.move_status = 0;break;
	}
	Communication_Action_Tx.vision_number = Aim_Ref.number;
	Communication_Action_Tx.vision_status = (uint8_t)Aim_Ref.usb_state;
	Communication_Action_Tx.shoot_status = (uint8_t)shoot_upload_data.shoot_status;
	Gimbal_data_send.Offset_Angle = (int16_t)((gimbal_upload_data.yaw_encorder - Yaw_Mid_Front) / 1303.80f  * 1000.0f);
	
	switch(cnt++)
	{
        case 0: CAN_Send_StdDataFrame(&hcan2, 0x120, (uint8_t *)&Communication_Action_Tx);break;
        case 4: CAN_Send_StdDataFrame(&hcan2, 0x130, (uint8_t *)&Gimbal_data_send);break;
		case 8: cnt = 0;break;
    }
}

/**  软件软重启  **/ 
void soft_rest()
{
    // 关闭所有中断
    __set_FAULTMASK(1); 
    // 复位
    NVIC_SystemReset();
}

/* 机器人停止模式 */
void stop_mode_update()
{
	// 关闭自身模块
	gimbal_control.mode = GIMBAL_ZERO_FORCE;
	shoot_control.mode  = SHOOT_ZERO_FORCE;
	// 关闭底盘模块
	send_data.Close_flag = 1;
}

// 机器人等级 用于适应不同功率
static  float Level_Gain, chassis_offset;
static  uint16_t SHOOTCHEAK_CNT = 0; // 底盘补偿角度
void remote_mode_update()
{
	static float limit_pitch_angle = 0.0f;
	int16_t left_right_ref = 0,forward_back_ref = 0;
	
		/* 云台前馈值 */
    gimbal_control.Feedback_Speed.Yaw = 0;
	gimbal_control.Feedback_Speed.Pitch = 0;

	/* 遥控器控制模式 */
	switch (rc_data->rc.s1) {
		case 1:
			if(ins->ins_dog->state == Dog_Online) {
				gimbal_control.mode  = GIMBAL_GYRO_MODE;
			    send_UI.chassis_mode = CHASSIS_FOLLOW;
			} else {
				gimbal_control.mode  = 0;
			    send_UI.chassis_mode = 0;
			}
		   shoot_control.mode = SHOOT_NOMAL;
           shoot_control.bullet_mode = BULLET_SINGLE;
		   shoot_control.fire_rate = 1;     // 设置弹频
		   Aim_Action = AIM_AUTO;
		break;
	    case 3:
			if(ins->ins_dog->state == Dog_Online) {
				gimbal_control.mode  = GIMBAL_GYRO_MODE;
			    send_UI.chassis_mode = CHASSIS_NORMAL;
			} else {
				gimbal_control.mode  = 0;
			    send_UI.chassis_mode = 0;
			}
	       shoot_control.mode = SHOOT_READY; 
           shoot_control.bullet_mode = BULLET_SINGLE;
		   shoot_control.fire_rate = 1;     // 设置弹频
		   Aim_Action = AIM_AID; 
			break;
	    case 2:
			if(ins->ins_dog->state == Dog_Online) {
				gimbal_control.mode  = GIMBAL_GYRO_MODE;
			    send_UI.chassis_mode = CHASSIS_FOLLOW;
			} else {
				gimbal_control.mode  = 0;
			    send_UI.chassis_mode = 0;
			}
		   shoot_control.mode = SHOOT_STOP;
		   Aim_Action = AIM_STOP;
		 break;
	}

	/* 陀螺仪模式 */
	if(gimbal_control.mode == GIMBAL_GYRO_MODE && Aim_Ref.auto_mode == auto_aim_off ) {
        gimbal_control.Gyro_Ref.Yaw -= rc_data->Key_CH[2] * DR16_Rocker_Yaw_Resolution * 180.0f / PI * Mouse_Sensitivity * 3;
        gimbal_control.Feedback_Speed.Yaw -= rc_data->Key_CH[2] * DR16_Rocker_Yaw_Resolution * 1000.0f * Mouse_Sensitivity * 1.5;
        gimbal_control.Gyro_Ref.Pitch -= rc_data->Key_CH[3] * DR16_Rocker_Pitch_Resolution * 180.0f / PI * Mouse_Sensitivity * 4.0f;
        gimbal_control.Feedback_Speed.Pitch = 0; //-= rc_data->Key_CH[3] * DR16_Rocker_Pitch_Resolution * 1000.0f * Mouse_Sensitivity * 5.0f;
//		if(gimbal_upload_data.yaw_encorder >= MCH_UP_limit || gimbal_upload_data.yaw_encorder <= MCH_DOWN_limit)
	        limit(gimbal_control.Gyro_Ref.Pitch, IMU_UP_limit+slope_thete, IMU_DOWN_limit+slope_thete);
//		else
//			limit(gimbal_control.Gyro_Ref.Pitch, IMU_UP_limit, IMU_DOWN_limit);
		 /* 如果底盘跟随很慢 及时切换零点 */
#if     Yaw_Mid_Right < Yaw_Mid_Left
        if ( (gimbal_upload_data.yaw_encorder <= Yaw_Mid_Left) && (gimbal_upload_data.yaw_encorder >= Yaw_Mid_Right)){
#elif   Yaw_Mid_Right > Yaw_Mid_Left
        if ( (gimbal_upload_data.yaw_encorder <= Yaw_Mid_Left) || (gimbal_upload_data.yaw_encorder >= Yaw_Mid_Right)){
#endif
			gimbal_control.Mid_mode = FRONT;
		} else
		   	gimbal_control.Mid_mode = BACK;
	} else {
		gimbal_control.Gyro_Ref.Yaw   = ins->ContinuousYaw;
		gimbal_control.Gyro_Ref.Pitch = ins->Pitch;//gimbal_upload_data.pitch_encorder;
	}
	if(shoot_control.mode == SHOOT_CHEAKOUT) {
		 SHOOTCHEAK_CNT++;
		shoot_control.mode = SHOOT_READY;
		if(SHOOTCHEAK_CNT >= 250)  {
		    shoot_control.mode = SHOOT_NOMAL;
			if(SHOOTCHEAK_CNT > 270)
		         SHOOTCHEAK_CNT = 0;
		}
	}
	 /** 归中模式控制 **/
	if(gimbal_control.mode == GIMBAL_MIDDLE || gimbal_control.mode == GIMBAL_MECH_MODE) {
		/* PITCH 轴角度控制 */
		gimbal_control.Mech_Ref.Pitch += rc_data->Key_CH[3] * DR16_Rocker_Pitch_Resolution * 180.0f / PI * Mouse_Sensitivity * 1.8f * 2;
		gimbal_control.Feedback_Speed.Pitch += rc_data->Key_CH[3] * DR16_Rocker_Pitch_Resolution * 1000.0f * Mouse_Sensitivity * 1.8f * 2;
		/*机械角度模式 YAW轴控制*/
		if(gimbal_control.mode == GIMBAL_MECH_MODE) {
			gimbal_control.Mech_Ref.Yaw -= rc_data->Key_CH[2] * DR16_Rocker_Yaw_Resolution * 180.0f / PI * Mouse_Sensitivity * 8192.0f / 360.0f * 2;
		    gimbal_control.Feedback_Speed.Yaw -= rc_data->Key_CH[2] * DR16_Rocker_Yaw_Resolution * 1000.0f * Mouse_Sensitivity * 8192.0f / 360.0f * 2;
		} else 
			gimbal_control.Mech_Ref.Yaw = gimbal_upload_data.yaw_encorder + gimbal_upload_data.yaw_round * 8192 * 2.0f;
		
		limit(gimbal_control.Mech_Ref.Pitch, MCH_UP_limit, MCH_DOWN_limit);
	} else {
		gimbal_control.Mech_Ref.Pitch = gimbal_upload_data.pitch_encorder;
		gimbal_control.Mech_Ref.Yaw = gimbal_upload_data.yaw_encorder + gimbal_upload_data.yaw_round * 8192;
	}
	/* 底盘参数控制 遥控器底盘功能调试*/
	send_UI.chassis_dispatch_mode = (uint8_t )chassis_dispatch_without_acc_limit; // 修改功率模式 调试用
	
    /* 速度上限与加减速规划控制 */
	if (send_UI.chassis_mode != CHASSIS_ZERO_FORCE) {
		 Level_Gain = 2.0f;
	    /* 获取底盘旋转速度 */
		if(send_UI.chassis_mode == CHASSIS_FOLLOW)
		    send_data.rotate = gimbal_upload_data.rotate_speed * 1;
		else
		    send_data.rotate = gimbal_upload_data.rotate_speed;
		if(send_UI.chassis_mode == CHASSIS_NORMAL && gimbal_control.mode == GIMBAL_GYRO_MODE) {
		     send_data.rotate  = (float )rc_data->Key_CH[0] * Level_Gain * 5500;
			 left_right_ref   = 0;
		} 
		else
			left_right_ref   = (float )rc_data->Key_CH[0] * Level_Gain * 1000;		
		forward_back_ref = (float )rc_data->Key_CH[1] * Level_Gain * 2000;
	}
	
	 /* 底盘补偿(小陀螺模式也能正常移动) */
	static float sin_theta, cos_theta; // 利用YAW电机回传角度 求解底盘相对于云台的角度
	chassis_offset = (gimbal_upload_data.yaw_encorder - Yaw_Mid_Front) / 1303.80f;
	sin_theta = arm_sin_f32(chassis_offset);
	cos_theta = arm_cos_f32(chassis_offset);
	
    send_data.vy  =  left_right_ref * cos_theta - forward_back_ref * sin_theta;  // left_right_ref 
	send_data.vx  =  left_right_ref * sin_theta + forward_back_ref * cos_theta;  // forward_back_ref
	
	/* 底盘平移前馈补偿 */
//	float sin_yaw_feedforward, cos_yaw_feedforward;
//	cos_yaw_feedforward = cosf(chassis_offset - receive_data.chassis_gyro * 0.10f);
//	sin_yaw_feedforward = sinf(chassis_offset - receive_data.chassis_gyro * 0.10f);
//	send_data.vy =  left_right_ref * cos_yaw_feedforward - forward_back_ref * sin_yaw_feedforward;
//	send_data.vx =  left_right_ref * sin_yaw_feedforward + forward_back_ref * cos_yaw_feedforward;

    /* 裁判系统返回热量 */
    shoot_control.heat_limit_remain = receive_data->shoot_referee_data.heat_limit_remain;  // 剩余热量
}


/**  键鼠控制  **/
static float mouse_key_shift = 1.0f, chassis_speed_K = 0.0f;
static int16_t CHASSIS_SPEED_CNT = 0;
int16_t gimbal_gyro_H, gimbal_gyro_L = 0;
void mouse_key_mode_update()
{
	static Chassis_mode_e last_chassis;
	static uint8_t mouse_r_flag = 0;
	static int16_t key_f_flag = 0, last_key_x= 0, last_key_c= 0, last_key_e = 0, last_key_q = 0, last_key_v = 0, struck_const = 0;
	static uint16_t soft_rest = 0;
	static float limit_pitch_angle = 0.0f;
	Gimbal_board_send_t *chassis = &send_data;
	Shoot_ctrl_cmd_t *shoot = &shoot_control;
	int16_t left_right_ref = 0, forward_back_ref = 0, normal_action = 0;
	
	/* Ctrl云台减速 */
	if(rc_data->key[KEY_PRESS].Ctrl) {
		Mouse_Sensitivity = 0.1f;
	} else Mouse_Sensitivity = 0.25f;
	
	/* Z键刷新UI */
	if(rc_data->key[KEY_PRESS].Z) {
		soft_rest ++;
		if(soft_rest >= 100)
			Communication_Action_Tx.Key = 1;
	} else {
		soft_rest = 0;
		Communication_Action_Tx.Key = 0;
	}

	/* 鼠标右键开启摩擦轮 发射控制 */
	if (rc_data->mouse.press_r && mouse_r_flag == 0) {
		if (shoot->mode ==  SHOOT_STOP)
			  shoot->mode = SHOOT_READY;
		 else
			 shoot->mode = SHOOT_STOP;
	   mouse_r_flag = 1;
	}
	
	if(rc_data->mouse.press_r == 0)
    	mouse_r_flag = 0;
	
	if(shoot->mode == SHOOT_READY || shoot->mode == SHOOT_NOMAL) {
		if(rc_data->mouse.press_l == 1) {
		   shoot->mode = SHOOT_NOMAL;
		   shoot->bullet_mode = BULLET_SINGLE;
		   shoot->fire_rate = 1;  // 设置弹频
		} else
		   	shoot->mode = SHOOT_READY;
	}  else shoot->mode = SHOOT_STOP;

	// R:底盘模式
	if(rc_data->key_count[KEY_PRESS][Key_R]) {
	  switch (rc_data->key_count[KEY_PRESS][Key_R] % 2) {
		case 0:
			if(ins->ins_dog->state == Dog_Online) {
				gimbal_control.mode  = GIMBAL_GYRO_MODE;
			    send_UI.chassis_mode = CHASSIS_SPIN;
			} else {
				gimbal_control.mode  = 0;
			    send_UI.chassis_mode = 0;
			}
			break;
		case 1:
				if(ins->ins_dog->state == Dog_Online) {
					gimbal_control.mode  = GIMBAL_GYRO_MODE;
					send_UI.chassis_mode = CHASSIS_FOLLOW;
				} else {
				   gimbal_upload_data.rotate_speed = 0;
				   gimbal_control.mode  = GIMBAL_MECH_MODE;
				   send_UI.chassis_mode = CHASSIS_NORMAL;
				}
			break;
		default :
			mousekey_GimbalChassis_default();
			break;
		}
	} else 	
		mousekey_GimbalChassis_default();
		
	 // C V：修改倍镜和图传位置
	switch (rc_data->key_count[KEY_PRESS][Key_V] % 2) {
	  case 0 :
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 2290);  //400  关    600开
		  break;
	  case 1:
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 2180);  //400  关    600开

		  break;
	  default:
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1, 2290);  //400  关    600开
		  break;
     }
	switch (rc_data->key_count[KEY_PRESS][Key_C] % 2) {
	  case 0 :
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 1800);  //400  关    600开
		  break;
	  case 1:
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 595);  //400  关    600开
		  break;
	  default:
		  __HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2, 1600);  //400  关    600开
		  break;
     }

	   // E Q与CTRL 连携键控制发射速度
	 if(rc_data->key_count[KEY_PRESS_WITH_CTRL][Key_E] != last_key_e&& rc_data->key_count[KEY_PRESS_WITH_CTRL][Key_E] != 0) {
		 shoot->speed_offset += 25;
		 last_key_e = rc_data->key_count[KEY_PRESS_WITH_CTRL][Key_E];
	 }
	 if(rc_data->key_count[KEY_PRESS_WITH_CTRL][Key_Q] != last_key_q&& rc_data->key_count[KEY_PRESS_WITH_CTRL][Key_Q] != 0) {
		 shoot->speed_offset -= 25;
		 last_key_q = rc_data->key_count[KEY_PRESS_WITH_CTRL][Key_Q];
	 }
	  
	  // X C 与CTRL 连携键控制运动速度
	 if(rc_data->key_count[KEY_PRESS_WITH_CTRL][Key_X] != last_key_x&& rc_data->key_count[KEY_PRESS_WITH_CTRL][Key_X] != 0) {
		 mouse_key_shift -= 0.1f;
		 last_key_x = rc_data->key_count[KEY_PRESS_WITH_CTRL][Key_X];
	 }
	 if(rc_data->key_count[KEY_PRESS_WITH_CTRL][Key_C] != last_key_c && rc_data->key_count[KEY_PRESS_WITH_CTRL][Key_C] != 0) {
		 mouse_key_shift += 0.1f;
	     last_key_c = rc_data->key_count[KEY_PRESS_WITH_CTRL][Key_C];
	 }
	 limit(mouse_key_shift, 2.5, 0.5);
	 
	 // Y 与CTRL 连携键手动退弹
	 if(rc_data->key_count[KEY_PRESS_WITH_CTRL][Key_V] != last_key_v && struck_const == 0) {
	     shoot->mode = SHOOT_STUCKING;
		 struck_const = 1;
	     last_key_v = rc_data->key_count[KEY_PRESS_WITH_CTRL][Key_V] ;
	 }
	 if(struck_const) {
		  struck_const ++;
		  shoot->mode = SHOOT_STUCKING;
	      if(struck_const >= 400)
			  struck_const =0;
	 }
	 
    /** 速度上限与加减速规划控制 **/
	if (send_UI.chassis_mode != CHASSIS_ZERO_FORCE) {
		Level_Gain = 2.0f;
	    /** 计算底盘旋转速度 **/
		send_UI.chassis_mode == CHASSIS_FOLLOW ? ( send_data.rotate = gimbal_upload_data.rotate_speed)
								 : ( send_data.rotate = gimbal_upload_data.rotate_speed * mouse_key_shift);
		/** 水平速度增益 **/ 
		if(rc_data->key[KEY_PRESS].W || rc_data->key[KEY_PRESS].S) {
			CHASSIS_SPEED_CNT++;
			if(CHASSIS_SPEED_CNT >= 200) {
			    chassis_speed_K += 0.1f;
				CHASSIS_SPEED_CNT = 0;
			}
	    } else {
			CHASSIS_SPEED_CNT = 0;
	        chassis_speed_K = 0.0f;
		}
		/** 平移速度计算 **/
		left_right_ref   = (float )( rc_data->key[KEY_PRESS].D - rc_data->key[KEY_PRESS].A ) * Level_Gain * 2000.0f * mouse_key_shift;
	    forward_back_ref = (float )( rc_data->key[KEY_PRESS].W - rc_data->key[KEY_PRESS].S ) * (Level_Gain+chassis_speed_K) * 2500.0f * mouse_key_shift;
		/** 小陀螺时平移速度削减 **/
	    if(send_UI.chassis_mode == CHASSIS_SPIN && abs(left_right_ref) + abs(forward_back_ref) > 600) {
			forward_back_ref *= 1.3f;
			left_right_ref *= 1.2f;
			send_data.rotate *= 0.5f;
		}
	}
	 /* 底盘补偿(小陀螺模式也能正常移动) */
	static float sin_theta, cos_theta; // 利用YAW电机回传角度 求解底盘相对于云台的角度
	chassis_offset = (gimbal_upload_data.yaw_encorder - Yaw_Mid_Front) / 1303.80f;
	sin_theta = arm_sin_f32(chassis_offset);
	cos_theta = arm_cos_f32(chassis_offset);
	
    send_data.vy  =  left_right_ref * cos_theta - forward_back_ref * sin_theta;  // left_right_ref 
	send_data.vx  =  left_right_ref * sin_theta + forward_back_ref * cos_theta;  // forward_back_ref
	
	/* 陀螺仪模式 */
	if(gimbal_control.mode == GIMBAL_GYRO_MODE && Aim_Ref.aim_action == auto_aim_off) {
        gimbal_control.Gyro_Ref.Yaw -= rc_data->Mouse_Ch[0] * Mouse_Sensitivity * 2.5;
        gimbal_control.Feedback_Speed.Yaw = -rc_data->Mouse_Ch[0] * Mouse_Sensitivity*2.5f;
		gimbal_control.Gyro_Ref.Pitch -= rc_data->Mouse_Ch[1] * Mouse_Sensitivity * 1.0f;
        gimbal_control.Feedback_Speed.Pitch = -rc_data->Mouse_Ch[0] * Mouse_Sensitivity*1.0f;
		if(gimbal_upload_data.pitch_encorder >= MCH_UP_limit || gimbal_upload_data.pitch_encorder <= MCH_DOWN_limit)
	        limit(gimbal_control.Gyro_Ref.Pitch, IMU_UP_limit+slope_thete, IMU_DOWN_limit+slope_thete);
		else
	        limit(gimbal_control.Gyro_Ref.Pitch, IMU_UP_limit, IMU_DOWN_limit);
		
		 /* 如果底盘跟随很慢 及时切换零点 */
#if     Yaw_Mid_Right < Yaw_Mid_Left
        if((gimbal_upload_data.yaw_encorder <= Yaw_Mid_Left) && (gimbal_upload_data.yaw_encorder >= Yaw_Mid_Right)){
#elif   Yaw_Mid_Right > Yaw_Mid_Left
        if((gimbal_upload_data.yaw_encorder <= Yaw_Mid_Left) || (gimbal_upload_data.yaw_encorder >= Yaw_Mid_Right)){
#endif
			gimbal_control.Mid_mode = FRONT;
		} else
		   	gimbal_control.Mid_mode = BACK;
	} else {
		gimbal_control.Gyro_Ref.Yaw   = ins->ContinuousYaw;
		gimbal_control.Gyro_Ref.Pitch = ins->Pitch;
	}	

	 /** 归中模式控制 **/
	if(gimbal_control.mode == GIMBAL_MIDDLE || gimbal_control.mode == GIMBAL_MECH_MODE) {
		/* PITCH 轴角度控制 */
		gimbal_control.Mech_Ref.Pitch += rc_data->Mouse_Ch[1] * DR16_Rocker_Pitch_Resolution * 180.0f / PI * Mouse_Sensitivity * 5.0f;
		gimbal_control.Feedback_Speed.Pitch = 0; // -= rc_data->Mouse_Ch[1] * DR16_Rocker_Pitch_Resolution * 1000.0f * Mouse_Sensitivity * 1.8f;
		gimbal_control.Feedback_Speed.Yaw -= 0;//rc_data->Mouse_Ch[0];  
		/*机械角度模式 YAW轴控制*/
		if(gimbal_control.mode == GIMBAL_MECH_MODE)
			gimbal_control.Mech_Ref.Yaw -= rc_data->Mouse_Ch[0] * 180.0f/PI * Mouse_Sensitivity * 0.6f;
		else 
			gimbal_control.Mech_Ref.Yaw = gimbal_upload_data.yaw_encorder + gimbal_upload_data.yaw_round * 8192;
		
		limit(gimbal_control.Mech_Ref.Pitch, MCH_UP_limit, MCH_DOWN_limit);
	} else {
		gimbal_control.Mech_Ref.Pitch = gimbal_upload_data.pitch_encorder;
		gimbal_control.Mech_Ref.Yaw = gimbal_upload_data.yaw_encorder + gimbal_upload_data.yaw_round * 8192;
	}
	// shift加速 长按
	if (rc_data->key[KEY_PRESS].Shift) {
		send_data.Shift_flag = 1;
		if(send_UI.chassis_mode == CHASSIS_SPIN) {
		    chassis->vx *= 1.2f;
		    chassis->vy *= 1.2f;
		    chassis->rotate *= 1.6f;
		} else {
		    chassis->vx *= 2.0f;
		    chassis->vy *= 1.4f;
		}
	}
	else 
		send_data.Shift_flag = 0;
	// Ctrl 解限功率控制
	if (rc_data->key[KEY_PRESS].Ctrl == 1 && Communication_Action_Tx.Key != 1) {
		Communication_Action_Tx.Key = 2;
	}
	
	// F: 快速转头 点按
	if (rc_data->key[KEY_PRESS].F == 1 && key_f_flag == 0) {
		if (gimbal_control.Mid_mode == FRONT )
			 gimbal_control.Mid_mode = BACK;
		else gimbal_control.Mid_mode = FRONT;
		key_f_flag = 1;
		gimbal_control.Gyro_Ref.Yaw += 180.0f;
	}
	if(key_f_flag) {
		key_f_flag ++;
		if(send_UI.chassis_mode == CHASSIS_FOLLOW)
		    send_data.rotate = 0;
		if(rc_data->key[KEY_PRESS].F == 0 && key_f_flag >= 800)
			 key_f_flag = 0;
	}
		
	/* 底盘参数控制 遥控器底盘功能调试*/
	send_UI.chassis_dispatch_mode = (uint8_t )chassis_dispatch_without_acc_limit; // 修改功率模式 调试用
	if(last_chassis != send_UI.chassis_mode)
		last_chassis = send_UI.chassis_mode;
	
    /* 裁判系统返回热量 */
    shoot_control.bullet_mode = BULLET_SINGLE;
    shoot_control.heat_limit_remain = receive_data->shoot_referee_data.heat_limit_remain;  // 剩余热量
}

// 云台底盘模式重置
void mousekey_GimbalChassis_default()
{
    gimbal_control.mode =  GIMBAL_ZERO_FORCE;
    send_UI.chassis_mode = CHASSIS_ZERO_FORCE;
    send_UI.chassis_dispatch_mode = chassis_dispatch_mild;
}

/**
 * @brief 根据gimbal app传回的当前电机角度计算和零位的误差
 */
static float get_offset_angle(short init_forward, short now_encoder) {
    short tmp = 0;
    if (init_forward < 4096) {
        if (now_encoder > init_forward && now_encoder <= 4096 + init_forward) {
            tmp = now_encoder - init_forward;
        } else if (now_encoder > 4096 + init_forward) {
            tmp = -8192 + now_encoder - init_forward;
        } else {
            tmp = now_encoder - init_forward;
        }
    } else {
        if (now_encoder > init_forward) {
            tmp = now_encoder - init_forward;
        } else if (now_encoder <= init_forward && now_encoder >= init_forward - 4096) {
            tmp = now_encoder - init_forward;
        } else {
            tmp = now_encoder + 8192 - init_forward;
        }
    }
    return tmp * 360.0 / 8192.0;
}

static void Gimbal_Refreeget(CANInstance *_instance)
{
	CANCommInstance *comm = (CANCommInstance *)_instance->id;
	
	memcpy(receive_data, _instance->rx_buff, sizeof(Chassis_board_send_t));
	comm->recv_state = 0;
	comm->cur_recv_len = 0;
	comm->update_flag = 1;
	Feed_Dog(comm->Dog);
}
