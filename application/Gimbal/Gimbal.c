#include "Gimbal.h"

#include "DJIMotor.h"
#include "DMmotor.h"
#include "pub_sub.h"
#include "ins_task.h"
#include "remote.h"
#include "PID.h"

#include "controller.h"
#include "usb_typdef.h"

#define YAW 0
#define PITCH 1
#define MIDDLE 0
#define GYRO 1
#define MECH 2

/* 云台部分 */
static Publisher_t    *gimbal_pub;            // 云台控制消息发布者
static Subscriber_t   *gimbal_sub;          // 云台反馈信息订阅者

Gimbal_upload_t gimbal_feedback_data;  // 云台发布的反馈信息

extern  Gimbal_ctrl_cmd_t gimbal_control;
Gimbal_ctrl_cmd_t *gimbal_get_ctrl;// 云台接收CMD的控制信息

DMMotor_Instance_1_4 *pitch_motor;
DJIMotor_Instance *yaw_motor;     
//  功能函数
static attitude_t *ins;
static RC_Ctl_t *rc_data;
static void Gimbal_GYRO_Calc(void);   /** 云台陀螺仪模式  **/ 
static void Gimbal_MIDDLE_Calc(void); /** 云台机械模式  **/ 
static void Gimbal_STOP(void);        /** 云台停止模式 **/ 
static void Gimbal_SystemCalc(float Gimbal_pitch, float Gimbal_yaw); /** 云台电机系统辨识模式 **/
static float auto_rotate_param(void);

static PID_Smis Gimbal_Pos_PID[2][4] = {{{.Kp = 3.0f, .Ki = 0, .Kd = -40.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 10.0f, .inter_threLow = 50, .inter_threUp = 500  },   // YAW轴归中
                              {.Kp =  6.0f, .Ki = 0.0f, .Kd = -35.0f,  .interlimit = 2000, .outlimit = 25000, .DeadBand = 0.01f, .inter_threLow = 5, .inter_threUp = 10},        // YAW轴陀螺仪
							  {.Kp = 10.0f, .Ki = 0.0f, .Kd = -45.0f,  .interlimit = 2000, .outlimit = 25000, .DeadBand = 2, .inter_threLow = 5, .inter_threUp = 10},
                              {.Kp = 15.0f, .Ki = 0.0f, .Kd = -40.0f,  .interlimit = 2000, .outlimit = 25000, .DeadBand = 0.001f, .inter_threLow = 5, .inter_threUp = 10},},       // YAW轴发射

                            {{.Kp = 1.0f , .Ki = 0, .Kd = -2.0f,  .interlimit = 3000, .outlimit = 1000, .DeadBand = 0.10f, .inter_threLow = 5, .inter_threUp = 500},   // PITCH轴归中
                             {.Kp = 0.50f,  .Ki = 0, .Kd = -2.5f, .interlimit = 2000, .outlimit = 1000, .DeadBand = 0.0f, .inter_threLow = 5, .inter_threUp = 10},   // PITCH轴陀螺仪
                             {.Kp = 2.5f,  .Ki = 0, .Kd = 0.25f, .interlimit = 2000, .outlimit = 1000, .DeadBand = 0.0f, .inter_threLow = 5, .inter_threUp = 10},
                             {.Kp = 1.8f,  .Ki = 0, .Kd = -5.0f, .interlimit = 2000, .outlimit = 1000, .DeadBand = 0.0f, .inter_threLow = 5, .inter_threUp = 10}}};   // PITCH轴发射
						     
static PID Gimbal_Speed_PID[2][4] = {{{.Kp = 5.0f, .Ki = 0.5f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 25000, .DeadBand = 2, .inter_threLow = 500, .inter_threUp = 1000},
							   {.Kp = 700.0f, .Ki = 0.50f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 29000, .DeadBand = 0.0f, .inter_threLow = 2.5f, .inter_threUp = 30},
							   {.Kp = 12.0f,  .Ki = 0.0f, .Kd = 0.0f, .interlimit = 2000, .outlimit = 29000, .DeadBand = 0.0f, .inter_threLow = 100, .inter_threUp = 500},
							   {.Kp = 250.0f, .Ki = 0.0f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 29000, .DeadBand = 0.0f, .inter_threLow = 50, .inter_threUp = 100}},
							
                            {{.Kp = 1.0f,  .Ki = 0.0f,  .Kd = 0.0f, .interlimit = 50, .outlimit = 1000, .DeadBand = 0.25f, .inter_threLow = 500, .inter_threUp = 1000},
                             {.Kp = 8.0f, .Ki = 0.0f,  .Kd = 0.0f, .interlimit = 50, .outlimit = 1000, .DeadBand = 0.00f, .inter_threLow = 5, .inter_threUp = 15},
                             {.Kp = 1.8f,  .Ki = 0.0f,  .Kd = 0.0f, .interlimit = 50, .outlimit = 1000, .DeadBand = 0.0f, .inter_threLow = 20, .inter_threUp = 50},
                             {.Kp = 2.0f,  .Ki = 0.0f,  .Kd = 0.0f, .interlimit = 50, .outlimit = 1000, .DeadBand = 0.0f, .inter_threLow = 10, .inter_threUp = 50}}};

/* 位置环PID为底盘跟随解算旋转速度 */
static PID_Smis Chassis_Rotate_PIDS = {.Kp = 0.4f, .Ki = 0, .Kd = -40.0f, .interlimit = 1500, .outlimit = 2000*PI, .DeadBand = 5, .inter_threLow = 500, .inter_threUp = 2000};     // 底盘跟随   位置环
static PID Chassis_Rotate_PID = { .Kp = 6.0f, .Ki = 0.0f, .Kd = 0, .interlimit = 4000 * PI, .outlimit = 10000*PI, .DeadBand = 0, .inter_threLow = 10*PI, .inter_threUp = 4000*PI};     // 速度环
// 前反馈
static FeedForward_Typedef FF_Mose_rotete = { .K1 = 30.0f, .K2 = 0.0f, .K3 = 0.0f, .OutMax = 8000 * PI, .DWT_CNT = 0};
static FeedForward_Typedef FF_Remote_rotete = { .K1 = 20.0f, .K2 = 0.0f, .K3 = 0.0f, .OutMax = 8000 * PI, .DWT_CNT = 0};
static FeedForward_Typedef GimbalYaw_FF = { .K1 = 0.0f, .K2 = 20.0f, .K3 = 0.0f , .OutMax = 3000, .DWT_CNT = 0};

void Gimbal_Init(void)
{
	memset(&gimbal_feedback_data, 0, sizeof(Gimbal_upload_t) );
 	gimbal_pub = PubRegister("gimbal_upload", sizeof(Gimbal_upload_t) );
	gimbal_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_ctrl_cmd_t) );
    
	gimbal_get_ctrl = (Gimbal_ctrl_cmd_t* )RMLIB_MALLOC(sizeof(Gimbal_ctrl_cmd_t));
	memset(gimbal_get_ctrl, 0, sizeof(Gimbal_ctrl_cmd_t));
	ins = INS_Init();
	rc_data = get_remote_control_point();
	
Motor_Init_config_s pitch_config = {
	.motor_type = DM4310,
	.dog_init_config = {
		.dog_name = "Pitch",
		.GPIO_Pin = GPIO_PIN_13,
	},
	.can_init_config = {
		.can_handle = &hcan2,
		.tx_id = 1,
		.rx_id = 0x301,
	},
	.contruller_config = {
		.MotorCenter = Pitch_Mid,
		.setting_config = {
			.feedback_reverse_flag = FEEDBACK_NORMAL,
			.motor_reverse_flag =   MOTOR_NORMAL,   // 电机反转
			.angle_feedback_source = OTHER_FEED,    //云台为MID归中模式时使用机械角度
			.speed_feedback_source = OTHER_FEED,    // 云台初始化后使用 DJIMotorChangeFeed函数修改反馈类型使用IMU控制
			.close_loop_type = ANGLE_LOOP,
			.feedfoward_type = NONE_FEEDFORWARD,    // 不使用前馈类型 
			.control_mode = PID_MODEL,
		},
	},
	.other_angle_feedback_ptr = &ins->Pitch,
	.other_speed_feedback_ptr = &ins->gyro[0],
};

Motor_Init_config_s yaw_config = {
	.motor_type = GM6020,
	.dog_init_config = {
		.dog_name = "Yaw",
	    .GPIO_Pin = GPIO_PIN_13,
	},
	.can_init_config = {
		.can_handle = &hcan2,
		.tx_id = 1,
		.rx_id = 205,
	},
	.contruller_config = {
		.MotorCenter = Yaw_Mid_Front,
		.setting_config = {
			.motor_reverse_flag = MOTOR_NORMAL,   // 电机不反转
			.feedback_reverse_flag = FEEDBACK_NORMAL,
			.angle_feedback_source = MOTOR_FEED,
			.speed_feedback_source = MOTOR_FEED,    // 云台初始化后使用 DJIMotorChangeFeed函数修改反馈类型使用IMU控制
			.close_loop_type = ANGLE_LOOP,          // 使用闭环类型 默认位置环位最外环
			.feedfoward_type = NONE_FEEDFORWARD, // 不使用前馈类型 
			.control_mode = PID_MODEL,
		},
	},
	.other_angle_feedback_ptr = &ins->ContinuousYaw,
	.other_speed_feedback_ptr = &ins->gyro[2],
};
    
/* 跟随PID初始化 */
	pitch_motor =  DMMotorInit_1_4(&pitch_config);
    yaw_motor = DJI_Motor_create(&yaw_config);
}

/*
	PubPushMessage(gimbal_pub, (void *)&gimbal_feed);
	SubGetMessage(gimbal_sub, gimbal_get_control);
*/
void Gimbal_Task(void)
{
	// 通过订阅者获得控制参数
	if (SubGetMessage( gimbal_sub,(void *) gimbal_get_ctrl) != 1) {
		yaw_motor->stop_flag  = MOTOR_STOP;
	    pitch_motor->stop_flag = MOTOR_STOP;
		return;
	}
	// 检测是否有外设掉线
	if ( yaw_motor->watchdog->state == Dog_Online) {
		gimbal_feedback_data.gimbal_status = Device_Online;
		gimbal_feedback_data.yaw_encorder = yaw_motor->measure.MchanicalAngle;
		gimbal_feedback_data.yaw_round = yaw_motor->measure.round;
		gimbal_feedback_data.pitch_encorder = pitch_motor->measure.Angle_DEG;
	} else {
		gimbal_feedback_data.gimbal_status = Device_Offline;
		gimbal_get_ctrl->mode = GIMBAL_ZERO_FORCE ;
	}
   gimbal_feedback_data.pitch_status = (uint8_t)pitch_motor->watchdog->state;
   switch (gimbal_get_ctrl->mode) {
		// 启动电机
			case GIMBAL_ZERO_FORCE :
		   	    yaw_motor->stop_flag   = MOTOR_STOP;
		        pitch_motor->stop_flag = MOTOR_STOP;
			    gimbal_feedback_data.rotate_speed = 0;
			    Gimbal_STOP();
			break;
        // 使用陀螺仪的反馈,底盘根据yaw电机的offset跟随云台或视觉模式采用
			case GIMBAL_GYRO_MODE :
				DJIMotorEnable(yaw_motor);
			    DMMotorEnable(pitch_motor);
			   if(gimbal_get_ctrl->last_mode != GIMBAL_GYRO_MODE) {
					DJIMotorChangeFeed(yaw_motor,  OTHER_FEED);
					DMMotorChangeFeed(pitch_motor, OTHER_FEED);
    			}
			    DJIMotorSetRef(yaw_motor,  gimbal_get_ctrl->Gyro_Ref.Yaw);
			    DMMotorSetRef(pitch_motor, gimbal_get_ctrl->Gyro_Ref.Pitch);
			    /* 底盘跟随速度解算 */
				Gimbal_GYRO_Calc();
 			    auto_rotate_param();
			break;
				/* 机械控制模式  */
			case GIMBAL_MECH_MODE :
			case GIMBAL_MIDDLE :
				DJIMotorEnable(yaw_motor);
			    DMMotorEnable(pitch_motor);
			    if(gimbal_get_ctrl->last_mode != GIMBAL_MIDDLE) {
					DJIMotorChangeFeed(yaw_motor, MOTOR_FEED);
					DMMotorChangeFeed(pitch_motor, MOTOR_FEED);
			    }
				Gimbal_MIDDLE_Calc();
			break;
			
			case GIMBAL_SYSTEM_MODE :
				Gimbal_SystemCalc(0, 15000);
			break;
				
			default :
		   	    yaw_motor->stop_flag  = MOTOR_STOP;
		        pitch_motor->stop_flag = MOTOR_STOP;
			    Gimbal_STOP();
			break;
	}
	gimbal_feedback_data.mode = gimbal_get_ctrl->mode;
	// 推送消息
	PubPushMessage( gimbal_pub, (void *)&gimbal_feedback_data);
}
float rotate_ff, ff_yawspeed, yaw_pidout;
static void Gimbal_GYRO_Calc()
{
     /* 电机发送 */
    static int16_t can2_dji_send[4], can2_dm_send[4];
	float deadline_pitch ;
	if(Aim_Ref.auto_mode == 0) {
		/**  YAW轴解算  遥控器模式  **/
		PID_Control_Smis( ins->ContinuousYaw, gimbal_get_ctrl->Gyro_Ref.Yaw, &Gimbal_Pos_PID[0][1], ins->gyro[2] - gimbal_get_ctrl->Feedback_Speed.Yaw); //  
		PID_Control( ins->gyro[2], Gimbal_Pos_PID[0][1].pid_out + gimbal_get_ctrl->Feedback_Speed.Yaw *0.8f - gimbal_get_ctrl->rotate_feedforward * 4.0f, &Gimbal_Speed_PID[0][1]);  // * 2.0f
		can2_dji_send[0] = (int16_t )Gimbal_Speed_PID[0][1].pid_out;
		DJIMotor_Transmit(&hcan2, 0x1FF, can2_dji_send);

		/**  PITCH轴解算 **/
//		PID_Control_Smis( pitch_motor->measure.Angle_DEG, gimbal_get_ctrl->Gyro_Ref.Pitch, &Gimbal_Pos_PID[1][1], pitch_motor->measure.SpeedFilter*PI/30 + gimbal_get_ctrl->Feedback_Speed.Pitch);
//		PID_Control( pitch_motor->measure.SpeedFilter*PI/30, Gimbal_Pos_PID[1][1].pid_out + gimbal_get_ctrl->Feedback_Speed.Pitch * 2.0f, &Gimbal_Speed_PID[1][1]);
//		can2_dm_send[0] = (int16_t )Gimbal_Speed_PID[1][1].pid_out;
//		DJIMotor_Transmit(&hcan2, 0x3FE, can2_dm_send);

        if(gimbal_feedback_data.pitch_encorder > MCH_UP_limit || gimbal_feedback_data.pitch_encorder < MCH_DOWN_limit)
			can2_dm_send[0] = 0;
		else {
			deadline_pitch = ins->gyro[0];
			deadline_limit(deadline_pitch, 0.1f);
			PID_Control_Smis( ins->Pitch, gimbal_get_ctrl->Gyro_Ref.Pitch, &Gimbal_Pos_PID[1][1], deadline_pitch);
			PID_Control( deadline_pitch, -Gimbal_Pos_PID[1][1].pid_out - gimbal_get_ctrl->Feedback_Speed.Pitch, &Gimbal_Speed_PID[1][1]);
			can2_dm_send[0] = (int16_t )Gimbal_Speed_PID[1][1].pid_out;
		}

		DJIMotor_Transmit(&hcan2, 0x3FE, can2_dm_send);
		yaw_pidout = Gimbal_Pos_PID[1][1].pid_out;
		rotate_ff = gimbal_get_ctrl->rotate_feedforward;
		ff_yawspeed = gimbal_get_ctrl->Feedback_Speed.Pitch;
	} else {
		/**  YAW轴解算  自瞄模式  **/
		PID_Control_Smis( ins->ContinuousYaw, Aim_Ref.yaw, &Gimbal_Pos_PID[0][3], ins->gyro[2]);
		PID_Control( ins->gyro[2], Gimbal_Pos_PID[0][3].pid_out + Aim_Ref.yaw_vel*3.0f, &Gimbal_Speed_PID[0][3]);
		can2_dji_send[0] = (int16_t )Gimbal_Speed_PID[0][3].pid_out;
		DJIMotor_Transmit(&hcan2, 0x1FF, can2_dji_send);

		/**  PITCH轴解算 **/ 
	    limit(Aim_Ref.pitch, IMU_UP_limit, IMU_DOWN_limit);		
		PID_Control_Smis( pitch_motor->measure.Angle_DEG, Aim_Ref.pitch, &Gimbal_Pos_PID[1][3], pitch_motor->measure.SpeedFilter*PI/30);
		PID_Control( pitch_motor->measure.SpeedFilter*PI/30, Gimbal_Pos_PID[1][3].pid_out + Aim_Ref.pitch_vel, &Gimbal_Speed_PID[1][3]);
		can2_dm_send[0] = (int16_t )Gimbal_Speed_PID[1][3].pid_out;
		DJIMotor_Transmit(&hcan2, 0x3FE, can2_dm_send);
//		PID_Control_Smis( ins->Pitch, Aim_Ref.pitch, &Gimbal_Pos_PID[1][2], ins->gyro[0]);
//		PID_Control( ins->gyro[0], -Gimbal_Pos_PID[1][2].pid_out + Aim_Ref.pitch_vel, &Gimbal_Speed_PID[1][2]);
//		can2_dm_send[0] = (int16_t )Gimbal_Speed_PID[1][2].pid_out;
//		DJIMotor_Transmit(&hcan2, 0x3FE, can2_dm_send);
	}
	PID_IoutReset(&Gimbal_Speed_PID[1][0]);
	PID_IoutReset(&Gimbal_Speed_PID[1][2]);
	PID_IoutReset(&Gimbal_Speed_PID[0][0]);
	PID_IoutReset(&Gimbal_Speed_PID[0][2]);
}

static void Gimbal_MIDDLE_Calc()
{                                                                                                
	/* 电机发送 */
    static int16_t can2_dji_send[4], can2_dm_send[4];
	static float Expect_Yaw, Expect_Pitch;  // 获取期望机械角度	
	
	if(gimbal_get_ctrl->mode == GIMBAL_MIDDLE) {  
		if(gimbal_get_ctrl->Mid_mode == FRONT )
		   Expect_Yaw = QuickCentering(yaw_motor->measure.MchanicalAngle, Yaw_Mid_Front);
	    else 
		   Expect_Yaw = QuickCentering(yaw_motor->measure.MchanicalAngle, Yaw_Mid_Back);
 	    auto_rotate_param();
	} else { 
		Expect_Yaw =  gimbal_get_ctrl->Mech_Ref.Yaw;
	    gimbal_feedback_data.rotate_speed = 0;
	}
	
	Expect_Pitch = gimbal_get_ctrl->Mech_Ref.Pitch;
    /**  YAW轴解算  **/ 	/**  PITCH轴解算 **/
	if (gimbal_get_ctrl->shoot_mode == SHOOT_NOMAL ||  gimbal_get_ctrl->shoot_mode == SHOOT_READY) {
        		
		gimbal_get_ctrl->mode == GIMBAL_MIDDLE ? PID_Control_Smis( yaw_motor->measure.MchanicalAngle, Expect_Yaw, &Gimbal_Pos_PID[0][2], yaw_motor->measure.SpeedFilter)
							: PID_Control_Smis( yaw_motor->measure.Angle, Expect_Yaw, &Gimbal_Pos_PID[0][2], yaw_motor->measure.SpeedFilter);
		PID_Control( yaw_motor->measure.SpeedFilter, Gimbal_Pos_PID[0][2].pid_out + gimbal_get_ctrl->Feedback_Speed.Yaw*0.8f, &Gimbal_Speed_PID[0][2]);
		can2_dji_send[0] = (int16_t )Gimbal_Speed_PID[0][2].pid_out;
   	    DJIMotor_Transmit(&hcan2, 0x1FF, can2_dji_send);
		
		PID_Control_Smis( pitch_motor->measure.Angle_DEG, Expect_Pitch, &Gimbal_Pos_PID[1][2], pitch_motor->measure.SpeedFilter*PI/30);
		PID_Control( pitch_motor->measure.SpeedFilter*PI/30, Gimbal_Pos_PID[1][2].pid_out, &Gimbal_Speed_PID[1][2]);
		can2_dm_send[0] = (int16_t )Gimbal_Speed_PID[1][2].pid_out;
   	    DJIMotor_Transmit(&hcan2, 0x3FE, can2_dm_send);
		
		PID_IoutReset(&Gimbal_Speed_PID[0][0]);
  	    PID_IoutReset(&Gimbal_Speed_PID[1][0]);
	} else {
		gimbal_get_ctrl->mode == GIMBAL_MIDDLE ? PID_Control_Smis( yaw_motor->measure.MchanicalAngle, Expect_Yaw, &Gimbal_Pos_PID[0][0], yaw_motor->measure.SpeedFilter)
						: PID_Control_Smis( yaw_motor->measure.Angle, Expect_Yaw, &Gimbal_Pos_PID[0][0], yaw_motor->measure.SpeedFilter);
		PID_Control(yaw_motor->measure.SpeedFilter, Gimbal_Pos_PID[0][0].pid_out, &Gimbal_Speed_PID[0][0]);
		can2_dji_send[0] = (int16_t )Gimbal_Speed_PID[0][0].pid_out;
   	    DJIMotor_Transmit(&hcan2, 0x1FF, can2_dji_send);

		PID_Control_Smis( pitch_motor->measure.Angle_DEG, Expect_Pitch, &Gimbal_Pos_PID[1][0], pitch_motor->measure.SpeedFilter*PI/30);
		PID_Control( pitch_motor->measure.SpeedFilter*PI/30, Gimbal_Pos_PID[1][0].pid_out, &Gimbal_Speed_PID[1][2]);
		can2_dm_send[0] = (int16_t )Gimbal_Speed_PID[1][2].pid_out;
   	    DJIMotor_Transmit(&hcan2, 0x3FE, can2_dm_send);
																  
		PID_IoutReset(&Gimbal_Speed_PID[0][2]);
  	    PID_IoutReset(&Gimbal_Speed_PID[1][2]);
	}
	PID_IoutReset(&Gimbal_Speed_PID[0][1]);
	PID_IoutReset(&Gimbal_Speed_PID[1][1]);
}

static void Gimbal_STOP()
{      /* 电机发送 */
    static int16_t can2_dji_send[4], can2_dm_send[4];
	
	can2_dji_send[0] = 0;
	can2_dm_send[0] = 0;
#if GIMBAL_MOVE
	DJIMotor_Transmit(&hcan2, 0x1FF, can2_dji_send);
	DJIMotor_Transmit(&hcan2, 0x3FE, can2_dm_send);
#endif
	PID_IoutReset(&Gimbal_Speed_PID[0][0]);
	PID_IoutReset(&Gimbal_Speed_PID[1][0]);
	PID_IoutReset(&Gimbal_Speed_PID[0][1]);
	PID_IoutReset(&Gimbal_Speed_PID[0][2]);
	PID_IoutReset(&Gimbal_Speed_PID[1][2]);
}


/**
 * @brief  云台系统辨识模式
 * 
 * @param 根据不同的频率获得对应不同周期下sin信号,F低于50重复10周期，否则20周期
*/
__WEAK void get_F();
float Gimbal_direct = 0.0f, Gimbal_Speed = 0.0f;
uint8_t F_Change_flag = 0, T_cnt = 0;
static void Gimbal_SystemCalc(float Gimbal_pitch, float Gimbal_yaw)
{
    /* 电机发送 */
    static int16_t can2_dji_send[4], can2_dm_send[4], idx = 0, gimbalyaw_init;
	if(idx++  == 0 ) {
		 gimbalyaw_init = ins->ContinuousYaw;
	}
																											      
     if( F_Change_flag == 1 && fabs(ins->ContinuousYaw - gimbalyaw_init) <= 1.0f) {
		get_F();
		T_cnt = 0;
		F_Change_flag = 0;
	 }
	 
	/* 用于开环系统辨识 */
	Gimbal_Speed = yaw_motor->measure.SpeedFilter;
    can2_dji_send[0] = Gimbal_direct * Gimbal_yaw;
    MotorSend(&hcan2, 0x1FF, can2_dji_send);
}

/**
 * @brief 底盘不同模式旋转速度解算 
 *
**/
float ff_cahssis_follow = 0;
float rotate_ = 0.0f;

static float auto_rotate_param()
{
	uint16_t roate_speed_MAX = 8000 * PI;   // 设置最大旋转速度
	float angle, offset_angle;
	int16_t yaw_mid;
	offset_angle = yaw_motor->measure.MchanicalAngle;
    
	float gyro_deadband = ins->gyro[2];
	deadline_limit(gyro_deadband, 0.50f);
	
	 gimbal_get_ctrl->Mid_mode == FRONT ? ( yaw_mid = QuickCentering(offset_angle ,Yaw_Mid_Front) ) : ( yaw_mid = QuickCentering(offset_angle ,Yaw_Mid_Back) );
    if(gimbal_get_ctrl->chassis_mode == CHASSIS_FOLLOW)
	{
		PID_Control_Smis(offset_angle, yaw_mid,&Chassis_Rotate_PIDS, gimbal_get_ctrl->rotate_feedforward);
		PID_Control(gimbal_get_ctrl->rotate_feedforward, Chassis_Rotate_PIDS.pid_out + gimbal_get_ctrl->Feedback_Speed.Yaw,&Chassis_Rotate_PID);
		ff_cahssis_follow = rc_data->RemoteMode == REMOTE_INPUT ? FeedForward_Calc(&FF_Remote_rotete, gimbal_get_ctrl->Feedback_Speed.Yaw * 30)
						: FeedForward_Calc(&FF_Mose_rotete, gyro_deadband * 35);
		rotate_ = Chassis_Rotate_PID.pid_out - ff_cahssis_follow;
        limit(rotate_, +roate_speed_MAX*0.8f, -roate_speed_MAX*0.8f);		
	}
	else if (gimbal_get_ctrl->chassis_mode == CHASSIS_SPIN)
	{
		//按角度变速（头为前方，装甲板向前转的快，对角转的慢）（angle为比例）
//		if (offset_angle <= 7880 && offset_angle >= 5830)
//			angle = ABS(6855.0f - offset_angle) / 2050.57f;
//		if (offset_angle <= 5830 && offset_angle >= 3780)
//			angle = ABS(4805.0f - offset_angle) / 2050.57f;
//		if (offset_angle <= 3780 && offset_angle >= 1730)
//			angle = ABS(2755.0f - offset_angle) / 2050.57f;
//		if (offset_angle <= 1730 && offset_angle > 0)
//			angle = ABS(705.0f - offset_angle) / 2050.57f;
//		if (offset_angle <= 8191 && offset_angle >= 7880)
//			angle = ABS(705.0f + 8191.0f - offset_angle) / 2050.57f;
		rotate_ = 3500.0f * PI;// + 1500.0f * PI * arm_cos_f32(angle*PI);
	}
	else if (gimbal_get_ctrl->chassis_mode == CHASSIS_NORMAL) 
	{
		if(rc_data->RemoteMode == REMOTE_INPUT)
			 rotate_  = (float )rc_data->Key_CH[2] * 2000 * PI + FeedForward_Calc(&FF_Remote_rotete, rc_data->Key_CH[2]*0.5f);  
		else if(rc_data->RemoteMode == KEY_MOUSE_INPUT)
			 rotate_  = (float )rc_data->Mouse_Ch[0] * 2000 * PI + FeedForward_Calc(&FF_Mose_rotete, rc_data->Mouse_Ch[0]*0.1f);
	} 
	else 
		rotate_ = 0;
    
    limit(rotate_, +roate_speed_MAX, -roate_speed_MAX);
	gimbal_feedback_data.rotate_speed = rotate_;
	return rotate_;
}

 /* 时间式变速 等待测试*/
////   static uint8_t spin_speed_change = 1;// 0：定速 1：加速 2：减速 （初始从低往高加）
////     // 基准转速 = 最低转速 + 高功率下加速旋转
////   float rotate_benchmark = 8000 + Referee_data_Rx.robot_level * 200;
////   if (spin_speed_change == 0) {
////     // 定速
////     rotate = rotate_benchmark;
////   } else {
////     // 变速范围
////     float rotate_max = rotate_benchmark + 50;
////     float rotate_min = rotate_benchmark - 50;
////     if (rotate < rotate_min) rotate = rotate_min;
////     if (rotate > rotate_max) rotate = rotate_max;
////     if (spin_speed_change == 1) {
////         // 加速（目前为线性）
////         rotate += 0.0005;
////         if (rotate > rotate_max){
////             spin_speed_change = 2;
////         }
////     }else {
////         // 减速 建议不要使用功率减速 以使同功率下整体平均转速较高
////         rotate -= 0.0001;
////         if (rotate < rotate_min){
////             spin_speed_change = 1;
////            }
////         }
////    }

