#include "Shoot.h"
#include "motor.h"
#include "PID.h"
#include "user_lib.h"
#include "bsp_dwt.h"

#include "usb_typdef.h"

/*  发射部分 */
static DJIMotor_Instance *friction_r, *friction_l, *friction_u, *loader;
static Publisher_t      *shoot_pub;          // 云台控制消息发布者
static Subscriber_t     *shoot_sub;          // 云台反馈信息订阅者
static Shoot_ctrl_cmd_t *shoot_ctrl_cmd;     // 云台接收CMD的控制信息
static Shoot_upload_t    shoot_feedback_data;// 云台发布的反馈信息

/* 用于拨弹盘发射 */
static float Angle_Target = 0;                //!< @brief 拨弹盘期望角度
static float Last_Target_Pluck = 0, Off_Angle = 0;   //!< @brief 发弹补偿
static float RAMP_Angle_Target = 0;           //!< @brief 拨弹盘期望斜坡角度
static Bullet_mode_e Last_Bullet_mode;        //!< @brief 用于计算拨弹盘角度补偿
static float cooldown_start;                  //!< @brief 冷却起始时间点
static int16_t cooldown_tim;                  //!< @brief 冷却时间

/**  双发补偿控制  **/
//!< @brief 双发检测时间
uint8_t double_heat_flag = 0;         //!< @brief 双发检测标志位
// uint8_t double_heat_tims = 0;         //!< @brief 双发检测时间戳
float DOUBLE_HEAT_DETECT = 0.0f, SHOOT_UNIT_HEAT_START = 0.0f, SHOOT_DOUBLE_HEAT_START = 0.0f;    //!< @brief 双发检测时间戳
uint8_t SHOOT_FLAG_TIMS[50] = {0}, SHOOT_FLAG_NUM = 0; //!< @brief 发射弹丸记录
/**  其他功能函数  **/
static void Shoot_load_Update(void);     // 拨弹盘控制
static void Shoot_friction_Update(void); // 摩擦轮控制
static void Shoot_Calc(void);
static void Shoot_STOP(void);
static void Shoot_Get_OffsetAngle(int16_t load_delta_pos);
static void DOUBLE_HEAL_Detect();
static Smc shoot_smc[4];
static PID Shoot_Speed_PID[3] = {{.Kp = 5, .Ki = 0.0f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 16000, .DeadBand = 0.50f, .inter_threLow = 500, .inter_threUp = 1000},            //摩擦轮左
                          {.Kp = 5, .Ki = 0.0f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 16000, .DeadBand = 0.50f, .inter_threLow = 500, .inter_threUp = 1000},                  //摩擦轮右
						  {.Kp = 5, .Ki = 0.0f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 16000, .DeadBand = 0.50f, .inter_threLow = 500, .inter_threUp = 1000}};                 // 摩擦轮上
							  
static PID_Smis Pluck_Place_PIDS = {.Kp = 5.0f, .Ki = 0, .Kd = 0.4f, .interlimit = 3000, .outlimit = 16000, .DeadBand = 0.0f, .inter_threLow = 500, .inter_threUp = 1000};         //拨弹盘单发位置环
static PID Pluck_Speed_PID = {.Kp = 5.0f, .Ki = 0.0f, .Kd = 0.0f, .interlimit = 5000, .outlimit = 15000, .DeadBand = 0.0f, .inter_threLow = 20, .inter_threUp = 5000};                   //拨弹盘单发速度环
static PID Pluck_Continue_PID = {.Kp = 40, .Ki = 10.0, .Kd = 0, .interlimit = 5000, .outlimit = 15000, .DeadBand = 0.0f, .inter_threLow = 500, .inter_threUp = 1000};               //拨弹盘连发模式

void Shoot_Init()
{
    SMC_SetConfig(&shoot_smc[0].config, 4.0f, 25, 200, ReachingLaw_sqrt, 15000);
    SMC_SetConfig(&shoot_smc[1].config, 4.0f, 25, 200, ReachingLaw_sqrt, 15000);
    SMC_SetConfig(&shoot_smc[2].config, 4.0f, 25, 200, ReachingLaw_sqrt, 15000);
    
    SMC_SetConfig(&shoot_smc[3].config, 6.0f, 50, 100, ReachingLaw_sqrt, 15000);
	
	shoot_pub = PubRegister("shoot_upload", sizeof(Shoot_upload_t));
	shoot_sub = SubRegister("shoot_cmd", sizeof(Shoot_ctrl_cmd_t));

Motor_Init_config_s loader_config = {
	.motor_type = RM3508,
	.dog_init_config = {
		.dog_name = "loader",
		.GPIO_Pin = GPIO_PIN_1,
	},
	.can_init_config = {
		.can_handle = &hcan1,
		.tx_id = 1,
		.rx_id = 201,
	},
	.contruller_config = {
		.MotorCenter = 0,
		.setting_config = {
			.feedback_reverse_flag = FEEDBACK_NORMAL,
			.motor_reverse_flag =  MOTOR_NORMAL,    // 电机不反转
			.angle_feedback_source = MOTOR_FEED,    // 云台为MID归中模式时使用机械角度
			.speed_feedback_source = MOTOR_FEED,    // 云台初始化后使用 DJIMotorChangeFeed函数修改反馈类型使用IMU控制
			.close_loop_type = ANGLE_LOOP,
			.feedfoward_type = NONE_FEEDFORWARD, // 不使用前馈类型
			.control_mode = PID_MODEL,
		}
	},
	.other_angle_feedback_ptr = 0,
	.other_speed_feedback_ptr = 0,
};
	loader = DJI_Motor_create(&loader_config);

Motor_Init_config_s friction_u_config = {
	.motor_type = RM3508,
	.dog_init_config = {
		.dog_name = "fire_u",
	    .GPIO_Pin = GPIO_PIN_1,
	},
	.can_init_config = {
		.can_handle = &hcan1,
		.tx_id = 4,
		.rx_id = 204,
	},
	.contruller_config = {
		.setting_config = {
			.motor_reverse_flag = MOTOR_NORMAL,   // 电机不反转
			.feedback_reverse_flag = FEEDBACK_NORMAL,
			.angle_feedback_source = MOTOR_FEED,
			.speed_feedback_source = MOTOR_FEED,    // 云台初始化后使用 DJIMotorChangeFeed函数修改反馈类型使用IMU控制
		.close_loop_type = SPEED_LOOP,          // 使用闭环类型 默认位置环位最外环
		.feedfoward_type = NONE_FEEDFORWARD, // 不使用前馈类型 
		.control_mode = PID_MODEL,
		},
	},
	.other_angle_feedback_ptr = 0,
	.other_speed_feedback_ptr = 0,
};
	friction_u = DJI_Motor_create(&friction_u_config);

Motor_Init_config_s friction_r_config = {
	.motor_type = RM3508,
	.dog_init_config = { 
		.dog_name = "fire_r",
		.GPIO_Pin = GPIO_PIN_1,
	},
	.can_init_config = {
		.can_handle = &hcan1,
		.tx_id = 3,
		.rx_id = 203,
	},
	.contruller_config = {
		.setting_config = {
			.motor_reverse_flag = MOTOR_NORMAL,   // 电机不反转
			.feedback_reverse_flag = FEEDBACK_NORMAL,
			.angle_feedback_source = MOTOR_FEED,
			.speed_feedback_source = MOTOR_FEED,    // 云台初始化后使用 DJIMotorChangeFeed函数修改反馈类型使用IMU控制
		.close_loop_type = SPEED_LOOP,          // 使用闭环类型 默认位置环位最外环
		.feedfoward_type = NONE_FEEDFORWARD, // 不使用前馈类型 
		.control_mode = PID_MODEL,
		},
	},
	.other_angle_feedback_ptr = 0,
	.other_speed_feedback_ptr = 0,
};
  friction_r = DJI_Motor_create(&friction_r_config);
  
   Motor_Init_config_s friction_l_config = {
	.motor_type = RM3508,
	.dog_init_config = {
		.dog_name = "fire_l",
	    .GPIO_Pin = GPIO_PIN_1,
	},
	.can_init_config = {
		.can_handle = &hcan1,
		.tx_id = 2,
		.rx_id = 202,
	},
    .contruller_config = {
		.setting_config = {
		.motor_reverse_flag = MOTOR_NORMAL,   // 电机不反转
		.feedback_reverse_flag = FEEDBACK_NORMAL,
		.angle_feedback_source = MOTOR_FEED,
		.speed_feedback_source = MOTOR_FEED,    // 云台初始化后使用 DJIMotorChangeFeed函数修改反馈类型使用IMU控制
		.close_loop_type = SPEED_LOOP,          // 使用闭环类型 默认位置环位最外环
		.feedfoward_type = NONE_FEEDFORWARD, // 不使用前馈类型 
		.control_mode = PID_MODEL,
	  },
	},
	.other_angle_feedback_ptr = 0,
	.other_speed_feedback_ptr = 0,
};
  friction_l = DJI_Motor_create(&friction_l_config);

  // 订阅者初始化
  shoot_ctrl_cmd = (Shoot_ctrl_cmd_t *)malloc(sizeof(Shoot_ctrl_cmd_t));
  memset(shoot_ctrl_cmd, 0, sizeof(Shoot_ctrl_cmd_t));
  
  cooldown_tim = 0;
  cooldown_start = DWT_GetTimeline_ms();
}

/*
	PubPushMessage(gimbal_pub, (void *)&gimbal_feed);
	SubGetMessage(gimbal_sub, gimbal_get_control);
*/
static void Shoot_load_Update()
{
    static uint16_t  Stuck_time   = 0, continur_tims = 0;//!< @brief 检测卡弹的时间 连发计时
	static uint8_t Add_Angle_Flag = 0;               	 //!< @brief 拨弹盘角度+1标志位
    static float Stuck_Angle_Target = 0;                 //!< @brief 拨弹盘卡弹退弹期望角度
    static const uint16_t Stuck_thre = 800;              //!< @brief 卡弹阈值，卡弹超过此时间便认为卡弹
	static float Target_Continue_Speed = -360.0f * REDUCTION_RATIO_WHEEL / SHOOT_NUM_PER_CIRCLE * 1.0f;  // 播弹盘连发速度
    static float abs_deltePlack, abs_deltaSpeed, loader_angle_deg;
	// 设定拨弹盘转动角度
	static int16_t load_delta_pos = -1385;
	
    /* 暂停模式 */	
	if(shoot_ctrl_cmd->mode == SHOOT_STOP) 
	{
		shoot_ctrl_cmd->bullet_mode = BULLET_HOLDON;
		/** 卡弹时间清零  **/
		Stuck_time = 0;
		Off_Angle = 0;
		DJIMotorOuterLoop(loader, ANGLE_LOOP);   // 修改为角度控制
		DJIMotorSetRef(loader, Angle_Target);
	}
	else 
	{
	 /**  手动退弹模式 **/
		if(shoot_ctrl_cmd->mode == SHOOT_STUCKING) 
		{
		    if(Stuck_time < 800) {
				Stuck_time = 800;
			} else { 
				Stuck_time ++;
			}
		}
		
		/** 自瞄模式 **/
//		if(shoot_ctrl_cmd->mode == SHOOT_AIM) {
//			if(Aim_Ref.fire_on)
//			{
//				shoot_ctrl_cmd->mode = SHOOT_NOMAL;
//				// 自瞄模式不用连发
//				continur_tims = 0;
//				Add_Angle_Flag = 0;
//			} 
//			if(Aim_Ref.fire_flag == 1)
//			{
//				Add_Angle_Flag = 1;
//				Aim_Ref.fire_flag = 0;
//			}
//			else
//			{
//				Add_Angle_Flag = 0;
//			}
//	    }
		
		/**  连发设置  **/
		if(shoot_ctrl_cmd->mode == SHOOT_NOMAL) 
		{
			continur_tims ++;
			if(continur_tims >= 1000) 
			{
			   shoot_ctrl_cmd->bullet_mode = BULLET_CONTINUE;
			   Stuck_time = 0;
			}
		} else	continur_tims  = 0; 
		
		/** 判断卡弹时间 **/ 
		if(Stuck_time >= Stuck_thre)
		 	 shoot_ctrl_cmd->mode = SHOOT_STUCKING;
		
		/** 计算拨弹盘角度补偿 **/
		Shoot_Get_OffsetAngle(load_delta_pos);
		
		/** 卡弹检测 **/
	    if(shoot_ctrl_cmd->bullet_mode != BULLET_CONTINUE && shoot_ctrl_cmd->mode != SHOOT_STUCKING)    
	    {
		    abs_deltaSpeed = fabs(loader->measure.SpeedFilter);
		    abs_deltePlack = fabs(Angle_Target - loader->measure.Angle_DEG);
		    if(abs_deltePlack > fabs(load_delta_pos * 0.4f) && abs_deltaSpeed <= 25.0f )
			    Stuck_time ++;
			if(abs_deltaSpeed >= 25 && Stuck_time > 50)
			    Stuck_time = 0;
			/* 卡弹时可能卡在摩擦轮中 */
		    if(shoot_ctrl_cmd->mode == SHOOT_READY) 
		    {
			   if(abs(friction_l->measure.Speed) < 20 && abs(friction_r->measure.Speed) < 20 && abs(friction_u->measure.Speed) < 20)
			       Stuck_time ++;
		    }
	     }
		 else if(shoot_ctrl_cmd->bullet_mode == BULLET_CONTINUE && shoot_ctrl_cmd->mode == SHOOT_NOMAL) 
		 {
		     if (abs_deltaSpeed <= 50.0f)  // 连发时速度很小
				Stuck_time ++;
			 else
				Stuck_time = 0;
	     }
		 
	     /** 卡弹反转  **/
	     if(shoot_ctrl_cmd->mode == SHOOT_STUCKING) 
	     {
		   Stuck_Angle_Target = -Target_Continue_Speed * 0.5f;
		   DJIMotorOuterLoop(loader, SPEED_LOOP);
		   DJIMotorSetRef(loader, Stuck_Angle_Target);
	       Stuck_time ++;
			
		   if(Stuck_time >= Stuck_thre * 1.3f)
		       Stuck_time = 0;
	     }
	     else /* 未卡弹 */
	     {
		   /* 增加打弹标志位 */ 
		   if (shoot_ctrl_cmd->mode == SHOOT_READY )
			    Add_Angle_Flag = 1;
		   // 单发限制
		   if(fabs( DWT_GetTimeline_ms() - cooldown_start) < cooldown_tim)
			   Add_Angle_Flag = 0;
		   // 双发检测
		   if(DWT_GetTimeline_ms() - cooldown_start < 600)
			   double_heat_flag = 1;
		   else 
		       double_heat_flag = 0;
		   
		   /* 热量检测 */
		   if(shoot_ctrl_cmd->heat_limit_remain <= SHOOT_UNIT_HEAT_42MM*1.2f && shoot_ctrl_cmd->heat_limit_remain != 0) 
		   {
			   Add_Angle_Flag = 0;
			   shoot_ctrl_cmd->bullet_mode = BULLET_HOLDON;
		   }
		   
		   /* 开始打弹 */
		   if(Add_Angle_Flag == 1 && shoot_ctrl_cmd->mode == SHOOT_NOMAL && shoot_ctrl_cmd->bullet_mode != BULLET_CONTINUE)
		   {
			 switch (shoot_ctrl_cmd->bullet_mode)
			  {
				case BULLET_HOLDON :
					DJIMotorOuterLoop(loader, ANGLE_LOOP);  // 修改为角度控制
					DJIMotorSetRef(loader, Angle_Target);
				break;
				
				case BULLET_SINGLE : // 单发
					DJIMotorOuterLoop(loader, ANGLE_LOOP);
					Angle_Target = Angle_Target + load_delta_pos - Off_Angle;
					cooldown_tim   = 200; // 时间间隔
					cooldown_start = DWT_GetTimeline_ms();
				break;
				
				case BULLET_DOUBLE: // 双发
					DJIMotorOuterLoop(loader, ANGLE_LOOP);
					Angle_Target = Angle_Target + load_delta_pos * 2 - Off_Angle;
					cooldown_tim = 300;
					cooldown_start = DWT_GetTimeline_ms();
				break;
				
				case BULLET_TRIBLE:  // 三连发
					DJIMotorOuterLoop(loader, ANGLE_LOOP);
					Angle_Target = Angle_Target + load_delta_pos * 3 - Off_Angle;
					cooldown_tim = 450;
					cooldown_start = DWT_GetTimeline_ms();
				break;
				
				default:
				  DJIMotorOuterLoop(loader, ANGLE_LOOP);  // 修改为角度控制
				  Angle_Target = loader->measure.Angle_DEG;
				break;
			  }
			  Last_Target_Pluck = Angle_Target;
			  Add_Angle_Flag = 0;
			  Off_Angle = 0;
		   }
		  
		  /** 设置拨弹盘期望 **/
		  if (shoot_ctrl_cmd->bullet_mode == BULLET_CONTINUE) 
		  {
			  
			 DJIMotorOuterLoop(loader, SPEED_LOOP);
			 DJIMotorSetRef(loader, Target_Continue_Speed*0.4f);
			 Angle_Target      = loader->measure.Angle_DEG;
			 RAMP_Angle_Target = loader->measure.Angle_DEG;
		  }
		  else if(shoot_ctrl_cmd->bullet_mode != BULLET_CONTINUE) 
		  {
			 DJIMotorOuterLoop(loader, ANGLE_LOOP);
			 RAMP_Angle_Target = RAMP_float(Angle_Target, RAMP_Angle_Target, 16);
			 DJIMotorSetRef(loader, RAMP_Angle_Target);
		  }
	    }
    }
	Last_Bullet_mode = shoot_ctrl_cmd->bullet_mode;
}

/* 摩擦轮控制 */
void Shoot_friction_Update()
{
	shoot_ctrl_cmd->bullet_speed = 15; // 测试不同弹速时使用
	if(shoot_ctrl_cmd->mode == SHOOT_STOP) {
		DJIMotorSetRef(friction_l, 0);
		DJIMotorSetRef(friction_r, 0);
		DJIMotorSetRef(friction_u, 0);
	} else if (shoot_ctrl_cmd->mode == SHOOT_STUCKING) {
            /*发射机构电机全退弹*/
		DJIMotorSetRef(friction_l, -SHOOT_SPEED*0.8f);
		DJIMotorSetRef(friction_r,  SHOOT_SPEED*0.8f);
		DJIMotorSetRef(friction_u,  SHOOT_SPEED*0.8f);
	} else {
		switch(shoot_ctrl_cmd->bullet_speed) {
			case 15:
			    DJIMotorSetRef(friction_l,  SHOOT_SPEED + shoot_ctrl_cmd->speed_offset);
			    DJIMotorSetRef(friction_r, -SHOOT_SPEED - shoot_ctrl_cmd->speed_offset);
			    DJIMotorSetRef(friction_u, -SHOOT_SPEED - shoot_ctrl_cmd->speed_offset);
			    break;
			default :
			    DJIMotorSetRef(friction_l, 0);
			    DJIMotorSetRef(friction_r, 0);
			    DJIMotorSetRef(friction_u, 0);
			break;
		}
	}
}

/* 发射任务更新参数 */
void Shoot_Upeadt() 
{
	/* 接收消息 */
	if(SubGetMessage( shoot_sub, (void *)shoot_ctrl_cmd) != 1)
	{
   	    shoot_feedback_data.shoot_status = Device_Offline;
  	    shoot_ctrl_cmd->mode = SHOOT_ZERO_FORCE;
    }
	/* 电机在线检测 */
	if(loader->watchdog->state != Dog_Online)
	{
   	    shoot_feedback_data.shoot_status = Device_Offline;
	    shoot_ctrl_cmd->mode = SHOOT_ZERO_FORCE;
	}
	else
	{
	   shoot_feedback_data.shoot_status = Device_Online;
	}
    /* 记录摩擦轮速度 */
    shoot_feedback_data.fire_speed = (int16_t)((friction_l->measure.SpeedFilter - friction_r->measure.SpeedFilter - friction_u->measure.SpeedFilter) / 3.0f);

	// 电机控制
	if (shoot_ctrl_cmd->mode == SHOOT_ZERO_FORCE)
	{
		Shoot_STOP();
		RAMP_Angle_Target = Angle_Target = loader->measure.Angle_DEG;
		if(shoot_feedback_data.shoot_status == Device_Online) 
		{
		   Last_Target_Pluck = Angle_Target;
		   Off_Angle = 0;
		}
    }
	else 
	{
	    // 双发检测
	    DOUBLE_HEAL_Detect();
		// 拨弹盘控制
		Shoot_load_Update();
		// 摩擦轮控制
		Shoot_friction_Update();
		// 计算
		Shoot_Calc();
	}
   	// 推送消息
	shoot_feedback_data.mode = shoot_ctrl_cmd->mode;
   	shoot_feedback_data.bullet_mode = shoot_ctrl_cmd->bullet_mode;
   	PubPushMessage( shoot_pub, (void *)&shoot_feedback_data);
}

static void Shoot_Calc()
{
    static int16_t can1_send[4];
	/**  摩擦轮控制  **/
	if (friction_r->motor_controller.ref_speed != 0 ) {
		SMC_Calc(&shoot_smc[0], friction_l->motor_controller.ref_speed, friction_l->measure.SpeedFilter);
		SMC_Calc(&shoot_smc[1], friction_r->motor_controller.ref_speed, friction_r->measure.SpeedFilter);
		SMC_Calc(&shoot_smc[2], friction_u->motor_controller.ref_speed, friction_u->measure.SpeedFilter);
		can1_send[1] = (int16_t )shoot_smc[0].output;
		can1_send[2] = (int16_t )shoot_smc[1].output;
		can1_send[3] = (int16_t )shoot_smc[2].output;
	} else {
		PID_Control( friction_l->measure.SpeedFilter, friction_l->motor_controller.ref_speed, &Shoot_Speed_PID[0] );
		PID_Control( friction_r->measure.SpeedFilter, friction_r->motor_controller.ref_speed, &Shoot_Speed_PID[1] );
		PID_Control( friction_u->measure.SpeedFilter, friction_u->motor_controller.ref_speed, &Shoot_Speed_PID[2] );
		can1_send[1] = (int16_t )Shoot_Speed_PID[0].pid_out;
		can1_send[2] = (int16_t )Shoot_Speed_PID[1].pid_out;
		can1_send[3] = (int16_t )Shoot_Speed_PID[2].pid_out;
	}
	limit(can1_send[1] , RM3508_LIMIT, -RM3508_LIMIT);
    limit(can1_send[2] , RM3508_LIMIT, -RM3508_LIMIT);
    limit(can1_send[3] , RM3508_LIMIT, -RM3508_LIMIT);
    
	/**  拨弹盘控制  **/
	if(loader->motor_controller.motor_setting.close_loop_type >= ANGLE_LOOP) {
		PID_Control_Smis(loader->measure.Angle_DEG, loader->motor_controller.ref_position, &Pluck_Place_PIDS, loader->measure.SpeedFilter);
//		if(fabs(Pluck_Place_PIDS.pid_out) >= 150) {
//			SMC_Calc(&shoot_smc[3], Pluck_Place_PIDS.pid_out, loader->measure.SpeedFilter);
//		    can1_send[0] = (int16_t )shoot_smc[3].output;
//		} 
//		else {
			if(loader->measure.Speed <= -3750)
				limit(Pluck_Place_PIDS.pid_out, 5000, -3600);
		    PID_Control( loader->measure.Speed, Pluck_Place_PIDS.pid_out, &Pluck_Speed_PID);
			can1_send[0] = (int16_t )Pluck_Speed_PID.pid_out;
//		}
        limit(can1_send[0], RM3508_LIMIT, -RM3508_LIMIT);
	} else {
	    PID_Control(loader->measure.SpeedFilter, loader->motor_controller.ref_speed, &Pluck_Continue_PID);
   	   	can1_send[0] = (int16_t )Pluck_Continue_PID.pid_out;
        limit(can1_send[0], RM3508_LIMIT, -RM3508_LIMIT);
    }
	
#if SHOOT_MOVE
	DJIMotor_Transmit( &hcan1, 0x200, can1_send);
#endif
}

static void Shoot_STOP()
{
	static int16_t can1_send[4] = {0};
	DJIMotor_Transmit( &hcan1, 0x200, can1_send);
}

// 用于计算拨弹盘补偿角度
void Shoot_Get_OffsetAngle(int16_t load_delta_pos)
{
	if(Last_Bullet_mode != shoot_ctrl_cmd->bullet_mode && shoot_ctrl_cmd->mode != SHOOT_STUCKING)
        ABS(Last_Target_Pluck) <= 200 ? Off_Angle = 0 
		    : ( Off_Angle = (int16_t )(loader->measure.Angle_DEG - Last_Target_Pluck) % (int16_t)load_delta_pos);
}

/* 双发弹丸检测 
uint16_t DOUBLE_HEAT_DETECT = 100;    //!< @brief 双发检测时间
uint8_t double_heat_flag = 0;         //!< @brief 双发检测标志位
uint8_t double_heat_tims = 0;         //!< @brief 双发检测时间戳
*/                                
float real_detece_times = 0.0f, fir_current = 0.0f;
void DOUBLE_HEAL_Detect()
{
	static uint8_t detect_first_tims = 0;
	static uint16_t detect_num = 0;
	fir_current = (fabs(friction_u->measure.CurrentFilter) + fabs(friction_l->measure.CurrentFilter) + fabs(friction_r->measure.CurrentFilter) ) / 3.0f;
    if(double_heat_flag)
	{
	   // 检测第一发
	   if(fir_current > 2500 && detect_first_tims < 5) 
	   {
   	   	   detect_first_tims ++;	
		   SHOOT_UNIT_HEAT_START = DWT_GetTimeline_ms();
	   }
	   DOUBLE_HEAT_DETECT = DWT_GetTimeline_ms() - SHOOT_UNIT_HEAT_START;
	   if(detect_first_tims == 5 && fir_current < 5000 && DOUBLE_HEAT_DETECT  > 20) {
		   // 第一发检测完成 记录并更新时间戳
		   detect_first_tims ++;
		   SHOOT_FLAG_TIMS[SHOOT_FLAG_NUM++] = (uint16_t)DOUBLE_HEAT_DETECT;
		   SHOOT_UNIT_HEAT_START = DWT_GetTimeline_ms();
	    }
		// 等待第一发检测成功后进行第二发检测
		if(detect_first_tims > 5 && fir_current > 4500 && detect_first_tims < 11) {
			 detect_first_tims++;
			 SHOOT_DOUBLE_HEAT_START = DWT_GetTimeline_ms();
		}
		// 检测到第二发进行反转
		if(detect_first_tims >= 11) {
//			shoot_ctrl_cmd->mode = SHOOT_STUCKING;            
			if(DWT_GetTimeline_ms() - SHOOT_DOUBLE_HEAT_START > 100)
				detect_first_tims = 0;
		}
	} else {
		detect_first_tims = 0;
		DOUBLE_HEAT_DETECT = 0;
	}
	
	float a;
	for(uint8_t i ; i < SHOOT_FLAG_NUM; i++)
	   a += SHOOT_FLAG_TIMS[i]; 
	real_detece_times = a / SHOOT_FLAG_NUM;
}
