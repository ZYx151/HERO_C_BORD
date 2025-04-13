#include "Shoot.h"
#include "RMQueue.h"
#include "motor.h"
#include "PID.h"
#include "user_lib.h"

#include "usb_typdef.h"

/*  发射部分 */
extern int16_t SHOOT_OFFSET;
float fire_speed_nofil[3] = {0};

static DJIMotor_Instance *friction_r, *friction_l, *friction_u, *loader;
static Publisher_t      *shoot_pub;          // 云台控制消息发布者
static Subscriber_t     *shoot_sub;          // 云台反馈信息订阅者
static Shoot_ctrl_cmd_t *shoot_ctrl_cmd;     // 云台接收CMD的控制信息
static Shoot_upload_t    shoot_feedback_data;// 云台发布的反馈信息

/* 用于发射 */
static float Angle_Target = 0;                //!< @brief 拨弹盘期望角度
static float Last_Target_Pluck = 0, Off_Angle = 0;   //!< @brief 发弹补偿
static float RAMP_Angle_Target = 0;           //!< @brief 拨弹盘期望斜坡角度
static Bullet_mode_e Last_Bullet_mode;        //!< @brief 用于计算拨弹盘角度补偿
static uint32_t cooldown_start;               //!< @brief 冷却起始时间点
static uint32_t cooldown_tim;                 //!< @brief 冷却时间
// 补偿控制            
//RMQueue_Handle *bullet_speed_queue;
static uint32_t shoot_time;  // 发射时间计数
static uint16_t buttle_cnt;
static uint16_t last_bullet_cnt;
static float shoot_speed_reduce;
/**  其他功能函数  **/
static void Shoot_load_Update(void);     // 拨弹盘控制
static void Shoot_friction_Update(void); // 摩擦轮控制
static void Shoot_Calc(void);
static void Shoot_STOP(void);
static void Shoot_Get_OffsetAngle(float load_delta_pos);
	
static Smc shoot_smc[3];
static PID Shoot_Speed_PID[3] = {{.Kp = 15, .Ki = 0.0f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 16000, .DeadBand = 0.50f, .inter_threLow = 500, .inter_threUp = 1000},            //摩擦轮左
                          {.Kp = 15, .Ki = 0.0f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 16000, .DeadBand = 0.50f, .inter_threLow = 500, .inter_threUp = 1000},                  //摩擦轮右
						  {.Kp = 15, .Ki = 0.0f, .Kd = 0.0f, .interlimit = 3000, .outlimit = 16000, .DeadBand = 0.50f, .inter_threLow = 500, .inter_threUp = 1000}};                 // 摩擦轮上

							  
static PID_Smis Pluck_Place_PIDS = {.Kp = 6.0f, .Ki = 0, .Kd = 0.4f, .interlimit = 3000, .outlimit = 16000, .DeadBand = 0.0f, .inter_threLow = 500, .inter_threUp = 1000};         //拨弹盘单发位置环
static PID Pluck_Speed_PID = {.Kp = 20.0f, .Ki = 0.0f, .Kd = 0.0f, .interlimit = 4000, .outlimit = 15000, .DeadBand = 0.0f, .inter_threLow = 20, .inter_threUp = 5000};                   //拨弹盘单发速度环
static PID Pluck_Continue_PID = {.Kp = 10, .Ki = 0, .Kd = 0, .interlimit = 3000, .outlimit = 15000, .DeadBand = 5.0f, .inter_threLow = 500, .inter_threUp = 1000};               //拨弹盘连发模式

void Shoot_Init()
{
    SMC_SetConfig(&shoot_smc[0].config, 3.0f, 20, 100, ReachingLaw_sqrt, 15000);
    SMC_SetConfig(&shoot_smc[1].config, 3.0f, 20, 100, ReachingLaw_sqrt, 15000);
    SMC_SetConfig(&shoot_smc[2].config, 3.0f, 20, 100, ReachingLaw_sqrt, 15000);
    
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
  cooldown_start = cooldown_tim = 0;
}
/*
	PubPushMessage(gimbal_pub, (void *)&gimbal_feed);
	SubGetMessage(gimbal_sub, gimbal_get_control);
*/
static float abs_deltePlack, abs_deltaSpeed, loader_angle_deg;

static void Shoot_load_Update()
{
    static uint16_t  Stuck_time   = 0, continur_tims = 0;//!< @brief 检测卡弹的时间 连发计时
	static uint8_t Add_Angle_Flag = 0;               	 //!< @brief 拨弹盘角度+1标志位
    static float Stuck_Angle_Target = 0;                 //!< @brief 拨弹盘卡弹退弹期望角度
    static const uint16_t Stuck_thre = 800;              //!< @brief 卡弹阈值，卡弹超过此时间便认为卡弹
	static float Target_Continue_Speed = -360.0f * REDUCTION_RATIO_WHEEL / SHOOT_NUM_PER_CIRCLE * 1.0f;  // 播弹盘连发速度
	// 设定拨弹盘转动角度
//	static int16_t load_delta_pos = -360.0f * 19.203f / SHOOT_NUM_PER_CIRCLE * 0.98f;
	static int16_t load_delta_pos = -1385;
	
    /* 暂停模式 */	
	if(shoot_ctrl_cmd->mode == SHOOT_STOP) 
	{
		shoot_ctrl_cmd->bullet_mode = BULLET_HOLDON;
		/** 卡弹时间清零  **/
		Stuck_time = 0; 
//        ABS(Last_Target_Pluck) <= 200 ? Off_Angle = 0 
//		    : ( Off_Angle = (int16_t )(loader->measure.Angle_DEG - Last_Target_Pluck) % (int16_t)load_delta_pos);
		DJIMotorOuterLoop(loader, ANGLE_LOOP);   // 修改为角度控制
		DJIMotorSetRef(loader, Angle_Target);
	}
	else 
	{
		if(shoot_ctrl_cmd->mode == SHOOT_STUCKING) 
		{
		    if(Stuck_time < 800) {
				Stuck_time = 800;
			} else { 
				Stuck_time ++;
			}
		}
//		if(shoot_ctrl_cmd->mode == SHOOT_AIM) {
//			if(Aim_Ref.fire_on ) 
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
		/** 计算角度补偿 **/
		Shoot_Get_OffsetAngle(load_delta_pos);
		/** 卡弹检测 **/
	    if(shoot_ctrl_cmd->bullet_mode != BULLET_CONTINUE && shoot_ctrl_cmd->mode != SHOOT_STUCKING)    
	    {
		     abs_deltaSpeed = fabs(loader->measure.SpeedFilter);
		     abs_deltePlack = fabs(Angle_Target - loader->measure.Angle_DEG);
		     if(abs_deltePlack > fabs(load_delta_pos * 0.6f) && abs_deltaSpeed <= 50.0f )
			     Stuck_time ++;
			 if(abs_deltaSpeed >= 200 && Stuck_time > 100)
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
		   Stuck_Angle_Target = -Target_Continue_Speed * 0.4f;
		   DJIMotorOuterLoop(loader, SPEED_LOOP);
		   DJIMotorSetRef(loader, Stuck_Angle_Target);
	       Stuck_time ++;
			
		   if(Stuck_time >= Stuck_thre * 1.25f)
		       Stuck_time = 0;
	     }
	     else /* 未卡弹 */
	     {
		   /* 增加打弹标志位 */ 
		   if (shoot_ctrl_cmd->mode == SHOOT_READY)
			    Add_Angle_Flag = 1;
		   /* 开始打弹 */
		   if (Add_Angle_Flag == 1 && shoot_ctrl_cmd->mode == SHOOT_NOMAL && shoot_ctrl_cmd->bullet_mode != BULLET_CONTINUE) 
			  {
				 switch (shoot_ctrl_cmd->bullet_mode)
				  {
					case BULLET_HOLDON :
						DJIMotorOuterLoop(loader, ANGLE_LOOP); // 修改为角度控制
						Angle_Target = loader->measure.Angle_DEG;
						DJIMotorSetRef(loader, Angle_Target);
					break;
					
					case BULLET_SINGLE : // 单发
						DJIMotorOuterLoop(loader, ANGLE_LOOP);
						Angle_Target = loader->measure.Angle_DEG + load_delta_pos - Off_Angle;
						cooldown_tim   = 100; // 时间间隔
					break;
					
					case BULLET_DOUBLE: // 双发
						DJIMotorOuterLoop(loader, ANGLE_LOOP);
						Angle_Target = loader->measure.Angle_DEG + load_delta_pos * 2 - Off_Angle;
						cooldown_tim = 200;
					break;
					
					case BULLET_TRIBLE:  // 三连发
						DJIMotorOuterLoop(loader, ANGLE_LOOP);
						Angle_Target = loader->measure.Angle_DEG + load_delta_pos * 3 - Off_Angle;
						cooldown_tim = 300;
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
				 DJIMotorSetRef(loader, Target_Continue_Speed);
				 Angle_Target      = loader->measure.Angle_DEG;
				 RAMP_Angle_Target = loader->measure.Angle_DEG;
			   }
			  else if ( shoot_ctrl_cmd->bullet_mode != BULLET_CONTINUE) 
			  {
				 DJIMotorOuterLoop(loader, ANGLE_LOOP);
				 RAMP_Angle_Target = RAMP_float(Angle_Target, RAMP_Angle_Target, 22);
				 DJIMotorSetRef(loader, RAMP_Angle_Target);
    		  }
	      }
     }
	 Last_Bullet_mode = shoot_ctrl_cmd->bullet_mode;
}

void Shoot_friction_Update()
{
	shoot_ctrl_cmd->bullet_speed = 15;        // 测试
	if (shoot_ctrl_cmd->mode == SHOOT_STOP) {
		DJIMotorSetRef(friction_l, 0);
		DJIMotorSetRef(friction_r, 0);
		DJIMotorSetRef(friction_u, 0);
	} else if (shoot_ctrl_cmd->mode == SHOOT_STUCKING) {  // SHOOT_SPEED
            /*发射机构电机全退弹*/
		DJIMotorSetRef(friction_l, -SHOOT_SPEED*0.2f);
		DJIMotorSetRef(friction_r,  SHOOT_SPEED*0.2f);
		DJIMotorSetRef(friction_u,  SHOOT_SPEED*0.2f);
	} else {
		switch (shoot_ctrl_cmd->bullet_speed) {
			case 20:
			    DJIMotorSetRef(friction_l,  SHOOT_SPEED+500);
			    DJIMotorSetRef(friction_r, -SHOOT_SPEED-500);
			    DJIMotorSetRef(friction_u, -SHOOT_SPEED-500);
                shoot_feedback_data.real_bullet_speed = 18.2f;  // 此处填写该case下调得实际弹速的典型值
			    break;
			case 15:
			    DJIMotorSetRef(friction_l,  SHOOT_SPEED + SHOOT_OFFSET);
			    DJIMotorSetRef(friction_r, -SHOOT_SPEED - SHOOT_OFFSET);
			    DJIMotorSetRef(friction_u, -SHOOT_SPEED - SHOOT_OFFSET);
                shoot_feedback_data.real_bullet_speed = 15.2f;  // 此处填写该case下调得实际弹速的典型值
			    break;
			case 0:
			    DJIMotorSetRef(friction_l, 0);
			    DJIMotorSetRef(friction_r, 0);
			    DJIMotorSetRef(friction_u, 0);
                shoot_feedback_data.real_bullet_speed = 0.0f;
			    break;
			default :  // 使用最低弹速保持一致
			    DJIMotorSetRef(friction_l, -SHOOT_SPEED);
			    DJIMotorSetRef(friction_r, SHOOT_SPEED);
			    DJIMotorSetRef(friction_u, -SHOOT_SPEED);
                shoot_feedback_data.real_bullet_speed = 14.2f;  // 此处填写该case下调得实际弹速的典型值
				break;
		}
	}
}

int32_t test_pluck_target, test_pluck_now;
void Shoot_Upeadt() 
{
	// 发弹计数及弹速显示
	static float last_bullet_speed_fdb, init = 0;

	/* 接收消息 */
	if (SubGetMessage( shoot_sub, (void *)shoot_ctrl_cmd) != 1)
	{
   	    shoot_feedback_data.shoot_status = Device_Offline;
  	    shoot_ctrl_cmd->mode = SHOOT_ZERO_FORCE;
    }
	/* 电机在线检测 */
	if (loader->watchdog->state != Dog_Online)
	{
   	    shoot_feedback_data.shoot_status = Device_Offline;
	    shoot_ctrl_cmd->mode = SHOOT_ZERO_FORCE;
	}
	else
	{
	   shoot_feedback_data.shoot_status = Device_Online;
	}
	
//    /* 弹频控制 */
//	if (shoot_ctrl_cmd->bullet_speed_fdb != last_bullet_speed_fdb) {
//		buttle_cnt ++;
//		shoot_time = HAL_GetTick();
////		RMQueuePush(bullet_speed_queue, &shoot_ctrl_cmd->bullet_speed_fdb);
//		last_bullet_speed_fdb = shoot_ctrl_cmd->bullet_speed_fdb;
//	} 
	
	// 电机控制
	if (shoot_ctrl_cmd->mode == SHOOT_ZERO_FORCE)
	{
		Shoot_STOP();
		RAMP_Angle_Target = Angle_Target = loader->measure.Angle_DEG;
		Off_Angle = 0;
		if(shoot_feedback_data.shoot_status == Device_Online)
			Last_Target_Pluck = Angle_Target;
    }
	else 
	{
		Shoot_load_Update();
		Shoot_friction_Update();
		Shoot_Calc();
	}
    // 记录摩擦轮速度
	fire_speed_nofil[0] =  friction_l->measure.SpeedFilter;
	fire_speed_nofil[1] = -friction_r->measure.SpeedFilter;
	fire_speed_nofil[2] = -friction_u->measure.SpeedFilter;
	// 推送消息
   	shoot_feedback_data.mode = shoot_ctrl_cmd->mode;
   	shoot_feedback_data.bullet_mode = shoot_ctrl_cmd->bullet_mode;
   	PubPushMessage( shoot_pub, (void *)&shoot_feedback_data);
	
	test_pluck_now = loader->measure.Angle_DEG;
	test_pluck_target = loader->motor_controller.ref_position;
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
	if( loader->motor_controller.motor_setting.close_loop_type >= ANGLE_LOOP) {
		PID_Control_Smis(loader->measure.Angle_DEG, loader->motor_controller.ref_position, &Pluck_Place_PIDS, loader->measure.SpeedFilter);
		PID_Control( loader->measure.SpeedFilter, Pluck_Place_PIDS.pid_out, &Pluck_Speed_PID);
  	    can1_send[0] = (int16_t )Pluck_Speed_PID.pid_out;
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
void Shoot_Get_OffsetAngle(float load_delta_pos)
{
	if(Last_Bullet_mode != shoot_ctrl_cmd->bullet_mode)
        ABS(Last_Target_Pluck) <= 200 ? Off_Angle = 0 
		    : ( Off_Angle = (int16_t )(loader->measure.Angle_DEG - Last_Target_Pluck) % (int16_t)load_delta_pos);
}
