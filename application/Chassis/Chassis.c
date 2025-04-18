#include "Chassis.h"

#include "pub_sub.h"
#include "robot_def.h"
#include "PID.h"
#include "powercontroller.h"
#include "bsp_dwt.h"
#include "slope.h"
#include "ins_task.h"
#include "Matrix.h"
#include "user_lib.h"

extern Gimbal_data_t    receive_vision;
extern Gimbal_action_t  receive_action; 

// 轮向电机摩擦阻力连续化的角速度阈值
float Wheel_Resistance_Omega_Threshold = 0.5f;
// 轮向电机动摩擦阻力电流值
float Dynamic_Resistance_Wheel_Current[4] = {0.16f,
											 0.16f,
											 0.16f,
											 0.16f};
float Dynamic_Resistance_Wheel_Current_Rpm[4] = {0.4f,
											 0.4f,
											 0.4f,
											 0.4f};
float Dynamic_Resistance_Wheel_Current_Rpm_Final[4];

/* 斜坡函数（int16_t） */
static Slope_s slope_X ,slope_Y ,slope_Omega;

static PID Chassis_Speed_PID[4] = {{.Kp = 30.0f, .Ki = 0.00, .Kd = 0, .interlimit = 3000, .outlimit = 6000, .DeadBand = 0.0f, .inter_threLow = 500, .inter_threUp = 1000},  /* 左前轮 */
								{.Kp = 30.0f, .Ki = 0.00, .Kd = 0, .interlimit = 3000, .outlimit = 6000, .DeadBand = 0.0f, .inter_threLow = 500, .inter_threUp = 1000},  /* 右前轮 */
								{.Kp = 30.0f, .Ki = 0.00, .Kd = 0, .interlimit = 3000, .outlimit = 6000, .DeadBand = 0.0f, .inter_threLow = 500, .inter_threUp = 1000},  /* 右后轮 */
								{.Kp = 30.0f, .Ki = 2.00, .Kd = 0, .interlimit = 3000, .outlimit = 6000, .DeadBand = 0.0f, .inter_threLow = 500, .inter_threUp = 1000}}; /* 左后轮 */

static PID TargetVelocity_PID[3] = {{.Kp = 50.0f, .Ki = 0.00, .Kd = 0, .interlimit = 3000, .outlimit = 16000, .DeadBand = 0.0f, .inter_threLow = 500, .inter_threUp = 1000},
	                                {.Kp = 50.0f, .Ki = 0.00, .Kd = 0, .interlimit = 3000, .outlimit = 16000, .DeadBand = 0.0f, .inter_threLow = 500, .inter_threUp = 1000},
		                            {.Kp = 20.0f, .Ki = 0.00, .Kd = 0, .interlimit = 3000, .outlimit = 16000, .DeadBand = 0.0f, .inter_threLow = 500, .inter_threUp = 1000}};
// 底盘状态
static DeviceState_e chassis_state;
/*  底盘部分 *lf, *rf, *lb, *rb, */
static DJIMotor_Instance *chassis_motor[4];
static PowerObj_s chassis_power[4];  // 功率重分配所需变量
static float currentAv[4];           // 电机转速(rad/s)
  int16_t Can2Send_Chassis[4];  // 向电机发送电流值
static attitude_t *ins;              // 底盘陀螺仪

static Publisher_t *chassis_pub;            // 云台控制消息发布者
static Subscriber_t *chassis_sub;           // 云台反馈信息订阅者

static Chassis_ctrl_cmd_t chassis_cmd_recv;       // 底盘发送CMD的控制信息
static Chassis_upload_t   chassis_feedback_data;  // 底盘接收的反馈信息
/* 功能变量 */
    // 底盘位移变量
static float omega_Factor, xy_Factor;
static Chassis_speed_s measuredVelocity, targetVelocity;
static float  force_target[4]; //  四个轮子需要的扭矩 
static float speed_target[4];  // 底盘速度解算后的临时输出,待进行限幅 单位(rad/s)

static float GetChassisMotorPower(float speed, float current); /* 用发送电流值计算功率 */
static void Get_Chassis_Inter(void);  /* 斜坡相对于底盘的角度矢量 */
  // 斜坡法向量在底盘方向向量
static float  Slope_Direction_X, Slope_Direction_Y, Slope_Direction_Z;
static float  Slope_theta, Slope_beta;
static float  sqrt_theat, sqrt_beta;
extern Referee_data_t Referee_SendData;

/**  底盘初始化  **/
void Chassis_Init()
{
		// 底盘期望速度斜坡
	Slope_Init(&slope_X, 6.0f / 1000.0f, 40.0f / 1000.0f, SLOPE_FIRST_REAL);
	Slope_Init(&slope_Y, 6.0f / 1000.0f, 40.0f / 1000.0f, SLOPE_FIRST_REAL);
	Slope_Init(&slope_Omega, 3.0f * PI / 1000.0f, 30.0f * PI / 1000.0f, SLOPE_FIRST_REAL);
	
Motor_Init_config_s LF_config = {
	.motor_type = RM3508,
	.dog_init_config.dog_name = "LF",  
	.can_init_config = {
		.can_handle = &hcan2,
		.tx_id = 1,
		.rx_id = 201,
	},
	.contruller_config = {
		.setting_config = {
			.feedback_reverse_flag = FEEDBACK_NORMAL,
			.motor_reverse_flag    = MOTOR_NORMAL,   // 电机不反转
			.angle_feedback_source = MOTOR_FEED,     // 电机反馈控制
			.speed_feedback_source = MOTOR_FEED,
			.close_loop_type = SPEED_LOOP,
			.feedfoward_type = NONE_FEEDFORWARD, // 不使用前馈类型
			.control_mode    = PID_MODEL,
		},
	},
};
    chassis_motor[lf] = DJI_Motor_create(&LF_config);
	
Motor_Init_config_s LB_config = {
	.motor_type = RM3508,
	.dog_init_config.dog_name = "LB",  
	.can_init_config = {
		.can_handle = &hcan2,
		.tx_id = 2,
		.rx_id = 202,
	},
	.contruller_config = {
		.setting_config = {
			.feedback_reverse_flag = FEEDBACK_NORMAL,
			.motor_reverse_flag    = MOTOR_NORMAL,   // 电机不反转
			.angle_feedback_source = MOTOR_FEED,  //  电机反馈控制
			.speed_feedback_source = MOTOR_FEED,     
			.close_loop_type = SPEED_LOOP,
			.feedfoward_type = NONE_FEEDFORWARD, // 不使用前馈类型
			.control_mode    = PID_MODEL,
		},
	},
};
    chassis_motor[lb] = DJI_Motor_create(&LB_config);

Motor_Init_config_s RB_config = {
	.motor_type = RM3508,
	.dog_init_config.dog_name = "RB",
	.can_init_config = {
		.can_handle = &hcan2,
		.tx_id = 3,
		.rx_id = 203,
	},
	.contruller_config = {
		.setting_config = {
			.feedback_reverse_flag = FEEDBACK_NORMAL,
			.motor_reverse_flag    = MOTOR_NORMAL,   // 电机不反转
			.angle_feedback_source = MOTOR_FEED,     // 电机反馈控制
			.speed_feedback_source = MOTOR_FEED,
			.close_loop_type = SPEED_LOOP,
			.feedfoward_type = NONE_FEEDFORWARD, // 不使用前馈类型
			.control_mode    = PID_MODEL,
		},
	},
};
    chassis_motor[rb] =  DJI_Motor_create(&RB_config);

Motor_Init_config_s RF_config = {
	.motor_type = RM3508,
	.dog_init_config.dog_name = "RF",
	.can_init_config = {
		.can_handle = &hcan2,
		.tx_id = 4,
		.rx_id = 204,
	},
	.contruller_config = {
		.setting_config = {
			.feedback_reverse_flag = FEEDBACK_NORMAL,
			.motor_reverse_flag    = MOTOR_NORMAL,   // 电机不反转
			.angle_feedback_source = MOTOR_FEED,     // 电机反馈控制
			.speed_feedback_source = MOTOR_FEED,
			.close_loop_type = SPEED_LOOP,
			.feedfoward_type = NONE_FEEDFORWARD,    // 不使用前馈类型 
			.control_mode    = PID_MODEL,
		},
	},
};
    chassis_motor[rf] = DJI_Motor_create(&RF_config);
    
	memset(&chassis_feedback_data, 0, sizeof(Chassis_upload_t));
	chassis_pub = PubRegister("chassis_feed", sizeof(Chassis_upload_t) );
	chassis_sub = SubRegister("chassis_cmd", sizeof(Chassis_ctrl_cmd_t) );
	
	memset(&chassis_cmd_recv, 0, sizeof(Chassis_ctrl_cmd_t));
    
    DJIMotorClose(chassis_motor[lf]); // 关闭电机自动计算PID
	DJIMotorClose(chassis_motor[rf]);
	DJIMotorClose(chassis_motor[lb]);
	DJIMotorClose(chassis_motor[rb]);
	/**  功率计算  **/
    Manager_Init(chassis_motor, HERO, RLS_Enable, 0.0091f, 350, 0.7f); //    0.0187587207f  1.18e-05f // 0.   0.000148522042                1.16999999e-07
      
    /** 移动速度计算 线速度m/s  **/
#if USE_MECANUM_CHASSIS
    xy_Factor =  RADIUS_WHEEL / 4.0f;
    omega_Factor  = -RADIUS_WHEEL / (4.0f * (HALF_WHEEL_BASE + HALF_TRACK_WIDTH)) / 2.0f;
#else
    // clang-format off
    omega_Factor   = -RADIUS_WHEEL / 4.0f * CHASSSIWHEEL_TO_CENTER;
    xy_Factor  =  RADIUS_WHEEL * SQRT2 / 4.0f;
    // clang-format on
#endif
    ins = INS_Init();
    /* 超级电容初始化 */
	
}


/**
 * @brief 底盘自身运动学正解算
 *
 */
void Self_Resolution(void)
{
    for(uint8_t i = 0; i < 4 ; i++)  
	   currentAv[i] = chassis_motor[i]->measure.Speed * PI / 30.0f;  // 将各个轮组转速RPM转化为角速度
	// 底盘实际速度解算
#ifdef USE_MECANUM_CHASSIS // 麦轮
    measuredVelocity.X = (currentAv[2] + currentAv[3] - currentAv[1] - currentAv[0]) * xy_Factor / MOTOR_DECELE_RATIO;
#else
    measuredVelocity.X = (currentAv[2] + currentAv[3] - currentAv[0] - currentAv[1]) * xy_Factor / MOTOR_DECELE_RATIO;
#endif
    measuredVelocity.Y = (currentAv[0] - currentAv[1] - currentAv[2] + currentAv[3]) * xy_Factor / MOTOR_DECELE_RATIO;
    measuredVelocity.Omega = (currentAv[0] + currentAv[1] + currentAv[2] + currentAv[3]) * omega_Factor / MOTOR_DECELE_RATIO;
	 /* 获取当前底盘与斜坡的矢量关系 */
	Get_Chassis_Inter();
}

/**
 * @brief 运动学逆解算 
 *
 */
void Kinematics_Inverse_Resolution() 
{	
	static Chassis_speed_s planningVelocity;
    planningVelocity.X = Slope_Calc(&slope_X, targetVelocity.X, measuredVelocity.X);
    planningVelocity.Y = Slope_Calc(&slope_Y, targetVelocity.Y, measuredVelocity.Y);
    planningVelocity.Omega = Slope_Calc(&slope_Omega, targetVelocity.Omega , measuredVelocity.Omega);
    
	// 根据当前底盘线速度进行运动学逆结算
	PID_Control(measuredVelocity.X, planningVelocity.X, &TargetVelocity_PID[0]);
	PID_Control(measuredVelocity.Y, planningVelocity.Y, &TargetVelocity_PID[1]);
	PID_Control(measuredVelocity.Omega, planningVelocity.Omega, &TargetVelocity_PID[2]);
    
	// 计算每个轮子的期望角速度
#ifdef USE_MECANUM_CHASSIS // 麦轮
    speed_target[lf] =( -planningVelocity.X + planningVelocity.Y + planningVelocity.Omega * LF_CENTER ) / RADIUS_WHEEL;
    speed_target[lb] =( -planningVelocity.X - planningVelocity.Y + planningVelocity.Omega * LB_CENTER ) / RADIUS_WHEEL;
    speed_target[rb] =( +planningVelocity.X - planningVelocity.Y + planningVelocity.Omega * RB_CENTER ) / RADIUS_WHEEL;
    speed_target[rf] =( +planningVelocity.X + planningVelocity.Y + planningVelocity.Omega * RF_CENTER ) / RADIUS_WHEEL;
#else  // 全向轮
    speed_target[lf] = (-SQRT2 / 2.0f * planningVelocity.X + SQRT2 / 2.0f * planningVelocity.Y + planningVelocity.Omega * CHASSSIWHEEL_TO_CENTER) / RADIUS_WHEEL;
    speed_target[lb] = (-SQRT2 / 2.0f * planningVelocity.X - SQRT2 / 2.0f * planningVelocity.Y + planningVelocity.Omega * CHASSSIWHEEL_TO_CENTER) / RADIUS_WHEEL;
    speed_target[rb] = (+SQRT2 / 2.0f * planningVelocity.X - SQRT2 / 2.0f * planningVelocity.Y + planningVelocity.Omega * CHASSSIWHEEL_TO_CENTER) / RADIUS_WHEEL;
    speed_target[rf] = (+SQRT2 / 2.0f * planningVelocity.X + SQRT2 / 2.0f * planningVelocity.Y + planningVelocity.Omega * CHASSSIWHEEL_TO_CENTER) / RADIUS_WHEEL;
#endif
}

/**
 * @brief 动力学逆解算
 * 
 */
void Dynamics_Inverse_Resolution()
{
    float force_x, force_y, torque_omega; // 底盘运动方向上需要的牵引力
    
    force_x = TargetVelocity_PID[0].pid_out;
    force_y = TargetVelocity_PID[1].pid_out;
    torque_omega = TargetVelocity_PID[2].pid_out;
	
	// 解算到每个轮子的具体摩擦力
    force_target[lf] =( -force_x + force_y + torque_omega * LF_CENTER ) * RADIUS_WHEEL;
    force_target[lb] =( -force_x - force_y + torque_omega * LB_CENTER ) * RADIUS_WHEEL;
    force_target[rb] =( +force_x - force_y + torque_omega * RB_CENTER ) * RADIUS_WHEEL;
    force_target[rf] =( +force_x + force_y + torque_omega * RF_CENTER ) * RADIUS_WHEEL;
}


/**
 * @brief 正常运行模式下，麦轮底盘的解算函数
 *
 * @param X方向的速度（向右为正，m/s）,Y方向的速度（向前为正，m/s）, Roate 绕底盘转轴的角速度, 最终解算出电机期望角速度
**/
void mecanum_calculate()
{
	static Chassis_speed_s planningVelocity;
    /* 速度规划 */
	if(receive_action.move_status == rotate) {
		slope_X.Increase_Value  = 30.0f / 1000.0f;
		slope_Y.Increase_Value  = 30.0f / 1000.0f;
        slope_Omega.Increase_Value = 35.0f / 1000.0f;
		slope_Omega.Decrease_Value = 8.0f / 1000.0f;
	} else {
		slope_X.Increase_Value  = 36.0f / 1000.0f;
		slope_Y.Increase_Value  = 36.0f / 1000.0f;
        slope_Omega.Increase_Value = 25.0f / 1000.0f;
		slope_Omega.Decrease_Value = 15.0f / 1000.0f;
	} 
	
	planningVelocity.X = Slope_Calc(&slope_X, targetVelocity.X, measuredVelocity.X);
    planningVelocity.Y = Slope_Calc(&slope_Y, targetVelocity.Y, measuredVelocity.Y);
    planningVelocity.Omega = Slope_Calc(&slope_Omega, targetVelocity.Omega , measuredVelocity.Omega);

#ifdef USE_MECANUM_CHASSIS // 麦轮
    speed_target[lf] =( -planningVelocity.X + planningVelocity.Y + planningVelocity.Omega * LF_CENTER ) / RADIUS_WHEEL;
    speed_target[lb] =( -planningVelocity.X - planningVelocity.Y + planningVelocity.Omega * LB_CENTER ) / RADIUS_WHEEL;
    speed_target[rb] =( +planningVelocity.X - planningVelocity.Y + planningVelocity.Omega * RB_CENTER ) / RADIUS_WHEEL;
    speed_target[rf] =( +planningVelocity.X + planningVelocity.Y + planningVelocity.Omega * RF_CENTER ) / RADIUS_WHEEL;
#else  // 全向轮
    speed_target[lf] = (-SQRT2 / 2.0f * planningVelocity.X + SQRT2 / 2.0f * planningVelocity.Y + planningVelocity.Omega * CHASSSIWHEEL_TO_CENTER) / RADIUS_WHEEL;
    speed_target[lb] = (-SQRT2 / 2.0f * planningVelocity.X - SQRT2 / 2.0f * planningVelocity.Y + planningVelocity.Omega * CHASSSIWHEEL_TO_CENTER) / RADIUS_WHEEL;
    speed_target[rb] = (+SQRT2 / 2.0f * planningVelocity.X - SQRT2 / 2.0f * planningVelocity.Y + planningVelocity.Omega * CHASSSIWHEEL_TO_CENTER) / RADIUS_WHEEL;
    speed_target[rf] = (+SQRT2 / 2.0f * planningVelocity.X + SQRT2 / 2.0f * planningVelocity.Y + planningVelocity.Omega * CHASSSIWHEEL_TO_CENTER) / RADIUS_WHEEL;
#endif
}

/**
 * @brief 底盘飞坡处理 仅飞坡和爬坡模式调用
 * @todo 待底盘添加陀螺仪之后进行斜坡角度判断 加入角度判断后或可以不用限制模式 可在车底盘中央添加一个测距判断前轮是否先飞出地面
*/
float cos_theat, sin_theat, cos_beta, sin_beta, forward_offset, back_offset;
void Chassis_Flay()
{   
	DEADLINE_LIMIT(Slope_theta, 3);
	limit(Slope_theta, 45, -45);
	if(fabs(Slope_theta) >  5)
	{
		cos_theat = arm_cos_f32(Slope_theta*ANGLE2RADIAN); sin_theat = arm_sin_f32(Slope_theta*ANGLE2RADIAN);
		cos_beta  = arm_cos_f32(Slope_beta*ANGLE2RADIAN);  sin_beta  = arm_sin_f32(Slope_beta*ANGLE2RADIAN);

		Dynamic_Resistance_Wheel_Current_Rpm_Final[lf] = Dynamic_Resistance_Wheel_Current[lf] + Dynamic_Resistance_Wheel_Current_Rpm[lf]*(1-sin_theat*0.6f);
		Dynamic_Resistance_Wheel_Current_Rpm_Final[lb] = Dynamic_Resistance_Wheel_Current[lb] + Dynamic_Resistance_Wheel_Current_Rpm[lb]*(1+sin_theat*0.6f);
		Dynamic_Resistance_Wheel_Current_Rpm_Final[rb] = Dynamic_Resistance_Wheel_Current[rb] + Dynamic_Resistance_Wheel_Current_Rpm[rb]*(1+sin_theat*0.6f);
		Dynamic_Resistance_Wheel_Current_Rpm_Final[rf] = Dynamic_Resistance_Wheel_Current[rf] + Dynamic_Resistance_Wheel_Current_Rpm[rf]*(1-sin_theat*0.6f);
	} else 
		for(uint8_t i = 0; i < 4 ; i++)
	  Dynamic_Resistance_Wheel_Current_Rpm_Final[i] = Dynamic_Resistance_Wheel_Current[i];

} 

/**
 * @brief 底盘功率重分配
 * 
**/
float rls_power, new_power;
float *controlleredOutput;
void chassis_powerlimit()
{
	static float Rampspeed[4];
	float speed_sum = 0.0f, current_sum = 0.0f;   // 打滑处理用
	static PowerObj_s *pobjs[4] = {&chassis_power[0], &chassis_power[1], &chassis_power[2], &chassis_power[3]};
	// 设定期望值
	for (int8_t i = 0; i < 4; i ++) {
		Rampspeed[i] = RAMP_int16( speed_target[i] * MOTOR_DECELE_RATIO, Rampspeed[i], 100.0f);
		if( fabs(speed_target[i]) <= 3.0f && fabs(chassis_motor[i]->measure.SpeedFilter) < 100.0f)
			Can2Send_Chassis[i] = Chassis_Speed_PID[i].pid_out = 0;
		else
			Can2Send_Chassis[i] = PID_Control(currentAv[i], Rampspeed[i] , &Chassis_Speed_PID[i]);
		// 底盘摩擦力补偿
		if( speed_target[i] > Wheel_Resistance_Omega_Threshold) {
		    Can2Send_Chassis[i] += Dynamic_Resistance_Wheel_Current_Rpm_Final[i] * CURRENT2OUT;
		} else if( speed_target[i] < -Wheel_Resistance_Omega_Threshold) {
		    Can2Send_Chassis[i] -= Dynamic_Resistance_Wheel_Current_Rpm_Final[i] * CURRENT2OUT;
		} else {
            Can2Send_Chassis[i] += speed_target[i] / Wheel_Resistance_Omega_Threshold * Dynamic_Resistance_Wheel_Current_Rpm_Final[i] * CURRENT2OUT;
		}
		
		speed_sum += ABS(chassis_motor[i]->measure.SpeedFilter);
		current_sum += ABS(Can2Send_Chassis[i]);
	}
    
    /* 悬空轮处理（依据电流与转速）与功率计算 */
	for (uint8_t j = 0; j < 4; j++) {
		if(ABS(chassis_motor[j]->measure.SpeedFilter) >= speed_sum * 0.4f && ABS(Can2Send_Chassis[j]) < current_sum * 0.3f)
 			  Can2Send_Chassis[j] *= 0.3;  // PID输出为零
        if(DJIMotor_detect(chassis_motor[j])) {
            chassis_power[j].motor_state = 0;
		} else {
		    chassis_power[j].motor_state = 1;
		}
		
		/**  在底盘功率限制模式中进行功率重分配  **/
		chassis_power[j].curAv = currentAv[j];
		chassis_power[j].setAv = Rampspeed[j];
		chassis_power[j].pidOutput = Can2Send_Chassis[j];
		chassis_power[j].errorAv = fabs(chassis_power[j].setAv - chassis_power[j].curAv)/19.203f;
		chassis_power[j].pidMaxOutput = 15500;
	}
	    
	// 底盘功率限制
	if (chassis_cmd_recv.dispatch_mode == chassis_dispatch_without_acc_limit)
	{
        controlleredOutput = getControlledOutput(pobjs);               
		rls_power = controlleredOutput[4];
		new_power = controlleredOutput[5];
	     for (uint8_t i = 0; i < 4 ; i ++) {
  	        Can2Send_Chassis[i] = controlleredOutput[i];
		 }
	}
}

/**
 * @brief 底盘任务
**/
void Chassis_Upeadt()
{
     uint8_t mtor_detect = 0;	
	/**  处理接收后信息  **/
	if(SubGetMessage(chassis_sub, (void *)&chassis_cmd_recv) != 1 ) {
		chassis_cmd_recv.mode = CHASSIS_ZERO_FORCE;
	} else  {
		if(chassis_cmd_recv.Close_flag == 1 )
		    chassis_cmd_recv.mode = CHASSIS_ZERO_FORCE;
		else {
            if(Referee_SendData.level_gain == 0)
                Referee_SendData.level_gain = 1;
			chassis_cmd_recv.mode =  CHASSIS_NORMAL;
			targetVelocity.X      = -Referee_SendData.level_gain * chassis_cmd_recv.vx/1500.0f;
			targetVelocity.Y      =  Referee_SendData.level_gain * chassis_cmd_recv.vy/1500.0f;
            if(receive_action.move_status == rotate )
                targetVelocity.Omega  =  Referee_SendData.level_gain * chassis_cmd_recv.rotate/1500.0f;
             else (targetVelocity.Omega  =  chassis_cmd_recv.rotate/1500.0f);
			chassis_cmd_recv.dispatch_mode = chassis_dispatch_without_acc_limit;
		}
    }
	
	/** 电机在线检测  **/
	for(uint8_t i = 0; i < 4; i ++)
	   if( !DJIMotor_detect(chassis_motor[i]))
		   mtor_detect ++;	 
	if (mtor_detect >= 2) { //底盘轮组掉线超过2个时停止移动
		chassis_state = Device_Offline;
		chassis_cmd_recv.mode = CHASSIS_ZERO_FORCE;
	    mtor_detect = 0;
	} else {
	    chassis_state = Device_Online;
        mtor_detect = 0;
	}
	
	// 自身运动学正解算
	Self_Resolution();
	
	// 底盘控制
	if (chassis_cmd_recv.mode == CHASSIS_ZERO_FORCE) {
	  for (uint8_t j = 0; j < 4; j++)
			Can2Send_Chassis[j] = 0;  // PID输出为零
	} else {     // 正常工作
//        Kinematics_Inverse_Resolution();
//        Dynamics_Inverse_Resolution();
		
		Chassis_Flay();
        mecanum_calculate();           // 麦轮解算
        chassis_powerlimit();          // 底盘功率限制
	}

#if CHASSIS_MOVE
	/**  底盘电机发送函数  **/
	DJIMotor_Transmit(&hcan2, 0x200, Can2Send_Chassis);
#endif
	
	/** 推送消息 **/
	chassis_feedback_data.chassis_status = chassis_state;
	chassis_feedback_data.mode = chassis_cmd_recv.mode;
	PubPushMessage( chassis_pub, (void *)&chassis_feedback_data);
}

/* 用发送电流值计算功率 */
float GetChassisMotorPower(float speed, float current)
{
    return (SS * speed * speed +
            SCI * speed * current +
            CCI * current * current +
            Constant);
}

/* 上坡时 根据底盘陀螺仪求解得斜坡法向量相对于底盘系的分量（外旋） */
void Slope_Direction()
{	
	Slope_Direction_X = arm_sin_f32(ins->Pitch*ANGLE2RADIAN) * arm_cos_f32(ins->Roll*ANGLE2RADIAN);
    Slope_Direction_Y = -arm_sin_f32(ins->Roll*ANGLE2RADIAN);
    Slope_Direction_Z = arm_cos_f32(ins->Pitch*ANGLE2RADIAN) * arm_cos_f32(ins->Roll*ANGLE2RADIAN);
}

void Get_Chassis_Inter()
{ 	
	Slope_Direction();
	arm_sqrt_f32(Slope_Direction_Y * Slope_Direction_Y + Slope_Direction_X * Slope_Direction_X, &sqrt_theat);
    Slope_theta = atan2(sqrt_theat, Slope_Direction_Z)/ANGLE2RADIAN;
	
	Slope_beta = atan2(Slope_Direction_Y / arm_sin_f32(Slope_theta*ANGLE2RADIAN), Slope_Direction_X / arm_sin_f32(Slope_theta*ANGLE2RADIAN))/ANGLE2RADIAN;
}


/** 底盘相对于世界坐标系旋转矩阵（内旋） **/
//  static float cos_alpha, sin_alpha, cos_beta, sin_beta, cos_theta, sin_theta ;
	
//	cos_alpha = arm_cos_f32(ins->Yaw*ANGLE2RADIAN) ; sin_alpha = arm_sin_f32(ins->Yaw*ANGLE2RADIAN);
//	cos_beta = arm_cos_f32(ins->Pitch*ANGLE2RADIAN) ; sin_beta = arm_sin_f32(ins->Pitch*ANGLE2RADIAN);
//	cos_theta = arm_cos_f32(ins->Roll*ANGLE2RADIAN) ; sin_theta = arm_sin_f32(ins->Roll*ANGLE2RADIAN);
	
//	// 底盘旋转矩阵（内旋）
//	Chassis.data_[0] = cos_alpha * cos_beta;
//	Chassis.data_[1] = -sin_alpha * cos_theta + cos_alpha * sin_beta * sin_theta;
//	Chassis.data_[2] = sin_alpha * sin_theta + cos_alpha * sin_beta * cos_beta;
//	Chassis.data_[3] = sin_alpha * cos_beta;
//	Chassis.data_[4] = cos_alpha * cos_theta + sin_alpha * sin_beta * sin_theta;
//	Chassis.data_[5] = -cos_alpha * sin_theta + sin_alpha * sin_beta * cos_theta;
//	Chassis.data_[6] = -sin_beta;
//	Chassis.data_[7] = cos_beta * sin_theta;
//	Chassis.data_[8] = cos_beta * cos_theta;
//	// 斜坡在底盘惯性系下的旋转矩阵
//	Slope_.data_[0] = cos_beta;
//	Slope_.data_[1] = sin_beta * sin_theta;
//	Slope_.data_[2] = sin_beta * cos_theta;
//	Slope_.data_[3] = 0;
//	Slope_.data_[4] = cos_beta;
//	Slope_.data_[5] = -sin_theta;
//	Slope_.data_[6] = -sin_beta;
//	Slope_.data_[7] = sin_beta * cos_theta;
//	Matrixf_Operator_mult(&Chassis, &Slope_, &Final);
	
//	arm_sqrt_f32(Chassis.data_[2] * Chassis.data_[2] + Chassis.data_[5] * Chassis.data_[5], &sqrt_theat );	
//	Slope_theta = atan2(sqrt_theat, Chassis.data_[8]) / PI * 180.0f;

/**
	// 底盘旋转矩阵（内旋）
	Chassis_Matrix[0] = cos_alpha * cos_beta;
	Chassis_Matrix[1] = -sin_alpha * cos_theta + cos_alpha * sin_beta * sin_theta;
	Chassis_Matrix[2] = sin_alpha * sin_theta + cos_alpha * sin_beta * cos_beta;
	Chassis_Matrix[3] = sin_alpha * cos_beta;
	Chassis_Matrix[4] = cos_alpha * cos_theta + sin_alpha * sin_beta * sin_theta;
	Chassis_Matrix[5] = -cos_alpha * sin_theta + sin_alpha * sin_beta * cos_theta;
	Chassis_Matrix[6] = -sin_beta;
	Chassis_Matrix[7] = cos_beta * sin_theta;
	Chassis_Matrix[8] = cos_beta * cos_theta;
	// 斜坡在底盘惯性系下的旋转矩阵
	Slope_Matrix[0] = cos_beta;
	Slope_Matrix[1] = sin_beta * sin_theta;
	Slope_Matrix[2] = sin_beta * cos_theta;
	Slope_Matrix[3] = 0;
	Slope_Matrix[4] = cos_beta;
	Slope_Matrix[5] = -sin_theta;
	Slope_Matrix[6] = -sin_beta;
	Slope_Matrix[7] = sin_beta * cos_theta;
**/

