#include "DMmotor.h"

#include "cmsis_os.h"
#include "string.h"
#include "WatchDog.h"
#include "stdlib.h"

#include "motor.h"
HAL_StatusTypeDef DMMotor_send;
static uint8_t normal_idx = 0, idx = 0;  // 一拖四模式 类似于大疆电机
/* DM电机的实例,仅用来保存指针,内存的分配通过电机实例初始化时malloc分配内存 */
static DMMotor_Instance_1_4    *dm_motor_instance[DM_MOTOR_CNT];
static DMMotor_Instance_Normal *dm_motor_instance_Normal[DM_MOTOR_CNT];

/**
* DM电机一拖四通用一个协议 :0x3FE, 0x4FE
 * 反馈(rx_id): 0x300+id
 * can1: [0]:0x3FE,[1]:0x4FE
 * can2: [2]:0x3FE,[3]:0x4FE
**/
static CANInstance sender_assignment[4] = {
#if defined(CAN1)
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x3FE, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08},
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x4FE, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08},
#endif
#if defined(CAN2)
	[2] = {.can_handle = &hcan2, .txconf.StdId = 0x3FE, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08},
    [3] = {.can_handle = &hcan2, .txconf.StdId = 0x4FE, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08},
#endif
};

/*************	 传统模式	*****************/

/**
  * @brief 用于改变达妙电机控制模式 传统模式
**/
static void DMMotorSetMode( DMMotor_Mode_e cmd, DMMotor_Instance_Normal *motor)
{
	uint8_t tx_buff[8];
	memset(tx_buff, 0xff, 7); // 发送电机指令的时候前面7bytes都是0xff
	tx_buff[7] = (uint8_t )cmd; // 最后一位为命令
	CANTransmit(motor->motor_can_instance, 1, tx_buff);
}
/**
 * @brief 修改电机控制编码器0位点
 *
 */
void DMMotorCaliEncoder_Normal(DMMotor_Instance_Normal *motor)
 {
	DMMotorSetMode( DM_CMD_ZERO_POSITION, motor);
	DWT_Delay(0.1);
}        
/**
 * @brief 使能电机 清除错误
 *
*/
void DMMotorEnable_Normal(DMMotor_Instance_Normal *motor)
{
	DMMotorSetMode( DM_CMD_MOTOR_MODE, motor);
	DWT_Delay(0.1);
    motor->stop_flag = MOTOR_ENALBED;
}
void DMMotorClearError_Normal(DMMotor_Instance_Normal *motor)
{
	DMMotorSetMode( DM_CMD_CLEAR_ERROR, motor);
	DWT_Delay(0.1);
}
/**
 * @brief 停止电机
 *
*/
void DMMotorStop_Normal(DMMotor_Instance_Normal *motor)
{
	DMMotorSetMode(	DM_CMD_RESET_MODE, motor);
	DWT_Delay(0.1);
}
/**
 * @brief 看门狗回调
*/
uint8_t DMMotor_DetectCallback(void *id)
{
	DMMotor_Instance_Normal *motor = (DMMotor_Instance_Normal* )id;

	if(motor->measure.state == Motor_DM_Control_Status_ENABLE)
	{
	   return 1;
	}
	else if (motor->measure.state == Motor_DM_Control_Status_DISABLE)
	{   // 电机可能掉线, 使能电机
		DMMotorEnable_Normal(motor);
	}
	else
	{
		// 电机错误, 发送清除错误帧
		DMMotorClearError_Normal(motor);
	}
	return 0;
}
void DMMotor_StopCallback(void *id)
{
	DMMotor_Instance_Normal *motor = (DMMotor_Instance_Normal* )id;
	// 电机可能掉线, 使能电机
	DMMotorEnable_Normal(motor);
}
/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 */
static void MotorSenderGrouping_Normal(DMMotor_Instance_Normal *motor, CAN_Init_Config_s *_config)
{
    uint8_t motor_id = _config->tx_id; //达妙电机设置ID 
    
    switch (motor->motor_mode)
    {
		case (Motor_DM_Control_Method_NORMAL_MIT):
		{
			_config->tx_id = motor_id;
			break;
		}
		case (Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA):
		{
			_config->tx_id = motor_id + 0x100;
			break;
		}
		case (Motor_DM_Control_Method_NORMAL_OMEGA):
		{
			_config->tx_id = motor_id + 0x200;
			break;
		}
		case (Motor_DM_Control_Method_NORMAL_EMIT):
		{
			_config->tx_id = motor_id + 0x300;
			break;
		}
		default: break;
    }
	_config->rx_id = motor_id - 1;
	motor->message_num = motor_id;
}

/**
 * @brief 根据返回的can_instance对反馈报文进行解析
 *
 * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 */
static void DMMotorDecode_Normal(CANInstance *_instance)
{
	int32_t delta_encoder;
    uint16_t tmp_encoder, tmp_omega, tmp_torque;         // 用于暂存解析值,稍后转换成float数据,避免多次创建临时变量
    uint8_t *rxbuff = _instance->rx_buff;
    DMMotor_Instance_Normal *motor = (DMMotor_Instance_Normal *)_instance->id;
    DMMotor_Measure_Normal *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针
	
	measure->id = (rxbuff[0])&0x0F;
	measure->state = (rxbuff[0])>>4;

    measure->Last_Position = measure->Position;
    // 计算圈数与总角度值
    delta_encoder = tmp_encoder - measure->Last_Position;
    if (delta_encoder < -(1 << 15))
    {   // 正方向转过了一圈
        measure->Total_round++;
    }
    else if (delta_encoder > (1 << 15))
    {   // 反方向转过了一圈
        measure->Total_round--;
    }
    measure->Total_Position = measure->Total_round * (1 << 16) + tmp_encoder - ((1 << 15) - 1);

    tmp_encoder = (uint16_t)((rxbuff[1] << 8) | rxbuff[2]);
    measure->Position = uint_to_float(tmp_encoder, DM_P_MIN, DM_P_MAX, 16);

    tmp_omega = (uint16_t)((rxbuff[3] << 4) | rxbuff[4] >> 4);
    measure->Velocity = uint_to_float(tmp_omega, DM_V_MIN, DM_V_MAX, 12);

    tmp_torque = (uint16_t)(((rxbuff[4] & 0x0f) << 8) | rxbuff[5]);
    measure->Torque = uint_to_float(tmp_torque, DM_T_MIN, DM_T_MAX, 12);

    measure->Mos_Temp = (float)rxbuff[6];
    measure->Rotor_Temp  = (float)rxbuff[7];

    Feed_Dog(motor->watchdog);
}

/**
 * @brief 电机初始化 返回一个电机实例
**/
DMMotor_Instance_Normal *DMMotorInit_Normal(Motor_Init_config_s *_config,DM_Control_Method_e motor_mode)
{
	DMMotor_Instance_Normal *instance = (DMMotor_Instance_Normal* )RMLIB_MALLOC(sizeof(DMMotor_Instance_Normal));
	memset(instance, 0, sizeof(DMMotor_Instance_Normal));
	
	instance->motor_mode = motor_mode;
	instance->other_angle_feedback_ptr = _config->other_angle_feedback_ptr;
	instance->other_speed_feedback_ptr = _config->other_speed_feedback_ptr;
    
    MotorSenderGrouping_Normal(instance, &_config->can_init_config);
    _config->can_init_config.can_module_callback = DMMotorDecode_Normal;
	_config->can_init_config.id                  = instance;
	instance->motor_can_instance = CANRegister(_config->can_init_config);
	
    instance->motor_controller = create_controller(&_config->contruller_config);
	
	_config->dog_init_config.watch_callback = DMMotor_StopCallback;
	_config->dog_init_config.feed_callback  = DMMotor_DetectCallback;
	_config->dog_init_config.Max_num        = 5;
	_config->dog_init_config.owner_id       = instance;
	instance->watchdog = WatchDog_Init(_config->dog_init_config); // 注册看门狗
	
    DMMotorEnable_Normal(instance);
	dm_motor_instance_Normal[normal_idx ++] = instance;
	return instance;
}

void DMMotor_Feedback(DMMotor_Instance_Normal *motor)
{
	uint8_t tx_buff[8];
	uint16_t tmp_angle, tmp_omega, tmp_torque, tmp_k_p, tmp_k_d;
	
	DMMotor_Send_s *motor_set = &motor->motor_set;
    // 电机控制
    switch (motor->motor_mode)
    {
		case (Motor_DM_Control_Method_NORMAL_MIT):
		{

			tmp_angle = float_to_uint(motor_set->Control_Angle, DM_P_MIN, DM_P_MAX, 16);
			tmp_omega = float_to_uint(motor_set->Control_Omega, DM_V_MIN, DM_V_MAX, 12);
			tmp_torque = float_to_uint(motor_set->Control_Torque, DM_T_MIN, DM_T_MAX, 12);
			tmp_k_p = float_to_uint(motor_set->K_P, 0, 500.0f, 12);
			tmp_k_d = float_to_uint(motor_set->K_D, 0, 5.0f, 12);

			tx_buff[0] = (uint8_t)(tmp_angle >> 8);
			tx_buff[1] = (uint8_t)(tmp_angle);
			tx_buff[2] = (uint8_t)(tmp_omega >> 4);
			tx_buff[3] = (uint8_t)(((tmp_omega & 0xF) << 4) | (tmp_k_p >> 8));
			tx_buff[4] = (uint8_t)(tmp_k_p);
			tx_buff[5] = (uint8_t)(tmp_k_d >> 4);
			tx_buff[6] = (uint8_t)(((tmp_k_d & 0xF) << 4) | (tmp_torque >> 8));
			tx_buff[7] = (uint8_t)(tmp_torque);
			
			CANTransmit(motor->motor_can_instance, 0.5f, tx_buff);
			break;
		}
		case (Motor_DM_Control_Method_NORMAL_ANGLE_OMEGA):
		{
			DM_Tx_Data_Normal_Angle_Omega *tx_data = (DM_Tx_Data_Normal_Angle_Omega *)tx_buff;
			tx_data->Control_Angle = motor_set->Control_Angle;
			tx_data->Control_Omega = motor_set->Control_Omega; 

			CANTransmit(motor->motor_can_instance, 0.5f, tx_buff);
			break;
		}
		case (Motor_DM_Control_Method_NORMAL_OMEGA):
		{
			DM_Tx_Data_Normal_Omega *tx_data = (DM_Tx_Data_Normal_Omega *)tx_buff;

			tx_data->Control_Omega = motor_set->Control_Omega;

			CANTransmit(motor->motor_can_instance, 0.5f, tx_buff);
			break;
		}
		case (Motor_DM_Control_Method_NORMAL_EMIT):
		{
			DM_Tx_Data_Normal_EMIT *tx_data = (DM_Tx_Data_Normal_EMIT *)tx_buff;

			tx_data->Control_Angle = motor_set->Control_Angle;
			tx_data->Control_Omega = (uint16_t)(motor_set->Control_Omega * 100.0f);
			tx_data->Control_Current = (uint16_t)(motor_set->Control_Current / Theoretical_Output_Current_Max * 10000.0f);

			CANTransmit(motor->motor_can_instance, 0.5f, tx_buff);
			break;
		}
		default: break;
    }
}

/*****************	 一拖四模式	*****************/

/**
 * @brief 用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DMMotorControl()使用
 *        flag的初始化在 MotorSenderGrouping()中进行
 *         [0]:0x3FE,[1]:0x4FE,[2]:0x3FE,[3]:0x4FE
**/
static uint8_t sender_enable_flag[4] = {NULL};   // 被填充为1表示此处有电机被注册，为0表示此处没有电机被注册
/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 */
static void MotorSenderGrouping_1_4(DMMotor_Instance_1_4 *motor, CAN_Init_Config_s *_config)
{
    uint8_t motor_id = _config->tx_id - 1; //达妙电机设置ID 
    uint8_t motor_send_num;
    uint8_t motor_grouping;
    
	if (motor_id <= 4) {
		motor_send_num = motor_id;
		motor_grouping = _config->can_handle == &hcan1 ? 0 : 2;  // 0x3FE
	} else {
		motor_send_num = motor_id - 4;
		motor_grouping = _config->can_handle == &hcan1 ? 1 : 3;  // 0x4FE
	}
	_config->rx_id = 0x300 + motor_id + 1;
    sender_enable_flag[motor_grouping] = 1 ;// 设置送标志位,防止发送空帧 可能会重复定义
	
	motor->message_num = motor_send_num;
	motor->sender_group = motor_grouping;
}

/**
 * @brief 电机一拖四接收CAN反馈报文  ID为 0x301~0x308
 *
 * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 */
static void DMMotorDecode_1_4(CANInstance *_instance)
{
    // 数据处理过程
	uint8_t *rxbuff = _instance->rx_buff;
	DMMotor_Instance_1_4 *motor = (DMMotor_Instance_1_4 *)_instance->id;
	DMMotor_Measure_1_4 *measure = &(motor->measure); // 将can实例中保存的id转换成电机实例的指针
	Feed_Dog(motor->watchdog); // 喂狗
	
	measure->MchanicalAngle = (uint16_t )((rxbuff[0] << 8) | rxbuff[1]);
	measure->Speed = (int16_t )((rxbuff[2] << 8) | rxbuff[3]) / 100.0f;  // 真实速度
	measure->TorqueCurrent = (int16_t )((rxbuff[4] << 8) | rxbuff[5]) / 1000.0f;
    measure->Mos_temperature =  (int16_t )(rxbuff[6]);
    measure->ERROR_Handle =  (int16_t )(rxbuff[7]);
	
    // 多圈角度计算,前提是假设两次采样间电机转过的角度小于180°
    int16_t diff = measure->MchanicalAngle - measure->LastMchanicalAngle;
    if(diff != measure->MchanicalAngle)
        measure->flag = 1;
    if(measure->flag == 1) {
		if (diff > 4000)
			measure->round --;
		else if (diff < -4000)
			measure->round ++;
	}

	measure->Angle = measure->MchanicalAngle + measure->round * 8192;
	measure->Angle_DEG =( measure->Angle - measure->MotorCenter ) * 0.0439453125f;
	/* 一阶低通滤波器 */
	measure->SpeedFilter =  (1.0f - DMMOTOR_SPEED_SMOOTH_COEF) * measure->SpeedFilter +
                           DMMOTOR_SPEED_SMOOTH_COEF * (float)(measure->Speed);
    measure->CurrentFilter = (1.0f - DMMOTOR_CURRENT_SMOOTH_COEF) * measure->CurrentFilter +
                            DMMOTOR_CURRENT_SMOOTH_COEF * (float)(measure->TorqueCurrent);	
    
	measure->LastAngle = measure->Angle;
	measure->LastMchanicalAngle = measure->MchanicalAngle ;
}

void DMMotorChangeFeed(DMMotor_Instance_1_4 *motor, Feedback_Source_e type)
{
	motor->motor_controller->motor_setting.angle_feedback_source  = type;
	motor->motor_controller->motor_setting.speed_feedback_source  = type;
}

/* 设置期望值 */
void DMMotorSetRef(DMMotor_Instance_1_4 *motor, float ref)
{
	motor->motor_controller->motor_setting.close_loop_type >= ANGLE_LOOP ? (motor->motor_controller->ref_position = ref )
		: (motor->motor_controller->ref_speed = ref );
}

/* 使能电机 */
void DMMotorEnable(DMMotor_Instance_1_4 *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/* 停止电机 */
void DMMotorStop(void *id)//不使用使能模式是因为需要收到反馈
{
	DMMotor_Instance_1_4 *motor = (DMMotor_Instance_1_4* )id;
    motor->stop_flag = MOTOR_STOP;
}

/* 修改电机的实际闭环对象 */
void DMMotorOuterLoop(DMMotor_Instance_1_4 *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_controller->motor_setting.close_loop_type = outer_loop;
}

/* 作为看门狗回调 */
uint8_t DMMotorErroText(void *id)
{
	DMMotor_Instance_1_4 *motor = (DMMotor_Instance_1_4* )id;
	if (motor->measure.ERROR_Handle != 1 || motor->measure.ERROR_Handle != 0) {
          motor->stop_flag = MOTOR_STOP;
	}
	return 1;
}
/**
 * @brief 电机初始化 返回一个电机实例
 *
**/
DMMotor_Instance_1_4 *DMMotorInit_1_4(Motor_Init_config_s *_config)
{
	DMMotor_Instance_1_4 *instance = (DMMotor_Instance_1_4* )RMLIB_MALLOC(sizeof(DMMotor_Instance_1_4));
	memset(instance, 0, sizeof(DMMotor_Instance_1_4));
	
	instance->other_angle_feedback_ptr = _config->other_angle_feedback_ptr;
	instance->other_speed_feedback_ptr = _config->other_speed_feedback_ptr;
    
    MotorSenderGrouping_1_4(instance, &_config->can_init_config);
    _config->can_init_config.can_module_callback = DMMotorDecode_1_4;
	_config->can_init_config.id                  = instance;
	instance->motor_can_instance = CANRegister(_config->can_init_config);
	
    instance->motor_controller = create_controller(&_config->contruller_config);

	_config->dog_init_config.watch_callback = DMMotorStop;
//	_config->dog_init_config.feed_callback  = DMMotorErroText;
	_config->dog_init_config.Max_num        = 5;
	_config->dog_init_config.owner_id       = instance;
	instance->watchdog = WatchDog_Init(_config->dog_init_config); // 注册看门狗
	instance->measure.MotorCenter = _config->contruller_config.MotorCenter;
	
    DMMotorEnable(instance);
	dm_motor_instance[idx++] = instance;
	return instance;
}
