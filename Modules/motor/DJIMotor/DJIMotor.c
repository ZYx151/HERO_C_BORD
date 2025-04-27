#include "DJIMotor.h"
#include "bsp_dwt.h"
#include "CANDrive.h"

#define VELOCITY_WINDOW 5  // 用于计算电机线速度

static uint8_t idx = 0; // register idx,是该文件的全局电机索引,在注册时使用
/* DJI电机的实例,仅用来保存指针,内存的分配通过电机实例初始化时malloc分配内存 */
static DJIMotor_Instance *motor_instance[DJI_MOTOR_CNT] = {NULL};  // motor_instances用于存放DJIMotor_Instance*类型的指向电机实体的指针，在每个电机初始化时被填充

/**
  *
 * C610(m2006)/C620(m3508):0x1ff,0x200;
 * GM6020:0x1ff,0x2ff
 * 反馈(rx_id): GM6020: 0x204+id ; C610/C620: 0x200+id
 * can1: [0]:0x1FF,[1]:0x200,[2]:0x2FF
 * can2: [3]:0x1FF,[4]:0x200,[5]:0x2FF
**/
 CANInstance sender_assignment[6] = {
#if defined(CAN1)
    [0] = {.can_handle = &hcan1, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .txconf.TransmitGlobalTime = DISABLE},
    [1] = {.can_handle = &hcan1, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .txconf.TransmitGlobalTime = DISABLE},
    [2] = {.can_handle = &hcan1, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .txconf.TransmitGlobalTime = DISABLE},
#endif
#if defined(CAN2)
	[3] = {.can_handle = &hcan2, .txconf.StdId = 0x1ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .txconf.TransmitGlobalTime = DISABLE},
    [4] = {.can_handle = &hcan2, .txconf.StdId = 0x200, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .txconf.TransmitGlobalTime = DISABLE},
    [5] = {.can_handle = &hcan2, .txconf.StdId = 0x2ff, .txconf.IDE = CAN_ID_STD, .txconf.RTR = CAN_RTR_DATA, .txconf.DLC = 0x08, .txconf.TransmitGlobalTime = DISABLE},
#endif
};

/**
 * @brief 用于确认是否有电机注册到sender_assignment中的标志位,防止发送空帧,此变量将在DJIMotorControl()使用
 *        flag的初始化在 MotorSenderGrouping()中进行
 *         [0]:0x1FF,[1]:0x200,[2]:0x2FF[3]:0x1FF,[4]:0x200,[5]:0x2FF
 *  
*/
static uint8_t sender_enable_flag[6] = {NULL};   // 被填充为1表示此处有电机被注册，为0表示此处没有电机被注册

/**
 * @brief 根据电调/拨码开关上的ID,根据说明书的默认id分配方式计算发送ID和接收ID,
 *        并对电机进行分组以便处理多电机控制命令
 */
static void MotorSenderGrouping(DJIMotor_Instance *motor, CAN_Init_Config_s *_config)
{
    uint8_t motor_id = _config->tx_id - 1 ; //电机绿灯闪动次数 减一方便计算
    uint8_t motor_send_num;
    uint8_t motor_grouping;
	
    switch (motor->DJI_motor){
		case M2006  :
		case RM3508 :
			if (motor_id < 4){
				motor_send_num = motor_id;
				motor_grouping = _config->can_handle == &hcan1 ? 1 : 4 ;  // tx_id == 0x200
 			} else {
				motor_send_num = motor_id - 4;
				motor_grouping = _config->can_handle == &hcan1 ? 0 : 3 ;  // tx_id == 0x1ff
			}
			  // 计算接收id并设置分组发送id
			_config->rx_id = 0x200 + motor_id + 1; 
            sender_enable_flag[motor_grouping] = 1 ;// 设置发送标志位,防止发送空帧
		 	
			motor->message_num = motor_send_num;
			motor->sender_group = motor_grouping;
			break;
			
		case GM6020 :
		
			if (motor_id < 4) {
				motor_send_num = motor_id;
				motor_grouping = _config->can_handle == &hcan1 ? 0 : 3;  // tx_id == 0x1ff
			} else {
				motor_send_num = motor_id - 4;
				motor_grouping = _config->can_handle == &hcan1 ? 2 : 5;  // tx_id == 0x2ff
			}
			
			_config->rx_id = 0x204 + motor_id + 1;   // 把ID+1,进行分组设置
			
			sender_enable_flag[motor_grouping] = 1 ;// 设置发送标志位,防止发送空帧 可能会重复定义
            
			motor->message_num = motor_send_num;
			motor->sender_group = motor_grouping;
			break;
			
		default:   break;
	}
}


/**
 * @brief 根据返回的can_instance对反馈报文进行解析
 *
 * @param _instance 收到数据的instance,通过遍历与所有电机进行对比以选择正确的实例
 */
static void DecodeDJIMotor(CANInstance *_instance)
{
    // 这里对can instance的id进行了强制转换,从而获得电机的instance实例地址
    // _instance指针指向的id是对应电机instance的地址,通过强制转换为电机instance的指针,再通过->运算符访问电机的成员motor_measure,最后取地址获得指针
    uint8_t *rxbuff = _instance->rx_buff;
    DJIMotor_Instance *motor = (DJIMotor_Instance *)_instance->id;
    DJIMotor_Measure_s *measure = &motor->measure; // measure要多次使用,保存指针减小访存开销	
	Feed_Dog(motor->watchdog); // 喂狗
	// 解析数据并对电流和速度进行滤波
	measure->MchanicalAngle = ((uint16_t)rxbuff[0]) << 8 | rxbuff[1];
	measure->Speed          = (int16_t)(rxbuff[2] << 8 | rxbuff[3]);
	measure->TorqueCurrent  = (uint16_t)(rxbuff[4] << 8 | rxbuff[5]);
	motor->DJI_motor == M2006 ? (measure->temp = 0) : (measure->temp = rxbuff[6]) ;
    
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
    measure->Angle = measure->round * 8192 + measure->MchanicalAngle;
	measure->Angle_DEG = (float)((measure->Angle - measure->MotorCenter)*0.0439453125f);
	measure->SpeedFilter =  (1.0f - SPEED_SMOOTH_COEF) * measure->SpeedFilter +
                           (float)(SPEED_SMOOTH_COEF * measure->Speed);
    measure->CurrentFilter = (1.0f - CURRENT_SMOOTH_COEF) * measure->CurrentFilter +
                            CURRENT_SMOOTH_COEF * (float)(measure->TorqueCurrent);	
	measure->LastAngle = measure->Angle;
	measure->LastMchanicalAngle = measure->MchanicalAngle;	
	/* 用于计算电机线速度 */
//    int16_t Delta = measure->Angle - measure->LastAngle;

//	if (motor->Position_Queue->size != VELOCITY_WINDOW) {
//		 float *now = RMQueuePop(motor->Position_Queue);
//		motor->position_sum -= *now;
//	}
//	RMQueuePush(motor->Position_Queue, &Delta) ;
//	motor->position_sum += Delta;
//	float vnow = motor->position_sum * 43.9453125f / motor->Position_Queue->size;
//	measure->velocity = 0.2f * measure->velocity + 0.8f * vnow;
}

/* 电流只能通过电机自带传感器监测*/
void DJIMotorChangeFeed(DJIMotor_Instance *motor, Feedback_Source_e type)
 {
	motor->motor_controller.motor_setting.angle_feedback_source  = type;
	motor->motor_controller.motor_setting.speed_feedback_source  = type;
}

/* 停止电机 */
void DJIMotorStop(void *id)
{
	DJIMotor_Instance *motor = (DJIMotor_Instance *)id;
    motor->stop_flag = MOTOR_STOP;
}

/* 关闭电机 */
void DJIMotorClose(DJIMotor_Instance *motor )
   {	
    motor->stop_flag = MOTOR_CLOSE;
}

/* 使能电机 */
void DJIMotorEnable(DJIMotor_Instance *motor)
{
    motor->stop_flag = MOTOR_ENALBED;
}

/* 修改电机的实际闭环对象 */
 void DJIMotorOuterLoop(DJIMotor_Instance *motor, Closeloop_Type_e outer_loop)
{
    motor->motor_controller.motor_setting.close_loop_type	= outer_loop;
}

/* 设置期望值 */
 void DJIMotorSetRef(DJIMotor_Instance *motor, float ref)
{
	motor->motor_controller.motor_setting.close_loop_type >= ANGLE_LOOP ? (motor->motor_controller.ref_position = ref )
		: (motor->motor_controller.ref_speed = ref );
}

/* 获取电机测量值 */
int16_t DJIMotor_GetTorque(DJIMotor_Instance *motor)
{
	return motor->measure.TorqueCurrent;
}
int16_t DJIMotor_GetRPMSpeed(DJIMotor_Instance *motor)
{
	return motor->measure.Speed;
}
int16_t DJIMotor_GetAngle(DJIMotor_Instance *motor)
{
	return motor->measure.Angle_DEG;
}

RM_Status DJIMotor_detect(DJIMotor_Instance *motor)
{
	return ( motor->watchdog->state == Dog_Online ) ? RM_SUCCESS : RM_ERROR;
}

/**  电机发送函数  **/
HAL_StatusTypeDef DJIMotor_Transmit(CAN_HandleTypeDef *hcan,uint32_t StdId, int16_t *Data)                                                                      
{
    uint8_t temp[8];
    temp[0] = (uint8_t)(Data[0] >> 8);
    temp[1] = (uint8_t)(Data[0] & 0xff);
    temp[2] = (uint8_t)(Data[1] >> 8);
    temp[3] = (uint8_t)(Data[1] & 0xff);
    temp[4] = (uint8_t)(Data[2] >> 8);
    temp[5] = (uint8_t)(Data[2] & 0xff);
    temp[6] = (uint8_t)(Data[3] >> 8);
    temp[7] = (uint8_t)(Data[3] & 0xff);
    return CAN_Send_StdDataFrame(hcan, StdId, temp);
	
}
//	config->contruller_config = &instance->measure->Angle_DEG;
//	config->contruller_config.speed_feedback_ptr = &instance->measure->Speed;
//	config->contruller_config.current_feedback_ptr = &instance->measure->TorqueCurrent;


/**
 * @brief 电机初始化 返回一个电机实例
 *
**/
DJIMotor_Instance* DJI_Motor_create(Motor_Init_config_s* _config)
{
    DJIMotor_Instance *instance = (DJIMotor_Instance *)RMLIB_MALLOC(sizeof(DJIMotor_Instance));
    memset(instance, 0, sizeof(DJIMotor_Instance));
    
	instance->DJI_motor = _config->motor_type;
	instance->other_angle_feedback_ptr = _config->other_angle_feedback_ptr;
	instance->other_speed_feedback_ptr = _config->other_speed_feedback_ptr;
    
    MotorSenderGrouping(instance, &_config->can_init_config);
    _config->can_init_config.can_module_callback = DecodeDJIMotor;
	_config->can_init_config.id                  = instance;
	instance->motor_can_instance = CANRegister(_config->can_init_config);
    
    instance->motor_controller = *create_controller(&_config->contruller_config);
	
	_config->dog_init_config.watch_callback = DJIMotorStop;
	_config->dog_init_config.GPIOx          = GPIOC;
	_config->dog_init_config.Max_num        = 15;
	_config->dog_init_config.owner_id       = instance;
	instance->watchdog = WatchDog_Init(_config->dog_init_config); // 注册看门狗
	instance->measure.MotorCenter = _config->contruller_config.MotorCenter;
	
	motor_instance[idx++] = instance;
	DJIMotorEnable(instance);
	
   return instance;
}

/** 为所有电机实例计算三环PID,发送控制报文 **/
void DJIMotorControl()
{
	// 直接保存一次指针引用从而减小访存的开销,同时提高可读性
	uint8_t group, num;			// 电机组号和组内编号
	uint16_t can_set;			// 电机控制CAN发送设定值    
	DJIMotor_Instance *motor ;
	Controller_s *controller_motor;
	DJIMotor_Measure_s *measure; 
    // 遍历所有电机实例,并进行串级PID计算
 	for( size_t i = 0; i < idx ; i++ )
	{
		motor = motor_instance[i];
		if (motor->stop_flag == MOTOR_CLOSE) {
			sender_enable_flag[motor->sender_group] = 0; // 停止发送 仅有四个电机同时CLOSE时才生效 
//			sender_assignment[motor_instance[i]->sender_group].tx_buff[2 * motor_instance[i]->message_num]   = 0;
//			sender_assignment[motor_instance[i]->sender_group].tx_buff[2 * motor_instance[i]->message_num+1] = 0;
			continue;
		} else sender_enable_flag[motor->sender_group] = 1;
		
		controller_motor = &motor_instance[i]->motor_controller;
		measure = &motor_instance[i]->measure;
		group = motor_instance[i]->sender_group;
		num   = motor_instance[i]->message_num;
		// 若电机处于停止状态
       if (motor->stop_flag == MOTOR_ENALBED)
	   {
			if (controller_motor->motor_setting.motor_reverse_flag == MOTOR_REVERSE)
					controller_motor->motor_setting.close_loop_type >= ANGLE_LOOP ? (controller_motor->ref_position *= -1 )
						: (controller_motor->ref_speed *= -1 );
			// 角度环
			if (controller_motor->motor_setting.angle_feedback_source == MOTOR_FEED)
			    controller_motor->fdb_position = measure->Angle_DEG;
			else controller_motor->fdb_position = *motor->other_angle_feedback_ptr;
			//速度环
			if (controller_motor->motor_setting.speed_feedback_source == MOTOR_FEED)
				controller_motor->fdb_speed = measure->SpeedFilter;
			else controller_motor->fdb_speed = *motor->other_speed_feedback_ptr;
			// 电流环
			controller_motor->fdb_current = measure->CurrentFilter;
			
//			controller_clc(controller_motor);
			
			if (controller_motor->motor_setting.feedback_reverse_flag == FEEDBACK_REVERSE)
                 controller_motor->output *= -1;
//			// 获得输出
//			can_set = (int16_t)controller_motor->output;
//			// 分组填入
//			sender_assignment[group].tx_buff[2 * num]   = (uint8_t)(can_set >> 8);
//			sender_assignment[group].tx_buff[2 * num+1] = (uint8_t)(can_set & 0xff);
//		} else {
//			sender_assignment[group].tx_buff[2 * num]   = 0;
//			sender_assignment[group].tx_buff[2 * num+1] = 0;
		}
	}
//	for( size_t j = 0; j < 6 ; j++ )
//	{
//	   if(sender_enable_flag[j] ) {
//          CANTransmit(&sender_assignment[j], 1.0f);
//	   }
//    }
}


