#include "Chassis_board_cmd.h"

#include "robot_def.h"
#include "pub_sub.h"
#include "bsp_dwt.h"
#include "SuperCAP.h"
//#include "Referee_unpack.h"
#include "can_comm.h"
#include "CANDrive.h"
#include "ins_task.h"

#include "Referee.h"

static void stop_mode_update(void);         //机器人停止模式更新函数
void Chassis_Speedget(CANInstance *_instance);
static void soft_rest(void);

/**
 * @brief 底盘应用,负责接收robot_cmd的控制命令并根据命令进行运动学解算,得到输出
 *        注意底盘采取右手系,对于平面视图,底盘纵向运动的正前方为x正方向;横向运动的右侧为y正方向 
**/                                        
CANCommInstance *Action_can_comm;  // 双板通信CAN comm

CANCommInstance *Vision_can_comm;  // 双板通信CAN comm

void Action_get(CANInstance *_instance);

void Vision_get(CANInstance *_instance);

Gimbal_data_t receive_vision;
Gimbal_action_t  receive_action; 

static CANCommInstance *chasiss_can_comm;  // 双板通信CAN comm

static	Robot_Status_e chassis_state;      // 机器人整体工作状态
static	Robot_Status_e last_chassis_state; // 机器人整体工作状态

static Publisher_t  *chassis_cmd_pub;    // 底盘反馈消息发布者
static Subscriber_t *chassis_feed_sub;   // 底盘控制信息订阅者

static Chassis_ctrl_cmd_t chassis_cmd_ctrl;      // 底盘发送CMD的控制信息
static Chassis_upload_t  chassis_feedback_data;  // 底盘接收的反馈信息

static Publisher_t  *chassis_feed_pub;   // 底盘反馈消息发布者
static Subscriber_t *chassis_cmd_sub;    // 底盘控制信息订阅者

static	Gimbal_board_send_t  receive_data;   // 上下板通信接收
static	Gimbal_board_send_UI_t  receive_UI;  // 上下板通信接收
static	Chassis_board_send_t send_data;      // 上下板通信发送

/** 裁判系统  **/
static Subscriber_t  *referee_feedback_sub;   // 裁判系统反馈消息发布者
extern Referee_data_t Referee_SendData;              // 接收到裁判系统任务反馈

/**  功率重分配 RLS任务计算  **/
static RLS_update_t chassis_power_send;    // 发送功率数据结构体
static Publisher_t *rls_power_pub;         // 裁判系统反馈消息发布者

void Chassis_Cmd_Init()
{
	/* 上下板通信 */
	CANComm_Init_Config_s comm_conf = {
		.can_config = {
			.can_handle = &hcan1,
			.tx_id = 0x101,
			.rx_id = 0x110,
			.can_module_callback = Chassis_Speedget,
		},
		.recv_data_len = sizeof( Gimbal_board_send_t),
		.send_data_len = sizeof( Chassis_board_send_t),
		.dog_config = {
			.Max_num = 5,
			.dog_name = "LOBOARD",
//			.watch_callback = Chassis_Speedget,
		},
	};
	chasiss_can_comm = CANCommInit(comm_conf);
    
    chassis_feed_sub = SubRegister("chassis_feed", sizeof(Chassis_upload_t));
    chassis_cmd_pub  = PubRegister("chassis_cmd", sizeof(Chassis_ctrl_cmd_t));
	/* 裁判系统初始化 */
    referee_feedback_sub = SubRegister("referee", sizeof(Referee_data_t));
	rls_power_pub        = PubRegister("rlssend", sizeof(RLS_update_t));	
	CANComm_Init_Config_s action_conf = {
		.can_config = {
			.can_handle = &hcan1,
//			.tx_id = 0x101,
			.rx_id = 0x120,
			.can_module_callback = Action_get,
		},
		.recv_data_len = sizeof(Gimbal_action_t),
//		.send_data_len = sizeof( Chassis_board_send_t),
		.dog_config = {
			.Max_num = 10,
			.dog_name = "ACTDOG",
//			.watch_callback = Chassis_Speedget,
		},
	};
    Action_can_comm = CANCommInit(action_conf);
    
    CANComm_Init_Config_s vision_conf = {
		.can_config = {
			.can_handle = &hcan1,
//			.tx_id = 0x101,
			.rx_id = 0x130,
			.can_module_callback = Vision_get,
		},
		.recv_data_len = sizeof(Gimbal_data_t),
//		.send_data_len = sizeof( Chassis_board_send_t),
		.dog_config = {
			.Max_num = 10,
			.dog_name = "VISDOG",
//			.watch_callback = Chassis_Speedget,
		},
	};
    Vision_can_comm = CANCommInit(vision_conf);
}

void Chassis_board_CMD_Update()
{
	// 板间通信 -收
    if (chasiss_can_comm->Dog->state != Dog_Online) {
		chassis_state = ROBOT_STOP;
        receive_UI.chassis_mode = CHASSIS_ZERO_FORCE;
		chassis_cmd_ctrl.Close_flag = 1;
		chassis_cmd_ctrl.Shift_flag= 0;
		receive_data.vx = 0;
		receive_data.vy = 0;
		receive_data.rotate = 0;
	 } else {
		chassis_state = Robot_RUNNING;
		chassis_cmd_ctrl.mode = receive_UI.chassis_mode;
		chassis_cmd_ctrl.vy = receive_data.vy;
		chassis_cmd_ctrl.vx = receive_data.vx;
		chassis_cmd_ctrl.rotate = receive_data.rotate;
		chassis_cmd_ctrl.Close_flag = receive_data.Close_flag;
		chassis_cmd_ctrl.Shift_flag = receive_data.Shift_flag;
		chassis_cmd_ctrl.dispatch_mode  = receive_UI.chassis_dispatch_mode;
    }
	 
//    if(chassis_cmd_ctrl.soft_reset_flag > 0)
//		chassis_state = ROBOT_STOP;
//	  if (chassis_cmd_ctrl.soft_reset_flag == 1) {
//		   static int16_t robot_stop_cnt = 0;
//           robot_stop_cnt ++;
//		  // 等待20ms使得电机全部下线
//		   if(robot_stop_cnt >= 20) {
//               chassis_cmd_ctrl.soft_reset_flag = 2;
//		   }
//	  } else if (chassis_cmd_ctrl.soft_reset_flag == 2) {
//        //等待10ms使得can通信完成
//        static int16_t can_wait_cnt = 0;
//        can_wait_cnt++;
//        if (can_wait_cnt == 10) 
//             soft_rest();
//	  }
	  
	 //   接收底盘模块模块  todo : 与超电通信 以及消息内容的判断
	SubGetMessage(chassis_feed_sub, (void *)&chassis_feedback_data);
    if (chassis_feedback_data.chassis_status != Device_Online ) {
		chassis_state = ROBOT_STOP;
        chassis_cmd_ctrl.mode = CHASSIS_ZERO_FORCE;
	} else {
	 chassis_cmd_ctrl.mode = CHASSIS_NORMAL;
	 chassis_cmd_ctrl.dispatch_mode = chassis_dispatch_fly;
	}
	 
	 /* 裁判系统的在线判断 UI的初始化 动态UI 上下板通信 */
	 Referee_Update(); // 裁判系统数据处理
     SubGetMessage(referee_feedback_sub, (void *)&Referee_SendData);
     if (Referee_SendData.refree_status != Device_Online) {
		 chassis_power_send.refree_status = Device_Offline;
//		     chassis_state = ROBOT_STOP;
//          chassis_cmd_ctrl.mode = CHASSIS_ZERO_FORCE;
	 } else {
		  chassis_power_send.send_power.power_buffer = Referee_SendData.buffer_energy;                // 当前缓冲能量
		  chassis_power_send.send_power.power_now    = Referee_SendData.chassis_power;                // 当前底盘功率
		  chassis_power_send.send_power.power_limit  = Referee_SendData.chassis_power_limit;          // 底盘功率限制
		  chassis_power_send.refree_status = Referee_SendData.refree_status;
		  
		  send_data.shoot_referee_data.heat_limit_remain = Referee_SendData.shooter_barrel_heat_limit - Referee_SendData.heat_now_remain; // 剩余热量
	      send_data.chassis_slop = chassis_feedback_data.chassis_slop;
	 }
	 
	 /* 向功率控制发送数据 */ 
	 chassis_power_send.shitf_flag = chassis_cmd_ctrl.Shift_flag;
	 chassis_power_send.robot_level = Referee_SendData.robot_level;
	 PubPushMessage(rls_power_pub, (void *)&chassis_power_send);
	 chassis_cmd_ctrl.robot_levels = Referee_SendData.robot_level;
	 switch(Referee_SendData.robot_level)
     {
         case 1:   Referee_SendData.level_gain = 1.05;     break;
         case 2:   Referee_SendData.level_gain = 1.15f;     break;
         case 3:   Referee_SendData.level_gain = 1.20f;    break;
         case 4:   Referee_SendData.level_gain = 1.25f;  break;
         case 5:   Referee_SendData.level_gain = 1.35f;   break;
         case 6:   Referee_SendData.level_gain = 1.40f;    break;
         case 7:   Referee_SendData.level_gain = 1.45f;     break;
         case 8:   Referee_SendData.level_gain = 1.50f;    break;
         case 9:   Referee_SendData.level_gain = 1.60f;  break;
         case 10:  Referee_SendData.level_gain = 1.70f;    break;
         default:  Referee_SendData.level_gain = 1;     break;
	 }
	 
	// 所有系统都在线 开始执行任务模块
	 if (chassis_state == ROBOT_STOP ) {
		 stop_mode_update();
	 } else {
		 send_data.chassis_gyro = (int8_t )( GetINS_Gyro(2) * 200);
	 }
	 //  消息发布
    PubPushMessage(chassis_cmd_pub, (void *)&chassis_cmd_ctrl);
//     Chassis_CANComm_State = CANCommSend(chasiss_can_comm, (void *)&send_data);
    CAN_Send_StdDataFrame(&hcan1, 0x101, (uint8_t *)&send_data);
}

static void stop_mode_update()
{
	// 关闭自身模块
	memset(&chassis_cmd_ctrl, 0, sizeof(chassis_cmd_ctrl));
	chassis_cmd_ctrl.Close_flag = 1;
	// 底盘模块回传消息
	send_data.chassis_gyro = 0;
}

void Chassis_Speedget(CANInstance *_instance)
{
	CANCommInstance *comm = (CANCommInstance *)_instance->id;
	memcpy(&receive_data, _instance->rx_buff, sizeof(Gimbal_board_send_t));
	comm->recv_state = 0;
	comm->cur_recv_len = 0;
	comm->update_flag = 1;
	Feed_Dog(comm->Dog);
}

void Action_get(CANInstance *_instance)
{
    CANCommInstance *comm = (CANCommInstance *)_instance->id;
	memcpy(&receive_action, _instance->rx_buff, sizeof(Gimbal_action_t));
	comm->recv_state = 0;
	comm->cur_recv_len = 0;
	comm->update_flag = 1;
	Feed_Dog(comm->Dog);
}

void Vision_get(CANInstance *_instance)
{
    CANCommInstance *comm = (CANCommInstance *)_instance->id;
    memcpy(&receive_vision, _instance->rx_buff, sizeof(Gimbal_data_t));
    
	comm->recv_state = 0;
	comm->cur_recv_len = 0;
	comm->update_flag = 1;
	Feed_Dog(comm->Dog);
}

/**  软件软重启  **/ 
void soft_rest()
{
    // 关闭所有中断
    __set_FAULTMASK(1); 
    // 复位
    NVIC_SystemReset();
}