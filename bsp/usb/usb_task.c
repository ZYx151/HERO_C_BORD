/**
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
  * @file       usb_task.c/h
  * @brief      通过USB串口与上位机通信
  * @note
  * @history
  *  Version    Date            Author          Modification
  *  V1.0.0     Jun-24-2024     Penguin         1. done

  @verbatim
  =================================================================================

  =================================================================================
  @endverbatim
  ****************************(C) COPYRIGHT 2024 Polarbear*************************
*/

#include "usb_task.h"

#include <stdbool.h>
#include <string.h>

#include "CRC.h"
#include "cmsis_os.h"
#include "ins_task.h"
#include "usb_debug.h"
#include "usb_device.h"
#include "usbd_cdc_if.h"
#include "usbd_conf.h"
#include "WatchDog.h"
#include "bsp_dwt.h"

#if INCLUDE_uxTaskGetStackHighWaterMark
uint32_t usb_high_water;
#endif

#define USB_TASK_CONTROL_TIME 2  // ms USB任务执行频率

#define USB_OFFLINE_THRESHOLD 100  // ms USB离线时间阈值
#define USB_CONNECT_CNT 10

// clang-format off
// 发送时间间隔，每次发送经过以下时间后发送消息
#define SEND_DURATION_Imu         10   // ms
#define SEND_DURATION_Debug       5   // ms
#define SEND_DURATION_RobotInfo   10  // ms
#define SEND_DURATION_Pid         10  // ms
#define SEND_DURATION_AllRobotHp  10  // ms
#define SEND_DURATION_GameStatus  10  // ms
#define SEND_DURATION_RobotMotion 10  // ms
// clang-format on

#define USB_RX_DATA_SIZE 256  // byte
#define USB_RECEIVE_LEN 150   // byte
#define HEADER_SIZE 4         // byte

// ##send_name 是一个预处理器操作符，用于将 send_name 的值与 字符 连接
// 检查是否已经过了一定的时间间隔， 超过时间间隔后，发送消息
#define CheckDurationAndSend(send_name)                                                  \
    do {                                                                                 \
        if ((HAL_GetTick() - LAST_SEND_TIME.##send_name) >= SEND_DURATION_##send_name) { \
            LAST_SEND_TIME.##send_name = HAL_GetTick();                                  \
            UsbSend##send_name##Data();                                                  \
        }                                                                                \
    } while (0)

// Variable Declarations
static uint8_t USB_RX_BUF[USB_RX_DATA_SIZE];

// 记录通信时间戳
static  float USB_DWT_CNT = 0, IMU_DWT_CNT = 0;          

static const Imu_t * IMU;
static const ChassisSpeedVector_t * FDB_SPEED_VECTOR;
// 视觉接收全局变量
extern Aim_Rx_t Aim_Rx;
WatchDog_TypeDef *UsbDog;
ReceiveVisionData_t ReceiveVisionData;
	
// 陀螺仪数据接收
static attitude_t *ins;
// 判断USB连接状态用到的一些变量
bool USB_OFFLINE = true;
static uint32_t RECEIVE_TIME = 0;
static uint32_t LATEST_RX_TIMESTAMP = 0;
static uint32_t CONTINUE_RECEIVE_CNT = 0;

// 数据发送结构体
// clang-format off
static SendDataImu_s         SEND_DATA_IMU;             // 发送IMU数据
// clang-format on

// 数据接收结构体
// static ReceiveDataVirtualRc_s RECEIVE_VIRTUAL_RC_DATA;  // 机器人虚拟遥控器结构体

// 机器人控制指令数据
//static RobotCmdData_t ROBOT_CMD_DATA;
// static RC_ctrl_t VIRTUAL_RC_CTRL;

// 发送数据间隔时间
typedef struct
{
    uint32_t Imu;
} LastSendTime_t;
static LastSendTime_t LAST_SEND_TIME;

uint8_t a = 10;
/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

static void UsbSendData(void);
static void UsbReceiveData(void);
static void UsbInit(void);

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/

static void UsbSendImuData(void);
static void UsbSendDebugData(void);
static void UsbSendRobotInfoData(void);
static void UsbSendAllRobotHpData(void);
static void UsbSendGameStatusData(void);
static void UsbSendRobotMotionData(void);
static void UsbOfflineCaback(void *id);   //!< @brief  虚拟串口离线回调函数

/*******************************************************************************/
/* Receine Function                                                               */
/*******************************************************************************/
void VisionDataReceive(uint8_t *data);
/******************************************************************/
/* Task                                                           */
/******************************************************************/



/**
 * @brief      USB任务主函数
 * @param[in]  argument: 任务参数
 * @retval     None
 */
void Task_Usb(void *pvParameters)
{
    vTaskDelay(10);  //等待USB设备初始化完成
    UsbInit();

    while (1) {
        UsbSendData();
        UsbReceiveData();

        if (HAL_GetTick() - RECEIVE_TIME > USB_OFFLINE_THRESHOLD) {
            USB_OFFLINE = true;
            CONTINUE_RECEIVE_CNT = 0;
        } else if (CONTINUE_RECEIVE_CNT > USB_CONNECT_CNT) {
            USB_OFFLINE = false;
        } else {
            CONTINUE_RECEIVE_CNT++;
        }
        
        vTaskDelay(USB_TASK_CONTROL_TIME);

#if INCLUDE_uxTaskGetStackHighWaterMark
        usb_high_water = uxTaskGetStackHighWaterMark(NULL);
#endif
    }
}

/*******************************************************************************/
/* Main Function                                                               */
/*******************************************************************************/

/**
 * @brief      USB初始化
 * @param      None
 * @retval     None
 */
static void UsbInit(void)
{
    // 订阅数据
    // IMU = Subscribe(IMU_NAME);                        // 获取IMU数据指针
    // FDB_SPEED_VECTOR = Subscribe(CHASSIS_FDB_SPEED_NAME);  // 获取底盘速度矢量指针

    // 数据置零
    memset(&LAST_SEND_TIME, 0, sizeof(LastSendTime_t));

    // 1.初始化IMU数据包
    SEND_DATA_IMU.frame_header.sof = SEND_SOF;
    SEND_DATA_IMU.frame_header.len = (uint8_t)(sizeof(SendDataImu_s) - 6);
    SEND_DATA_IMU.frame_header.id  = DATA_SEND_ID_Imu;
    append_CRC8_check_sum(  // 添加帧头 CRC8 校验位
        (uint8_t *)(&SEND_DATA_IMU.frame_header), sizeof(SEND_DATA_IMU.frame_header));
	
	// 获取陀螺仪数据
	ins = INS_Init();

	/* 初始化虚拟串口看门狗 */
	WatchDog_Init_config UsbConfig = {
		.watch_callback = UsbOfflineCaback,
		.dog_name = "USBDOG",
		.owner_id = 0,
		.Max_num = 10,
	};
	UsbDog = WatchDog_Init(UsbConfig);

}

/**
 * @brief      用USB发送数据
 * @param      None
 * @retval     None
 */
static void UsbSendData(void)
{
    // 发送Imu数据
//    CheckDurationAndSend(Imu);
	if ((HAL_GetTick() - LAST_SEND_TIME.Imu) >= SEND_DURATION_Imu) { 
		LAST_SEND_TIME.Imu = HAL_GetTick();                                  
		SEND_DATA_IMU.time_stamp = HAL_GetTick();
		UsbSendImuData();
	}                                                                                
}

/**
 * @brief      USB接收数据
 * @param      None
 * @retval     None
 */
static void UsbReceiveData(void)
{
    static uint32_t len = USB_RECEIVE_LEN;
    static uint8_t * rx_data_start_address = USB_RX_BUF;  // 接收数据包时存放于缓存区的起始位置
    static uint8_t * rx_data_end_address;  // 接收数据包时存放于缓存区的结束位置
    uint8_t * sof_address = USB_RX_BUF;

    // 计算数据包的结束位置
    rx_data_end_address = rx_data_start_address + USB_RECEIVE_LEN;
    // 读取数据
    USB_Receive(rx_data_start_address, &len);  // Read data into the buffer

    while (sof_address <= rx_data_end_address) {  // 解析缓冲区中的所有数据包
        // 寻找帧头位置
        while (*(sof_address) != RECEIVE_SOF && (sof_address <= rx_data_end_address)) {
            sof_address++;
        }
        // 判断是否超出接收数据范围
        if (sof_address > rx_data_end_address) {
            break;  // 退出循环
        }
        // 检查CRC8校验
        bool crc8_ok = verify_CRC8_check_sum(sof_address, HEADER_SIZE);
        if (crc8_ok) {
            uint8_t data_len = sof_address[1];
            uint8_t data_id = sof_address[2];
            // 检查整包CRC16校验 4: header size, 2: crc16 size
            bool crc16_ok = verify_CRC16_check_sum(sof_address, 4 + data_len + 2);
            if (crc16_ok) {
                switch (data_id) {
                    case ROBOT_CMD_DATA_RECEIVE_ID: {
						VisionDataReceive(sof_address);
                    } 
                    break;
                    
                    default:break;
                }
                if (*((uint32_t *)(&sof_address[4])) > LATEST_RX_TIMESTAMP) {
                    LATEST_RX_TIMESTAMP = *((uint32_t *)(&sof_address[4]));
                    RECEIVE_TIME = HAL_GetTick();
                }
            }
            sof_address += (data_len + HEADER_SIZE + 2);
        } else {  //CRC8校验失败，移动到下一个字节
            sof_address++;
        }
    }
    // 更新下一次接收数据的起始位置
    if (sof_address > rx_data_start_address + USB_RECEIVE_LEN) {
        // 缓冲区中没有剩余数据，下次接收数据的起始位置为缓冲区的起始位置
        rx_data_start_address = USB_RX_BUF;
    } else {
        uint16_t remaining_data_len = USB_RECEIVE_LEN - (sof_address - rx_data_start_address);
        // 缓冲区中有剩余数据，下次接收数据的起始位置为缓冲区中剩余数据的起始位置
        rx_data_start_address = USB_RX_BUF + remaining_data_len;
        // 将剩余数据移到缓冲区的起始位置
        memcpy(USB_RX_BUF, sof_address, remaining_data_len);
    }
}

/*******************************************************************************/
/* Send Function                                                               */
/*******************************************************************************/

/**
 * @brief 发送IMU数据
 * @param duration 发送周期
 */
static void UsbSendImuData(void)
{
    if (ins->ins_dog->state != Dog_Online) {
        return;
    }
	SEND_DATA_IMU.data.yaw = ins->Yaw * PI /180.0f;
	SEND_DATA_IMU.data.pitch = ins->Pitch * PI /180.0f;
	SEND_DATA_IMU.data.roll = ins->Roll * PI /180.0f;

	SEND_DATA_IMU.data.yaw_vel = ins->gyro[2];
	SEND_DATA_IMU.data.pitch_vel = ins->gyro[0];
	SEND_DATA_IMU.data.roll_vel = ins->gyro[1];
	
	append_CRC16_check_sum((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
	USB_Transmit((uint8_t *)&SEND_DATA_IMU, sizeof(SendDataImu_s));
}

/*******************************************************************************/
/* Receive Function                                                            */
/*******************************************************************************/
/* 自瞄数据接收 */			   
void VisionDataReceive(uint8_t *data)
{
    memcpy(&ReceiveVisionData, data, sizeof(ReceiveVisionData_t));
	// 接收状态更新
	Aim_Rx.Predicted_Gap = DWT_GetDeltaT((void *)&USB_DWT_CNT) * 1000.0f;//量测更新,在视觉空窗期电控去预测
	Aim_Rx.Rx_State = UPDATING;
    Feed_Dog(UsbDog);
}

// 虚拟串口数据丢失处理
void UsbOfflineCaback(void *id)
{
  Aim_Rx.Rx_State = 0;
}
