#ifndef USB_TYPEDEF_H
#define USB_TYPEDEF_H

#include "attribute_typedef.h"
#include "struct_typedef.h"
#include "stdbool.h"
#include "robot_def.h"
#define DEBUG_PACKAGE_NUM 10

#define DATA_DOMAIN_OFFSET 0x08

#define SEND_SOF    ((uint8_t)0x5A)
#define RECEIVE_SOF ((uint8_t)0x5A)

#define DATA_SEND_ID_Imu           ((uint8_t)0x01)  // ms
#define DATA_SEND_ID_AllRobotHp    ((uint8_t)0x02)  // ms
#define DATA_SEND_ID_GameStatus    ((uint8_t)0x03)  // ms
#define DATA_SEND_ID_MyLocation    ((uint8_t)0x04)  // ms
#define DATA_SEND_ID_AllLocation   ((uint8_t)0x05)  // ms
#define DATA_SEND_ID_IsRFID        ((uint8_t)0x06)  // ms
#define DATA_SEND_ID_AutoAimMsg    ((uint8_t)0x07)  // ms
#define DATA_SEND_ID_AllowCapasity ((uint8_t)0x08)  // ms

#define ROBOT_CMD_DATA_RECEIVE_ID  ((uint8_t)0x03)
// clang-format on

typedef struct
{
    uint8_t sof;  // 数据帧起始字节，固定值为 0x5A
    uint8_t len;  // 数据段长度
    uint8_t id;   // 数据段id
    uint8_t crc;  // 数据帧头的 CRC8 校验
} __packed__ FrameHeader_t;
/*-------------------- Send --------------------*/
// IMU 数据包
typedef struct
{
    FrameHeader_t frame_header;  // 数据段id = 0x02
    uint32_t time_stamp;
    struct
    {
        float yaw;    // rad
        float pitch;  // rad
        float roll;   // rad

        float yaw_vel;    // rad/s
        float pitch_vel;  // rad/s
        float roll_vel;   // rad/s

    } __packed__ data;
    uint16_t crc;
} __packed__ SendDataImu_s;

/*-------------------- Receive --------------------*/

/* 目标位姿 */
typedef struct{
        float x;
        float y;
        float z;
        float HorizontalDistance;
        float theta;
}__packed__ Pose_t;

/* 接收视觉数据的结构体（内存对齐） */
typedef struct{
    enum {
        NOTHING = 0,  // 串口离线
        UPDATE = 1,   // 未接受到数据(通信空档期)
        UPDATING = 2, // 接收到数据 
    }Rx_State;                        //!< @brief 接收状态

    Pose_t Predicted_Center_Pose;     //!< @brief 惯性系下的目标中心点位姿
    float Predicted_Center_time;      //!< @brief 预测时间
    uint16_t  Fixed_Center_time;      //!< @brief 静态预测时间(拨弹盘转动，通信延迟等)
    float Predicted_Center_Yaw;       //!< @brief 预测中心的Y轴角度
    float Predicted_Center_Pitch;     //!< @brief 预测中心的P轴角度
    
    Pose_t Predicted_Armor_Pose[4];   //!< @brief 目标装甲板位姿（最多四块）
    float Predicted_Armor_time;
    uint16_t  Fixed_Armor_time;
    float Predicted_Armor_Yaw[4];
    float Predicted_Armor_Pitch; 
    
    float P_thre;                      //!< @brief 自动打弹角度阈值
    float Y_thre; 
    float K;                           //!< @brief 空气阻力系数
    float PitchOffset;                 //!< @brief 弹道补偿
	
	float Predicted_Gap;               //!< @brief 补偿单片机时间戳
}__packed__ Aim_Rx_t;

#endif  // USB_TYPEDEF_H
