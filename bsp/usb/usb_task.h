#ifndef USB_TASK_H
#define USB_TASK_H

#include "stdbool.h"
//#include "Task_Init.h"
#include "usb_typdef.h"

#define CHASSIS_TYPE 2  // 选择底盘类型
#define GIMBAL_TYPE 1     // 选择云台类型
#define SHOOT_TYPE 1    // 选择发射机构类型
#define CONTROL_TYPE 2  // 选择控制类型
#define MECHANICAL_ARM_TYPE 0

//extern TaskHandle_t Task_Usb_Handle;

typedef struct __Imu
{
    float yaw, pitch, roll;              // rad
    float yaw_vel, pitch_vel, roll_vel;  // rad/s
    float x_accel, y_accel, z_accel;     // m/s^2
} Imu_t;

typedef struct  // 底盘速度向量结构体
{
    float vx;  // (m/s) x方向速度
    float vy;  // (m/s) y方向速度
    float wz;  // (rad/s) 旋转速度
} ChassisSpeedVector_t;

typedef struct
{
    ChassisSpeedVector_t speed_vector;
    struct
    {
        float roll;
        float pitch;
        float yaw;
        float leg_length;
    } chassis;

    struct
    {
        float pitch;
        float yaw;
    } gimbal;

    struct
    {
        bool fire;
        bool fric_on;
    } shoot;

} RobotCmdData_t;

typedef struct{
  FrameHeader_t frame_header;

  uint32_t time_stamp;

  struct 
  {
    uint8_t tracking;
    uint8_t id;
    uint8_t armors_num;
    float x;
    float y;
    float z;
    float yaw;
    float vx;
    float vy;
    float vz;
    float v_yaw;
    float r1;
    float r2;
    float dz;
  } __attribute__((packed)) data;

  uint16_t checksum;
} __packed__ ReceiveVisionData_t;
extern ReceiveVisionData_t ReceiveVisionData;

void usb_task(void *pvParameters);

#endif /* USB_TASK_H */
