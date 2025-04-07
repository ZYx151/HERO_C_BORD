/*!
 * @file     IMU.c
 * @brief    陀螺仪模块
 */

#include "IMU.h"
#include "string.h"
#include "time.h"
#include "math.h"

/* IMU数据结构体定义 */
IMU_Typedef IMU;
USARTInstance *imu_uart;

void IMU_Receive(uint8_t *Data)
{
	if (Data[0] == 0x55)
	{
		for (uint16_t i = 1; i < IMU_LEN; i += 11)
		{
			switch (Data[i])
			{
#if Time_EN == 1 // 时间(片上时间，不是当前时区时间，实际用处不大,需校准)
			case kItemTime:
				IMU.Time.YY =  Data[i + 1];
				IMU.Time.MM =  Data[i + 2];
				IMU.Time.DD =  Data[i + 3];
				IMU.Time.HH =  Data[i + 4];
				IMU.Time.MN =  Data[i + 5];
				IMU.Time.SS =  Data[i + 6];
				IMU.Time.MS =  (int16_t)(Data[i + 8] << 8) | Data[i + 7];
//                IMU.Time.TimeStamp = standard_to_stamp(&IMU);
				break;
#endif

#if Quaternions_EN == 1   // 四元数
			case kItemQuaternions:
				IMU.Quaternions.W = ((int16_t)(Data[i + 2] << 8) | Data[i + 1]) / 32768.0f;
				IMU.Quaternions.X = ((int16_t)(Data[i + 4] << 8) | Data[i + 3]) / 32768.0f;
				IMU.Quaternions.Y = ((int16_t)(Data[i + 6] << 8) | Data[i + 5]) / 32768.0f;
				IMU.Quaternions.Z = ((int16_t)(Data[i + 8] << 8) | Data[i + 7]) / 32768.0f;
//                IMU.RotationMatrix = QuaternionToRotationMatrix(&IMU);
				break;
#endif

#if Acceleration_EN == 1   // 加速度
			case kItemAcceleration:
				IMU.Acceleration.X = ((int16_t)(Data[i + 2] << 8) | Data[i + 1]) / 32768.0f * 16 * 9.8f;
				IMU.Acceleration.Y = ((int16_t)(Data[i + 4] << 8) | Data[i + 3]) / 32768.0f * 16 * 9.8f;
				IMU.Acceleration.Z = ((int16_t)(Data[i + 6] << 8) | Data[i + 5]) / 32768.0f * 16 * 9.8f;
				break;
#endif

#if AngularVelocity_EN == 1 // 角速度
			case kItemAngularVelocity:
				IMU.AngularVelocity.X = ((int16_t)(Data[i + 2] << 8) | Data[i + 1]) / 32768.0f * 2000;
				IMU.AngularVelocity.Y = ((int16_t)(Data[i + 4] << 8) | Data[i + 3]) / 32768.0f * 2000;
				IMU.AngularVelocity.Z = ((int16_t)(Data[i + 6] << 8) | Data[i + 5]) / 32768.0f * 2000;
				break;
#endif

#if EulerAngle_EN == 1     // 欧拉角
			case kItemEulerAngler:
				IMU.EulerAngler.Roll  = ((int16_t)(Data[i + 2] << 8) | Data[i + 1]) / 32768.0f * 180;
				IMU.EulerAngler.Pitch = ((int16_t)(Data[i + 4] << 8) | Data[i + 3]) / 32768.0f * 180;
				IMU.EulerAngler.Yaw   = ((int16_t)(Data[i + 6] << 8) | Data[i + 5]) / 32768.0f * 180;

				float diff = IMU.EulerAngler.Yaw - IMU.EulerAngler.LsatAngle;
				if (diff > 100)
					IMU.EulerAngler.r--;
				else if (diff < -100)
					IMU.EulerAngler.r++;

				IMU.EulerAngler.ContinuousYaw = IMU.EulerAngler.r * 360.0f + IMU.EulerAngler.Yaw;
				IMU.EulerAngler.LsatAngle = IMU.EulerAngler.Yaw;
				break;
#endif
			}
		}
	}
   Feed_Dog(IMU.imu_dog);
}

#if Time_EN
/* 标准时间转换为时间戳 */
int64_t standard_to_stamp(IMU_Typedef *Dst)
{
	struct tm stm;
	stm.tm_year=Dst->Time.YY + 100;
	stm.tm_mon=Dst->Time.MM - 1;
	stm.tm_mday=Dst->Time.DD;
	stm.tm_hour=Dst->Time.HH - 8 ;
	stm.tm_min=Dst->Time.MN;
	stm.tm_sec=Dst->Time.SS;
//  	return (int64_t)mktime(&stm);   //秒时间戳
	return (int64_t)mktime(&stm) * 1000 + Dst->Time.MS; //毫秒时间戳
}
#endif

#if Quaternions_EN
/* 旋转矩阵 */
RotationMatrix_t  QuaternionToRotationMatrix(IMU_Typedef *Dst) {
    RotationMatrix_t R;
    float w = Dst->Quaternions.W, x = Dst->Quaternions.X, y = Dst->Quaternions.Y, z = Dst->Quaternions.Z;
    
    R.r[0][0] = 1 - 2*y*y - 2*z*z;
    R.r[0][1] = 2*x*y - 2*w*z;
    R.r[0][2] = 2*x*z + 2*w*y;
    R.r[1][0] = 2*x*y + 2*w*z;
    R.r[1][1] = 1 - 2*x*x - 2*z*z;
    R.r[1][2] = 2*y*z - 2*w*x;
    R.r[2][0] = 2*x*z - 2*w*y;
    R.r[2][1] = 2*y*z + 2*w*x;
    R.r[2][2] = 1 - 2*x*x - 2*y*y;
    
    return R;
}
#endif

uint8_t IMU_IfDataError(void * id){
    if(fabs(IMU .EulerAngler .Pitch)>180||fabs (IMU .EulerAngler .Roll)>180||fabs (IMU .EulerAngler .Yaw )>180
        ||(IMU .EulerAngler .Pitch ==0&&IMU .EulerAngler .Roll ==0&&IMU .EulerAngler .Yaw ==0))
        return 0;
    else
        return 1;
}

void IMU_Init()
{
	WatchDog_Init_config IMU_Dog = {
	.feed_callback = IMU_IfDataError,
	.dog_name = "IMUDog",
	.owner_id = &IMU,
	.Max_num = 20,
    };
    IMU.imu_dog = WatchDog_Init(IMU_Dog);
	
	USART_Init_Config_s imu_config = {
		.data_len = (2+ IMU_LEN),
		.usart_handle = &huart3,
		.module_callback = IMU_Receive,
	};
	imu_uart = USARTRegister(&imu_config );
}

/**
* @brief  获取遥控器指针
*/
IMU_Typedef *get_imu_control_point(void)
{
    return &IMU;
}

