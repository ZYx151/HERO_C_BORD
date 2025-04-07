/**
 ******************************************************************************
 * @file    ins_task.c
 * @author  Wang Hongxi
 * @author  annotation and modificaiton by neozng
 * @version V2.0.0
 * @date    2022/2/23
 * @brief
 ******************************************************************************
 * @attention
 *
 ******************************************************************************
 */
#include "ins_task.h"
#include "controller.h"
#include "QuaternionEKF.h"
#include "spi.h"
#include "tim.h"
#include "bsp_def.h"
#include "user_lib.h"
#include "bsp_gpio.h"


#define X 0
#define Y 1
#define Z 2

static INS_t INS;
static IMU_Param_t IMU_Param;
static PID_t TempCtrl = {0};

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

// 用于获取两次采样之间的时间间隔
static uint32_t INS_DWT_Count = 0;
static float dt = 0, t = 0;
static float RefTemp = 47; // 恒温设定温度

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);
static uint8_t INS_IfDataError();

static void IMUPWMSet(uint16_t pwm)
{
    __HAL_TIM_SetCompare(&htim10, TIM_CHANNEL_1, pwm);
}

static void imu_heat_init()
{
    HAL_TIM_PWM_Start(&htim10, TIM_CHANNEL_1);
    PID_Init_Config_s config = {.MaxOut = 3000,
                                .IntegralLimit = 300,
                                .DeadBand = 0,
                                .Kp = 1000,
                                .Ki = 1000,
                                .Kd = 0,
                                .Improve = 0x01}; // enable integratiaon limit 
    PID_Create(&config, &TempCtrl);
}
/**
 * @brief 温度控制
**/
static void IMU_Temperature_Ctrl(void)
{
    PID_Calculate(&TempCtrl, BMI088.Temperature, RefTemp, 0);
    IMUPWMSet(float_constrain(float_rounding(TempCtrl.Output), 0, 10000));
}

// 使用加速度计的数据初始化Roll和Pitch,而Yaw置0,这样可以避免在初始时候的姿态估计误差
static void InitQuaternion(float *init_q4)
{
    float acc_init[3] = {0};
    float gravity_norm[3] = {0, 0, 1}; // 导航系重力加速度矢量,归一化后为(0,0,1)
    float axis_rot[3] = {0};           // 旋转轴
    // 读取100次加速度计数据,取平均值作为初始值
    for (uint8_t i = 0; i < 100; ++i)
    {
        BMI088_Read(&BMI088);
        acc_init[X] += BMI088.Accel[X];
        acc_init[Y] += BMI088.Accel[Y];
        acc_init[Z] += BMI088.Accel[Z];
        DWT_Delay(0.001);
    }
    for (uint8_t i = 0; i < 3; ++i)
        acc_init[i] /= 100;
    Norm3d(acc_init);
    // 计算原始加速度矢量和导航系重力加速度矢量的夹角
    float angle = acosf(Dot3d(acc_init, gravity_norm));
    Cross3d(acc_init, gravity_norm, axis_rot);
    Norm3d(axis_rot);
    init_q4[0] = cosf(angle / 2.0f);
    for (uint8_t i = 0; i < 2; ++i)
        init_q4[i + 1] = axis_rot[i] * sinf(angle / 2.0f); // 轴角公式,第三轴为0(没有z轴分量)
}

attitude_t *INS_Init(void)
{
    if (!INS.init)
        INS.init = 1;
    else
        return (attitude_t *)&INS.attitude;
    
//	BMI088_config ins_config = {
//		.bsp_gpio_accel_index = GPIO_BMI088_ACCEL_NS,
//		.bsp_gpio_gyro_index = GPIO_BMI088_GYRO_NS,
//		.imu_axis_convert[0] = 1,
//		.imu_axis_convert[1] = 2,
//		.imu_axis_convert[2] = 3,
//	};
    while (BMI088Init(&hspi1, 1) != BMI088_NO_ERROR)
        ;
	
    IMU_Param.scale[X] = 1;
    IMU_Param.scale[Y] = 1;
    IMU_Param.scale[Z] = 1;
    IMU_Param.Yaw = 0;
    IMU_Param.Pitch = 0;
    IMU_Param.Roll = 0;
    IMU_Param.flag = 1;
    
    float init_quaternion[4] = {0};
    InitQuaternion(init_quaternion);
    IMU_QuaternionEKF_Init(init_quaternion, 10, 0.001, 1000000, 1, 0);
    
    imu_heat_init();
    
	WatchDog_Init_config INS_Dog = {
//		.feed_callback = INS_IfDataError,
		.dog_name = "INSDog",
		.owner_id = &INS,
		.Max_num = 20,
    };
    INS.attitude.ins_dog = WatchDog_Init(INS_Dog);
    // noise of accel is relatively big and of high freq,thus lpf is used
    INS.AccelLPF = 0.0085;
    DWT_GetDeltaT(&INS_DWT_Count);
	
    return (attitude_t *)&INS;
}

/* 注意以1kHz的频率运行此任务 */
void INS_Task(void)
{
    static uint32_t count = 0, led_count;
    const float gravity[3] = {0, 0, 9.81f};

    dt = DWT_GetDeltaT(&INS_DWT_Count);
    t += dt;
    BSP_GPIO_Toggle(3);
    BSP_GPIO_Toggle(4);
    BSP_GPIO_Toggle(5);
    // ins update 
    if ((count % 1) == 0)
    {
        BMI088_Read(&BMI088);
         
        INS.attitude.accel[0] = BMI088.Accel[0];  // X
        INS.attitude.accel[1] = BMI088.Accel[1];  // Y
        INS.attitude.accel[2] = BMI088.Accel[2];  // Z
		
        INS.attitude.gyro[0] = BMI088.Gyro[0];    // X
        INS.attitude.gyro[1] = BMI088.Gyro[1];    // Y
        INS.attitude.gyro[2] = BMI088.Gyro[2];    // Z
        
        // demo function,用于修正安装误差, 位移矩阵 如果安装位置再转轴中心则可以不用
//        IMU_Param_Correction(&IMU_Param, INS.attitude.gyro, INS.attitude.accel);
        
        // 计算重力加速度矢量和b系的XY两轴的夹角,可用作功能扩展
//         INS.atanxz = -atan2f(INS.attitude.accel[X], INS.attitude.accel[Z]) * 180 / PI;
//         INS.atanyz = atan2f(INS.attitude.accel[Y], INS.attitude.accel[Z]) * 180 / PI;
        
        // 核心函数,EKF更新四元数
        IMU_QuaternionEKF_Update(INS.attitude.gyro[X], INS.attitude.gyro[Y], INS.attitude.gyro[Z], INS.attitude.accel[X], INS.attitude.accel[Y], INS.attitude.accel[Z], dt);
        
        memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q)); 
        memcpy(INS.attitude.q, QEKF_INS.q, sizeof(QEKF_INS.q));
		
        // 机体系基向量转换到导航坐标系，本例选取惯性系为导航系
        BodyFrameToEarthFrame(xb, INS.xn, INS.q);
        BodyFrameToEarthFrame(yb, INS.yn, INS.q);
        BodyFrameToEarthFrame(zb, INS.zn, INS.q);
        
        // 将重力从导航坐标系n转换到机体系b,随后根据加速度计数据计算运动加速度
        float gravity_b[3];
        EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
        for (uint8_t i = 0; i < 3; ++i) // 同样过一个低通滤波
        {
            INS.MotionAccel_b[i] = (INS.attitude.accel[i] - gravity_b[i] ) * dt / (INS.AccelLPF + dt) + INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + dt);
        }
        BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q); // 转换回导航系n
        
        INS.attitude.Yaw   = QEKF_INS.Yaw;
        INS.attitude.Pitch = QEKF_INS.Pitch;
        INS.attitude.Roll  = QEKF_INS.Roll;
        INS.attitude.ContinuousYaw = QEKF_INS.YawTotalAngle;
		INS.attitude.YawRoundCount = QEKF_INS.YawRoundCount;
//		INS.ins_error = BMI088.error;
		if( INS_IfDataError() )
		    Feed_Dog(INS.attitude.ins_dog);
        
		//VisionSetAltitude(INS.Yaw, INS.Pitch, INS.Roll);  // 打印陀螺仪数据
    }
    count ++;
    // temperature control
    if ((count % 2) == 0)
    {
        // 500hz
        IMU_Temperature_Ctrl();
    }
}

/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
* @brief           四元数旋转矩阵从大地系R转换到机体坐标系b  Transform 3 dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float32_t cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
	
    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
//        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
//        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
//        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
//        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
//        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
//        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        cosYaw = cosf(param->Yaw / 57.295779513f);
        cosPitch = cosf(param->Pitch / 57.295779513f);
        cosRoll = cosf(param->Roll / 57.295779513f);
        sinYaw = sinf(param->Yaw / 57.295779513f);
        sinPitch = sinf(param->Pitch / 57.295779513f);
        sinRoll = sinf(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[X] = c_11 * gyro_temp[X] +
              c_12 * gyro_temp[Y] +
              c_13 * gyro_temp[Z];
    gyro[Y] = c_21 * gyro_temp[X] +
              c_22 * gyro_temp[Y] +
              c_23 * gyro_temp[Z];
    gyro[Z] = c_31 * gyro_temp[X] +
              c_32 * gyro_temp[Y] +
              c_33 * gyro_temp[Z];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];

    accel[X] = c_11 * accel_temp[X] +
               c_12 * accel_temp[Y] +
               c_13 * accel_temp[Z];
    accel[Y] = c_21 * accel_temp[X] +
               c_22 * accel_temp[Y] +
               c_23 * accel_temp[Z];
    accel[Z] = c_31 * accel_temp[X] +
               c_32 * accel_temp[Y] +
               c_33 * accel_temp[Z];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

//------------------------------------functions below are not used in this demo-------------------------------------------------
//----------------------------------you can read them for learning or programming-----------------------------------------------
//----------------------------------they could also be helpful for further design-----------------------------------------------

/**
 * @brief        Update quaternion
 */
void QuaternionUpdate(float *q, float gx, float gy, float gz, float dt)
{
    float qa, qb, qc;

    gx *= 0.5f * dt;
    gy *= 0.5f * dt;
    gz *= 0.5f * dt;
    qa = q[0];
    qb = q[1];
    qc = q[2];
    q[0] += (-qb * gx - qc * gy - q[3] * gz);
    q[1] += (qa * gx + qc * gz - q[3] * gy);
    q[2] += (qa * gy - qb * gz + q[3] * gx);
    q[3] += (qa * gz + qb * gy - qc * gx);
}

/**
 * @brief        Convert quaternion to eular angle
 */
void QuaternionToEularAngle(float *q, float *Yaw, float *Pitch, float *Roll)
{
    *Yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f) * 57.295779513f;
    *Pitch = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f) * 57.295779513f;
    *Roll = asinf(2.0f * (q[0] * q[2] - q[1] * q[3])) * 57.295779513f;
}

/**
 * @brief        Convert eular angle to quaternion
 */
void EularAngleToQuaternion(float Yaw, float Pitch, float Roll, float *q)
{
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;
    Yaw /= 57.295779513f;
    Pitch /= 57.295779513f;
    Roll /= 57.295779513f;
    cosPitch = arm_cos_f32(Pitch / 2);
    cosYaw = arm_cos_f32(Yaw / 2);
    cosRoll = arm_cos_f32(Roll / 2);
    sinPitch = arm_sin_f32(Pitch / 2);
    sinYaw = arm_sin_f32(Yaw / 2);
    sinRoll = arm_sin_f32(Roll / 2);
    q[0] = cosPitch * cosRoll * cosYaw + sinPitch * sinRoll * sinYaw;
    q[1] = sinPitch * cosRoll * cosYaw - cosPitch * sinRoll * sinYaw;
    q[2] = sinPitch * cosRoll * sinYaw + cosPitch * sinRoll * cosYaw;
    q[3] = cosPitch * cosRoll * sinYaw - sinPitch * sinRoll * cosYaw;
}

uint8_t INS_IfDataError(){
    if(fabs(INS .attitude .Pitch)>180||fabs (INS .attitude .Roll)>180||fabs (INS .attitude .Yaw )>180
        ||(INS .attitude  .Pitch ==0&&INS .attitude  .Roll ==0&&INS .attitude  .Yaw ==0))
        return 0;
    else
        return 1;
}

 float GetINS_Gyro(uint8_t gyro) {
  return INS.attitude.gyro[gyro];
 }
