#include "Task_Vision.h"

#include "WatchDog.h"
#include "Referee_unpack.h"
#include "ins_task.h"
#include "usb_task.h"
#include "arm_math.h"

/**  打弹阈值  **/
#define AM02_ARMOR_X  14
#define AM02_ARMOR_Y  12.5f
#define AM12_ARMOR_X  23.5f
#define AM12_ARMOR_Y  12.7f
// 打弹延迟
#define BULLET_FIRE_TIME 300
#define X_BIAS  0.0         //枪口前推的距离
#define Z_BIAS  0.15         //yaw轴电机到枪口水平面的垂直距离

// 全局变量
extern Aim_Action_e Aim_Action;       // 自瞄开关
extern Chassis_board_send_t  receive_data; // 接收裁判系统数据
extern WatchDog_TypeDef *UsbDog;
Aim_Data_t Aim_Ref;
Aim_Rx_t Aim_Rx;                //!< @brief  自瞄消息处理
// 静态变量
static attitude_t *ins;
/* 自瞄补偿 */
float yaw_v;
float temp_pitch;
float armor_x,armor_y,armor_z;

/**  功能函数  **/
void Aim_Init(void);        	      //!< @brief  PC通信初始化
void Aim_Control(void);            //!< @brief  自瞄主控函数

/**  坐标转换(N->B)  **/
//  void Coordinate_Transformation (RotationMatrix_t R, const Pose_t* PoseN, Pose_t* PoseB); 
/**  坐标点到原点的水平距离  **/
static float DistanceHorizontal(Pose_t pose);
/**  坐标点到原点的距离  **/
static float DistanceToOrigin(Pose_t pose);
/**  空气阻力模型  **/
static float monoAirResistance_Model(float horizontal, float bullet_speed, float angle_pitch, float k);
/**  获得飞行时间  **/
static float Get_bullet_fly_time( float horizontal, float bullet_speed, float angle_pitch);
/**  完全空气阻力模型 **/
static double ballisticSolver(float horizontal, float vertical, double bullet_speed, double k);
/**  Pitch轴弹道补偿  **/
static float Get_Pitch_Angle_Compensation(float horizontal, float vertical, float bullet_speed, float k);
/** 根据最优决策得出被击打装甲板 自动解算弹道 */
static void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z);
/**  PITCH轴补偿  **/
float pitchTrajectoryCompensation(float s, float z, float v);
/**  单向弹道解算  **/
static float monoDirectionalAirResistanceModel(float s, float v, float angle);
/**  全向弹道补偿  **/
static float Bullet_Offset(float horizontal, float vertical, float bullet_speed, float k);

/**
 * @brief INS更新函数
 */
void Task_INS(void *pvParameters)
{
	 static portTickType currentTime;
     for(;;)
	 {
		 currentTime = xTaskGetTickCount();
		 INS_Task();
		 vTaskDelayUntil(&currentTime, 1);
	}
}
/**
 * @brief 自瞄函数
 * 
 */
void Task_Vision()
{
	static portTickType currentTime;
	Aim_Init();
	for (;;)
	{
		currentTime = xTaskGetTickCount();
	    // 自瞄数据解算
//		Aim_Control();
		vTaskDelayUntil(&currentTime, 2);
	}
}

/** 虚拟串口初始化 **/
void Aim_Init()
{
	/* 获取INS指针  */
	ins = INS_Init();
	// 初始化自卖哦弹道解算参数
	Aim_Rx.Rx_State          = 0; 
	Aim_Rx.Fixed_Center_time = 10;               //静态预测时间(拨弹盘转动，通信延迟等)
	Aim_Rx.Fixed_Armor_time  = 100;
	Aim_Rx.K                 = 0.022928514188f;          //空气阻力系数	
}


/*******************************************************************************/
/* Action Function                                                            */
/*******************************************************************************/
/* 自瞄控制主程序 */
float hight = 0.0, longa = 0.0f;
void Aim_Control()
{
	/* 自瞄系统正在测试中 */
//	static int16_t Shoot_time_Gap = 0, Shoot_time_cnt = 0;      //射击间隔(击打能量机关需要间隔且单发)
	static float Bullet_fly_time = 0 , Bullet_Speed = 15.0f;//动态预测时间
	static  uint8_t idx = 0;  //索引
	int use_1 = 1;

	/** 测量弹速 **/
	// receive_data.shoot_referee_data.bullet_speed_now ? ( Bullet_Speed = Bullet_Speed * 0.05f + 0.95f * receive_data.shoot_referee_data.bullet_speed_now) 
								//: ( Bullet_Speed = 25.0f);
	/**  串口检测  **/
	if(UsbDog->state == Dog_Online) {
		/* 未识别到目标退出自瞄 */
		if(ReceiveVisionData.data.tracking == 0) {
		  Aim_Ref.auto_mode = auto_aim_off;
		  return;
		}
		/* Center_Pose更新(预测) */
		yaw_v = -ReceiveVisionData.data.v_yaw;

		/**  对方车体中心距离测量  **/
		Aim_Rx.Predicted_Center_Pose.HorizontalDistance = DistanceHorizontal(Aim_Rx.Predicted_Center_Pose);
//		Aim_Rx.Tar_horizontal        = Aim_Rx.Predicted_Center_Pose.HorizontalDistance;
//		Aim_Rx.Tar_vertical          = Aim_Rx.Predicted_Center_Pose.z;

//		/**  计算弹丸飞行时间  **/ 
//		Bullet_fly_time       = Get_bullet_fly_time(Aim_Rx.Tar_horizontal, Bullet_Speed, Aim_Rx.Tar_angle_pitch + Aim_Rx.PitchOffset);
//		Aim_Rx.PitchOffset    = Get_Pitch_Angle_Compensation(Aim_Rx.Tar_horizontal, Aim_Rx.Tar_vertical, Bullet_Speed, Aim_Rx.K);  //单方向空气阻力
		  
		/**  时间戳预测  **/
		Aim_Rx.Predicted_Center_time = (Aim_Rx.Fixed_Center_time + BULLET_FIRE_TIME  + Aim_Rx.Predicted_Gap + Bullet_fly_time) / 1000.0f;  //预测时间ms
		Aim_Rx.Predicted_Armor_time  = (Aim_Rx.Fixed_Armor_time  + BULLET_FIRE_TIME  + Aim_Rx.Predicted_Gap + Bullet_fly_time) / 1000.0f;
		
		/**  根据时间戳预测对方中心坐标  **/        
		Aim_Rx.Predicted_Center_Pose.x = ReceiveVisionData.data.x + Aim_Rx.Predicted_Center_time * ReceiveVisionData.data.vx;
		Aim_Rx.Predicted_Center_Pose.y = ReceiveVisionData.data.y + Aim_Rx.Predicted_Center_time * ReceiveVisionData.data.vy;
		Aim_Rx.Predicted_Center_Pose.z = ReceiveVisionData.data.z + Aim_Rx.Predicted_Center_time * ReceiveVisionData.data.vz;
		Aim_Rx.Predicted_Center_Yaw    = ReceiveVisionData.data.yaw + Aim_Rx.Predicted_Center_time * yaw_v * 2.0f;
		
		/**  整车解算  **/
		//前哨站
		if(ReceiveVisionData.data.armors_num == ARMOR_NUM_OUTPOST){
			for (int i = 0; i < 3; i++) {
				  float tmp_yaw = Aim_Rx.Predicted_Center_Yaw + i * 2.0 * PI / 3.0;  // 2/3PI
				  float r =  (ReceiveVisionData.data.r1 + ReceiveVisionData.data.r2)/2;   //理论上r1=r2 这里取个平均值
				  Aim_Rx.Predicted_Armor_Pose[i].x = Aim_Rx.Predicted_Center_Pose.x - r*sin(tmp_yaw);
				  Aim_Rx.Predicted_Armor_Pose[i].y = Aim_Rx.Predicted_Center_Pose.y + r*cos(tmp_yaw);
				  Aim_Rx.Predicted_Armor_Pose[i].z = Aim_Rx.Predicted_Center_Pose.z;
				  Aim_Rx.Predicted_Armor_Yaw[i] = tmp_yaw;
			}
			//  选板
		} else {
			for (uint8_t i = 0; i < 4 ; i++) {
				float tmp_yaw = Aim_Rx.Predicted_Center_Yaw + i  * PI / 2.0;  //PI/2
				// 装甲板不同高度
				float r = use_1 ? ReceiveVisionData.data.r1 : ReceiveVisionData.data.r2;
				// 计算各个装甲板位置
				Aim_Rx.Predicted_Armor_Pose[i].x = Aim_Rx.Predicted_Center_Pose.x - r*sin(tmp_yaw);
				Aim_Rx.Predicted_Armor_Pose[i].y = Aim_Rx.Predicted_Center_Pose.y + r*cos(tmp_yaw);
				Aim_Rx.Predicted_Armor_Pose[i].z = use_1 ? Aim_Rx.Predicted_Center_Pose.z : Aim_Rx.Predicted_Center_Pose.z + ReceiveVisionData.data.dz;
				Aim_Rx.Predicted_Armor_Yaw[i] = tmp_yaw;
				use_1 = !use_1;
			}
			//2种常见决策方案：
			//1.计算枪管到目标装甲板yaw最小的那个装甲板
			//2.计算距离最近的装甲板
		
			/** 计算枪管到目标装甲板yaw最小的那个装甲板 */
			float yaw_diff_min = fabsf(Aim_Ref.yaw - Aim_Rx.Predicted_Armor_Yaw[0]);
			for (uint8_t i = 1; i < 4 ; i++)  {
				float temp_yaw_diff = fabsf(Aim_Ref.yaw - Aim_Rx.Predicted_Armor_Yaw[i]);
				if (temp_yaw_diff < yaw_diff_min) {
					yaw_diff_min = temp_yaw_diff;
					idx = i;
				}
			}
			/**  选择最近的装甲板  **/
			armor_x = Aim_Rx.Predicted_Armor_Pose[idx].x /*+ x_v * Aim_Rx.Predicted_Armor_time*/;
			armor_y = Aim_Rx.Predicted_Armor_Pose[idx].y /*+ y_v * Aim_Rx.Predicted_Armor_time*/;
			armor_z = Aim_Rx.Predicted_Armor_Pose[idx].z /*+ z_v * Aim_Rx.Predicted_Armor_time*/;
			
						/* 得到期望角度（角度 + 弹道补偿）*/
			/**  PITCH轴弹道解算  **/
			// arm_sqrt_f32(armor_x*armor_x+armor_y*armor_y, &longa);
			longa = sqrt(Aim_Rx.Predicted_Center_Pose.x*Aim_Rx.Predicted_Center_Pose.x+Aim_Rx.Predicted_Center_Pose.y*Aim_Rx.Predicted_Center_Pose.y) - X_BIAS;
			hight = armor_z + Z_BIAS;
		    
			/* 是否开启弹道补偿*/
		    if(Aim_Rx.Predicted_Center_Pose.HorizontalDistance <= 2.0f) {
				Aim_Ref.auto_mode = auto_aim_normal;
			     Aim_Ref.pitch  = atan2(hight, longa ) * 180.0f / PI;	
		     } 
			else
			{
				 Aim_Ref.auto_mode = auto_aim_distant;
		       	 Aim_Ref.pitch  = pitchTrajectoryCompensation(Aim_Rx.Predicted_Center_Pose.HorizontalDistance, armor_z + Z_BIAS, Bullet_Speed) * 180.0f / PI; //Bullet_Offset(longa, hight, Bullet_Speed, Aim_Rx.K);
			 }
//			temp_pitch = pitchTrajectoryCompensation(Aim_Rx.Predicted_Center_Pose.HorizontalDistance, Aim_Rx.Predicted_Center_Pose.z + Z_BIAS, Bullet_Speed);
//			temp_pitch = 

//			 if(temp_pitch)
//                 Aim_Ref.pitch = (float)temp_pitch / PI * 180.0f;
//			else 
//				 Aim_Ref.pitch = atan2(longa , hight) * 180.0f / PI;
			
			// 超过PITCH轴极限角度
			if(Aim_Ref.pitch >= 34.0f || Aim_Ref.pitch < -9.0f)
				Aim_Ref.auto_mode = auto_aim_off;
			
//			Aim_Ref.horizontal = Aim_Rx.Tar_horizontal;
            Aim_Ref.number = ReceiveVisionData.data.armors_num;
			/**  YAW轴计算  **/ 
			if(armor_x || armor_y)
				Aim_Ref.yaw = (float)(atan2(Aim_Rx.Predicted_Center_Pose.y, Aim_Rx.Predicted_Center_Pose.x)) * 180.0f / PI
					+ ins->YawRoundCount * 360.0f; /* 多圈计算 */
						// yaw_v * 1000.0f / Aim_Rx.Predicted_Gap;  // 速度前馈
		}
		// PID前馈速度
		Aim_Ref.yaw_vel = yaw_v;
    } 
	else   /**  串口中断  **/
	{
		Aim_Ref.auto_mode = auto_aim_off;
	}
	
	if(Aim_Action == AIM_STOP)
		Aim_Ref.auto_mode = auto_aim_off;
	
	Aim_Ref.aim_action = Aim_Action;
	Aim_Ref.usb_state  = (uint8_t )UsbDog->state;
}


uint32_t AIM_SHOOT_CNT = 0, LAST_AIM_SHOOT_CNT = 0;
/** 自瞄发射 **/
void Aim_Shoot()
{
	Aim_Ref.fire_on = 0;
	static float Data,LastData;
	
	Data = Aim_Rx.Predicted_Center_Yaw;
	
	AIM_SHOOT_CNT = HAL_GetTick() - LAST_AIM_SHOOT_CNT;
//	if(Data == LastData)Aim_Ref.fire_flag = 0;
//	Aim_Ref.fire_flag = 0;
	float thersold = atan2(AM02_ARMOR_X / 2.0f, Aim_Rx.Predicted_Center_Pose.HorizontalDistance);
	if(Aim_Ref.aim_action >= AIM_STOP)
	{
		 Aim_Ref.fire_on = 1;
		 if(ABS(Aim_Ref.yaw - ins->ContinuousYaw) <= thersold && ReceiveVisionData.data.tracking && Aim_Ref.aim_action == AIM_AUTO && AIM_SHOOT_CNT >= 500)
		 {
			Aim_Ref.fire_flag = 1;
			AIM_SHOOT_CNT = 0;
	        LAST_AIM_SHOOT_CNT = HAL_GetTick();
		 }
	}
	LastData = Data;
}

/* 坐标点到原点水平距离 */
float DistanceHorizontal(Pose_t pose){
    return sqrt(  pose.x * pose.x + pose.y * pose.y);
}

/* 坐标点到原点的距离 */
float DistanceToOrigin(Pose_t pose){
    return sqrt(  pose.x * pose.x + pose.y * pose.y + pose.z * pose.z);
}

/* 获得子弹飞行时间 */
float Get_bullet_fly_time( float horizontal, float bullet_speed, float angle_pitch) {
    float t;
    t = (float)((exp(Aim_Rx.K * horizontal) - 1) / (Aim_Rx.K * bullet_speed * cosf(angle_pitch)));//水平方向求得飞行时间 t
    if(t < 0){
        //由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
        //重置t，防止下次调用会出现nan
        t = 0;
        return 0;
    }
    return t;
}

/* 求得Pitch轴角度补偿(单方向空气阻力，适用于远距离，小角度) */
float Get_Pitch_Angle_Compensation(float horizontal, float vertical, float bullet_speed, float k) {
    float temp_vertical, actual_vertical, error_vertical;
    float pitch, pitch_new;
    
    pitch = atan2(vertical, horizontal);
    temp_vertical = vertical;
    //迭代重力法
    for (uint8_t i = 0; i < 20; i++){
        pitch_new = atan2(temp_vertical, horizontal);
        actual_vertical = monoAirResistance_Model(horizontal, bullet_speed, pitch_new, k);
        error_vertical = 0.3f * (vertical - actual_vertical);
        temp_vertical = temp_vertical + error_vertical;
        if (fabsf(error_vertical) < 0.0001f)
            break;
    }
    return (pitch_new - pitch) * 180 / PI;
}

/* 单方向空气阻力弹道模型 利用飞行速度求Pitch轴竖直高度  */
float monoAirResistance_Model(float horizontal, float bullet_speed, float angle_pitch, float k){
    float actual_vertical, t;
    
    t = Get_bullet_fly_time(horizontal, bullet_speed, angle_pitch);
    actual_vertical = bullet_speed * sin(angle_pitch) * t - GRAVITY * t * t / 2; //得出子弹会打到的竖直高度
//    actual_vertical = (1 / k) * log(k * bullet_speed * sin(angle_pitch) * t + 1) - (GRAVITY * t*t)/( ( 1/k ) * (2 * t * bullet_speed * sin(angle_pitch) + 2) ); // 弹丸上升阻力模型
    return actual_vertical;
}

/* 完美弹道模型 */
//double ballisticSolver(float horizontal, float vertical, double bullet_speed, double k)
//{
//// FIX ME:  NaN Failed 是否因为 vertical 有时候为负
//// TODO: 世界坐标系下进行迭代
//    double pitch = atan2(vertical, horizontal);
////	if(vertical >= 0 && pitch <= 1.5f*PI/180.0f)
////		pitch = 1.5f * PI/180.0f
////	else if(vertical >= 0 && pitch <= 1.5f*PI/180.0f)
//    double temp_vertical = vertical;
//    double pitch_new = pitch;

//    // 迭代求解
//    for (int i = 0; i < 20; ++i) {
//        double x = 0.0;
//        double y = 0.0;
//        double p = tan(pitch_new);
//        double v = bullet_speed;
//        double u = v / sqrt(1 + p * p);
//        double delta_x = horizontal / 20;
//        
//        // 使用四阶龙格-库塔法求解微分方程，步长决定精度
//        for (int j = 0; j < 20; ++j) 
//		{
//            double k1_u = -k * u * sqrt(1 + (p*p) );
//            double k1_p = -GRAVITY / (u*u);
//            double k1_u_sum = u + k1_u * (delta_x / 2);
//            double k1_p_sum = p + k1_p * (delta_x / 2);

//            double k2_u = -k * k1_u_sum * sqrt(1 + (k1_p_sum*k1_p_sum) );
//            double k2_p = -GRAVITY / (k1_u_sum*k1_u_sum);
//            double k2_u_sum = u + k2_u * (delta_x / 2);
//            double k2_p_sum = p + k2_p * (delta_x / 2);

//            double k3_u = -k * k2_u_sum * sqrt(1 + (k2_p_sum*k2_p_sum) );
//            double k3_p = -GRAVITY / (k2_u_sum*k2_u_sum);
//            double k3_u_sum = u + k3_u * (delta_x / 2);
//            double k3_p_sum = p + k3_p * (delta_x / 2);

//            double k4_u = -k * k3_u_sum * sqrt(1 + (k3_p_sum*k3_p_sum) );
//            double k4_p = -GRAVITY / (k3_u_sum*k3_u_sum);

//            u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
//            p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);
//           
//            x += delta_x;
//            y += p * delta_x;
//        }

//        double error = vertical - y;

//        // 如果误差满足停止条件，则跳出迭代
//        if (fabs(error) <= 0.00001) {
//            break;
//        } else {
//            temp_vertical += error;
//            pitch_new = atan2(temp_vertical, horizontal);
//        }
//    }
//    // 返回仰角修正值
//    return (pitch_new - pitch) * 180 / PI;
//}

static float k1_u, k1_p, k1_u_sum, k1_p_sum;
static float k2_u, k2_p, k2_u_sum, k2_p_sum;
static float k3_u, k3_p, k3_u_sum, k3_p_sum;
static float k4_u, k4_p;

float Bullet_Offset(float horizontal, float vertical, float bullet_speed, float k)
{
//    float temp_vertical = vertical;
    float pitch_new = atan2(vertical, horizontal);
	float error, temp_vertical;

    for(uint8_t i = 0; i <= 20 ; i ++)
	{
		// 水平位移     竖直位移
		float x = 0.0f, y = 0.0f;
		// 飞行速度 
		float v = bullet_speed;
		// 飞行斜率
        float p = tan(pitch_new);
        float u = v / sqrt(1 + p * p);
		// 四阶龙格-库塔 迭代步长
        float delta_x = horizontal / 40;
		// 进行迭代
		for(uint16_t j = 0; j <= 40; j++) {
            k1_u = -k * u * sqrt(1 + (p*p));
            k1_p = -GRAVITY / (u * u);
            k1_u_sum = u + k1_u * (delta_x / 2);
            k1_p_sum = p + k1_p * (delta_x / 2);
    
            k2_u = -k * k1_u_sum * sqrt(1 + (k1_p_sum*k1_p_sum) );
            k2_p = -GRAVITY / (k1_u_sum * k1_u_sum);
            k2_u_sum = u + k2_u * (delta_x / 2);
            k2_p_sum = p + k2_p * (delta_x / 2);
            
            k3_u = -k * k2_u_sum * sqrt(1 + (k2_p_sum*k2_p_sum) );
            k3_p = -GRAVITY / (k2_u_sum*k2_u_sum);
            k3_u_sum = u + k3_u * (delta_x);
            k3_p_sum = p + k3_p * (delta_x);
            
            k4_u = -k * k3_u_sum * sqrt(1 + (k3_p_sum*k3_p_sum));
            k4_p = -GRAVITY / (k3_u_sum*k3_u_sum);
            
            u += (delta_x / 6) * (k1_u + 2 * k2_u + 2 * k3_u + k4_u);
            p += (delta_x / 6) * (k1_p + 2 * k2_p + 2 * k3_p + k4_p);

            x += delta_x;
            y += p * delta_x;
		}
		// 误差更新
		error = vertical - y;        
        // 误差判断
        if(fabs(error) < 0.01f)
	    {
            pitch_new = atan2(temp_vertical , horizontal);
            break;
		}
        else
		{		
            temp_vertical += error;
            pitch_new = atan2(temp_vertical , horizontal);
	    }
	}
    return pitch_new;
}

/**
 * @brief pitch轴解算
 * @param s:m 距离
 * @param z:m 高度
 * @param v:m/s
 * @return angle_pitch:rad
*/
float pitchTrajectoryCompensation(float s, float z, float v)
{
    float z_temp, z_actual, dz;
    float angle_pitch;
    int i = 0;
    z_temp = z;
    // iteration
    for (i = 0; i < 20; i++)
    {
        angle_pitch = atan2(z_temp, s); // rad
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        if(z_actual == 0)
        {
            angle_pitch = 0;
            break;
        }
        dz = 0.3*(z - z_actual);
        z_temp = z_temp + dz;
        if (fabsf(dz) < 0.001)
        {
            break;
        }
    }
    return angle_pitch;
}

/**
 * @brief 单方向空气阻力弹道模型
 * @param s:m 距离
 * @param v:m/s 速度
 * @param angle:rad 角度
 * @return z:m
*/
float monoDirectionalAirResistanceModel(float s, float v, float angle)
{
    float z, t;
    //t为给定v与angle时的飞行时间
    t = (float)((exp(Aim_Rx.K * s) - 1) / (Aim_Rx.K * v * arm_cos_f32(angle)));
    if(t < 0)
    {
        //由于严重超出最大射程，计算过程中浮点数溢出，导致t变成负数
        //重置t，防止下次调用会出现nan
        t = 0;
        return 0;
    }
    //z为给定v与angle时的高度
    z = (float)(v * arm_sin_f32(angle) * t - GRAVITY * t * t / 2);
    return z;
}

/*旋转矩阵*/
/* void applyRotation(const RotationMatrix_t *rotationMatrix, Aim_Rx_info *info) {
    // Define matrices A and B
    arm_matrix_instance_f32 matA, matB, matResult;
    float32_t A_data[3][3], B_data[3][1], result_data[3][1];

    // Copy rotation matrix data to matrix A
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            A_data[i][j] = rotationMatrix->r[i][j];
        }
    }
    
    // Copy pose data to matrix B
    B_data[0][0] = info->pose.pos_X;
    B_data[1][0] = info->pose.pos_Y;
    B_data[2][0] = info->pose.pos_Z;
    
    // Initialize matrix structures
    arm_mat_init_f32(&matA, 3, 3, (float32_t *)A_data);
    arm_mat_init_f32(&matB, 3, 1, (float32_t *)B_data);
    arm_mat_init_f32(&matResult, 3, 1, (float32_t *)result_data);
    
    // Perform matrix multiplication: Result = A * B
    arm_mat_mult_f32(&matA, &matB, &matResult);

    // Copy result back to pose
    info->pose.pos_X = result_data[0][0];
    info->pose.pos_Y = result_data[1][0];
    info->pose.pos_Z = result_data[2][0];
} */

