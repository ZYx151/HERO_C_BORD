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
// 打弹延迟 弹丸从接收到发射命令到离开摩擦轮
#define BULLET_FIRE_TIME 165
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

/**  功能函数  **/
/**  坐标点到原点的水平距离  **/
static float DistanceHorizontal(Pose_t pose);
/**  坐标点到原点的距离  **/
static float DistanceToOrigin(Pose_t pose);
/**  空气阻力模型  **/
//static float monoAirResistance_Model(float horizontal, float bullet_speed, float angle_pitch, float k);
/**  获得飞行时间  **/
//static float Get_Bullet_Time(float horizontal, float vertical, double bullet_speed);
/**  完全空气阻力模型 **/
static double ballisticSolver(float horizontal, float vertical, double bullet_speed, double k);
/**  Pitch轴弹道补偿  **/
float Get_Pitch_Angle_Compensation(float horizontal, float vertical, float bullet_speed);
/** 根据最优决策得出被击打装甲板 自动解算弹道 */
static void autoSolveTrajectory(float *pitch, float *yaw, float *aim_x, float *aim_y, float *aim_z);
/**  PITCH轴补偿  **/
float pitchTrajectoryCompensation(float s, float z, float v);
/**  单向弹道解算  **/
static float monoDirectionalAirResistanceModel(float s, float v, float angle);
/**  全向弹道补偿  **/
static float Bullet_Offset(float horizontal, float vertical, float bullet_speed, float k);
float computePitchBisection(float horizontal, float vertical, float bulletSpeed);
/**  旋转矩阵  **/
static void transformPointGunLinkToGun(float pointInGunLink[3],float xOffest, float yOffest, float zOffest, float pointInGun[3]) ;
static void transformPointGimbalToGunLink(float pointInGimbal[3], float roll, float pitch, float yaw, float pointOutGunLink[3]);
static void rotationMatrixFromRPY(float roll, float pitch, float yaw, float R[3][3]);

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
		Aim_Control();
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
	Aim_Rx.Fixed_Center_time = 2;               //静态预测时间(拨弹盘转动，通信延迟等)
	Aim_Rx.K                 = 0.47f * 1.169f * (2.0f * M_PI * 0.02125f * 0.02125f) / 2.0f / 0.041f;          //空气阻力系数	
	Aim_Rx.Predicted_Gap = 0 ; // 手动赋值
}


/*******************************************************************************/
/* Action Function                                                            */
/*******************************************************************************/
/* 自瞄控制主程序 */
float hight = 0.0, longa = 0.0f;
float Bullet_fly_time = 0 , Bullet_Speed = 15.0f;//动态预测时间
void Aim_Control()
{
	/* 自瞄系统正在测试中 */
	static int16_t Shoot_time_Gap = 0, Shoot_time_cnt = 0;      //射击间隔(击打能量机关需要间隔且单发)
	static  uint8_t idx = 0;  //索引
	int use_1 = 1;

	/**  串口检测  **/
	if(UsbDog->state == Dog_Online) {
		/* 未识别到目标退出自瞄 */
		if(ReceiveVisionData.data.tracking == 0) {
		  Aim_Ref.auto_mode = auto_aim_off;
		  return;
		}
		/* Center_Pose更新(预测) */
		Aim_Rx.yaw_v = ReceiveVisionData.data.v_yaw;
        Aim_Rx.Predicted_Center_Pose.x = ReceiveVisionData.data.x;
        Aim_Rx.Predicted_Center_Pose.y = ReceiveVisionData.data.y;
        Aim_Rx.Predicted_Center_Pose.z = ReceiveVisionData.data.z;

		/**  对方车体中心距离测量  **/
		Aim_Rx.Predicted_Center_Pose.HorizontalDistance = DistanceHorizontal(Aim_Rx.Predicted_Center_Pose);
        Aim_Ref.horizontal = DistanceToOrigin(Aim_Rx.Predicted_Center_Pose);
		Aim_Ref.id         = ReceiveVisionData.data.id;

		/**  计算弹丸飞行时间  **/
		float angle_pitch = atan2(Aim_Rx.Predicted_Center_Pose.HorizontalDistance, Aim_Rx.Predicted_Center_Pose.z);
		Bullet_fly_time = (pow(2.718281828, Aim_Rx.K * Aim_Ref.horizontal) - 1) / (Aim_Rx.K * Bullet_Speed * arm_cos_f32(angle_pitch));		  
		/**  时间戳预测  **/
		Aim_Rx.Predicted_Center_time = (Aim_Rx.Fixed_Center_time + BULLET_FIRE_TIME  + Aim_Rx.Predicted_Gap + Bullet_fly_time) / 1000.0f;  //预测时间ms
//		Aim_Rx.Predicted_Armor_time  = (Aim_Rx.Fixed_Armor_time  + BULLET_FIRE_TIME  + Aim_Rx.Predicted_Gap + Bullet_fly_time) / 1000.0f;

		DEADLINE_LIMIT(ReceiveVisionData.data.vz, 0.1);
		DEADLINE_LIMIT(ReceiveVisionData.data.vx, 0.1);
		DEADLINE_LIMIT(ReceiveVisionData.data.vy, 0.1);
		DEADLINE_LIMIT(Aim_Rx.yaw_v, 0.1);
				
		/**  整车解算  **/
		//前哨站
		if(ReceiveVisionData.data.armors_num == ARMOR_NUM_OUTPOST) {
		    /**  根据时间戳预测对方中心坐标  **/        
			Aim_Rx.Predicted_Center_Pose.x = ReceiveVisionData.data.x;
			Aim_Rx.Predicted_Center_Pose.y = ReceiveVisionData.data.y;
			Aim_Rx.Predicted_Center_Pose.z = ReceiveVisionData.data.z;
			Aim_Rx.Predicted_Center_Yaw    = ReceiveVisionData.data.yaw + Aim_Rx.Predicted_Center_time * Aim_Rx.yaw_v;

			for (int i = 0; i < 3; i++) {
				  float tmp_yaw = Aim_Rx.Predicted_Center_Yaw + i * 2.0 * M_PI / 3.0;  // 2/3PI
				  float r =  (ReceiveVisionData.data.r1 + ReceiveVisionData.data.r2)/2;   //理论上r1=r2 这里取个平均值
				  Aim_Rx.Predicted_Armor_Pose[i].x = Aim_Rx.Predicted_Center_Pose.x - r*sin(tmp_yaw);
				  Aim_Rx.Predicted_Armor_Pose[i].y = Aim_Rx.Predicted_Center_Pose.y + r*cos(tmp_yaw);
				  Aim_Rx.Predicted_Armor_Pose[i].z = Aim_Rx.Predicted_Center_Pose.z;
				  Aim_Rx.Predicted_Armor_Yaw[i] = tmp_yaw;
			}
			/** 选择前哨站中心 **/ 
			Aim_Rx.center_x = Aim_Rx.Predicted_Center_Pose.x;
			Aim_Rx.center_y = Aim_Rx.Predicted_Center_Pose.y;
			Aim_Rx.center_z = Aim_Rx.Predicted_Center_Pose.z;
			/** 计算枪管到目标装甲板yaw最小的那个装甲板 */
			float yaw_diff_min = fabsf(Aim_Ref.yaw - Aim_Rx.Predicted_Armor_Yaw[0]);
			for (uint8_t i = 1; i < 4 ; i++)  {
				float temp_yaw_diff = fabsf(Aim_Ref.yaw - Aim_Rx.Predicted_Armor_Yaw[i]);
				if (temp_yaw_diff < yaw_diff_min) {
					yaw_diff_min = temp_yaw_diff;
					idx = i;
				}
			}
			Aim_Rx.armor_x = Aim_Rx.Predicted_Armor_Pose[idx].x;
			Aim_Rx.armor_y = Aim_Rx.Predicted_Armor_Pose[idx].y;
			Aim_Rx.armor_z = Aim_Rx.Predicted_Armor_Pose[idx].z;
			Aim_Rx.armor_yaw = Aim_Rx.Predicted_Armor_Yaw[idx];
		} else {
			/**  根据时间戳预测对方中心坐标  **/        
			Aim_Rx.Predicted_Center_Pose.x = ReceiveVisionData.data.x + Aim_Rx.Predicted_Center_time * ReceiveVisionData.data.vx*1.5;
			Aim_Rx.Predicted_Center_Pose.y = ReceiveVisionData.data.y + Aim_Rx.Predicted_Center_time * ReceiveVisionData.data.vy*1.5;
			Aim_Rx.Predicted_Center_Pose.z = ReceiveVisionData.data.z + Aim_Rx.Predicted_Center_time * ReceiveVisionData.data.vz*1.5;
			Aim_Rx.Predicted_Center_Yaw    = ReceiveVisionData.data.yaw + Aim_Rx.Predicted_Center_time * Aim_Rx.yaw_v;
			for(uint8_t i = 0; i < 4 ; i++) {
				float tmp_yaw = 0; 
				if(fabsf(ReceiveVisionData.data.v_yaw)<0.1){
					tmp_yaw = ReceiveVisionData.data.yaw;
				} else {
					tmp_yaw = Aim_Rx.Predicted_Center_Yaw + i *  M_PI / 2.0; 								
				}
				// 装甲板不同高度
				float r = use_1 ? ReceiveVisionData.data.r1 : ReceiveVisionData.data.r2;
				// 计算各个装甲板位置
				Aim_Rx.Predicted_Armor_Pose[i].x = Aim_Rx.Predicted_Center_Pose.x - r*sin(tmp_yaw);
				Aim_Rx.Predicted_Armor_Pose[i].y = Aim_Rx.Predicted_Center_Pose.y + r*cos(tmp_yaw);
				Aim_Rx.Predicted_Armor_Pose[i].z = use_1 ? Aim_Rx.Predicted_Center_Pose.z : Aim_Rx.Predicted_Center_Pose.z + ReceiveVisionData.data.dz;
				Aim_Rx.Predicted_Armor_Yaw[i]    = tmp_yaw;
				use_1 = !use_1;
			}
			//2种常见决策方案:
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
			/** 选择跟随对方车体中心**/ 
			Aim_Rx.armor_x = Aim_Rx.Predicted_Center_Pose.x;
			Aim_Rx.armor_y = Aim_Rx.Predicted_Center_Pose.y;
			Aim_Rx.armor_z = Aim_Rx.Predicted_Armor_Pose[idx].z;			
			
			Aim_Rx.center_x = Aim_Rx.Predicted_Center_Pose.x;
			Aim_Rx.center_y = Aim_Rx.Predicted_Center_Pose.y;
			Aim_Rx.center_z = Aim_Rx.Predicted_Armor_Pose[idx].z;			
			Aim_Rx.armor_yaw = Aim_Rx.Predicted_Armor_Yaw[idx];

			/* 得到期望角度（角度 + 弹道补偿）*/
			/**  PITCH轴弹道解算  **/
			// arm_sqrt_f32(armor_x*armor_x+armor_y*armor_y, &longa);
//			longa = sqrt(Aim_Rx.Predicted_Center_Pose.x*Aim_Rx.Predicted_Center_Pose.x+Aim_Rx.Predicted_Ce nter_Pose.y*Aim_Rx.Predicted_Center_Pose.y) - X_BIAS;
//			hight = armor_z + Z_BIAS;
//		    
			/* 是否开启弹道补偿*/
//		    if(Aim_Rx.Predicted_Center_Pose.HorizontalDistance <= 2.0f) {
//				Aim_Ref.auto_mode = auto_aim_normal;
//			     Aim_Ref.pitch  = atan2(hight, longa ) * 180.0f / PI;	
//		     } 
//			else
//			{
//				 Aim_Ref.auto_mode = auto_aim_distant;
//		       	 Aim_Ref.pitch  = pitchTrajectoryCompensation(Aim_Rx.Predicted_Center_Pose.HorizontalDistance, armor_z + Z_BIAS, Bullet_Speed) * 180.0f / PI; //Bullet_Offset(longa, hight, Bullet_Speed, Aim_Rx.K);
//			}
//			
//			// 超过PITCH轴极限角度
//			if(Aim_Ref.pitch >= 34.0f || Aim_Ref.pitch < -9.0f)
//				Aim_Ref.auto_mode = auto_aim_off;
//			
////			Aim_Ref.horizontal = Aim_Rx.Tar_horizontal;
//            Aim_Ref.number = ReceiveVisionData.data.armors_num;
//			/**  YAW轴计算  **/ 
//			if(armor_x || armor_y)
//				Aim_Ref.yaw = (float)(atan2(Aim_Rx.Predicted_Center_Pose.y, Aim_Rx.Predicted_Center_Pose.x)) * 180.0f / PI
//					+ ins->YawRoundCount * 360.0f; /* 多圈计算 */
						// yaw_v * 1000.0f / Aim_Rx.Predicted_Gap;  // 速度前馈
		}
		
		Aim_Offset(ins->Roll*ANGLE2RADIAN, ins->Pitch*ANGLE2RADIAN, ins->ContinuousYaw*ANGLE2RADIAN, Bullet_Speed);
		// PID前馈速度
		Aim_Ref.yaw_vel = Aim_Rx.yaw_v;
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
//**************************************  自瞄补偿角度 **********************************//
/* 自瞄补偿 */
    //在gun_link 坐标系下的点
    float pointGunLink[3] = { 0.0f, 0.0f, 0.0f };
    //在gun坐标系下的点
    float pointGun[3] = { 0.0f, 0.0f, 0.0f };
    // 陀螺仪到枪口的偏移
    float xOffest = 0.0f;
    float yOffest = 0.0f;
    float zOffest = 0.0f;
	float pitch_offset = 0.0f, yaw_offset = 0.0f, flaot_horizon;

void Aim_Offset(float roll,float pitch,float yaw, float bullet_speed)
{
	//在 gimbal 坐标系下的装甲板的三维坐标点:
	float pointGimbal[3];
	pointGimbal[0] = Aim_Rx.armor_x;
	pointGimbal[1] = Aim_Rx.armor_y;
	pointGimbal[2] = Aim_Rx.armor_z;
	// 陀螺仪发给视觉的rpy
	//    float roll = 0.0f;
	//    float pitch = 0.0f;
	//    float yaw = 0.5236f;
	transformPointGimbalToGunLink(pointGimbal, roll, pitch, yaw, pointGunLink);
	transformPointGunLinkToGun(pointGunLink, xOffest, yOffest, zOffest, pointGun);

	flaot_horizon = sqrt(Aim_Rx.armor_x*Aim_Rx.armor_x + Aim_Rx.armor_y*Aim_Rx.armor_y);
	
    if(Aim_Rx.Predicted_Center_Pose.HorizontalDistance <= 2.5f) {
		Aim_Ref.auto_mode = auto_aim_normal;
	    Aim_Ref.pitch = -atan2(pointGun[2], pointGun[0]) * 180 / M_PI 
					 + ins->Pitch;
	} 
	else
	{
		 Aim_Ref.auto_mode = auto_aim_distant;                                                 
		 Aim_Ref.pitch = Get_Pitch_Angle_Compensation(flaot_horizon, Aim_Rx.Predicted_Center_Pose.z, bullet_speed);
//	     pitch_offset = computePitchBisection(sqrt(pointGun[0] * pointGun[0] + pointGun[1] * pointGun[1] + pointGun[2] * pointGun[2]), Aim_Rx.Predicted_Center_Pose.z, bullet_speed);
//	     Aim_Ref.pitch = -atan2(pointGun[2], pointGun[0]) * 180 / M_PI 
//					 + ins->Pitch;
	}

	yaw_offset = atan2(pointGun[1], pointGun[0]) * 180 / M_PI;//计算需要转的角度
	Aim_Ref.yaw = yaw_offset + ins->ContinuousYaw;
}
//****************************  自瞄发射  ****************************************//
/** 自瞄发射 **/
uint32_t AIM_SHOOT_CNT = 0, LAST_AIM_SHOOT_CNT = 0;
void Aim_Shoot()
{
	Aim_Ref.fire_on = 0;
	static float Data,LastData;
	
	Data = Aim_Rx.Predicted_Center_Yaw;
	
	AIM_SHOOT_CNT = HAL_GetTick() - LAST_AIM_SHOOT_CNT;
//	if(Data == LastData)Aim_Ref.fire_flag = 0;
//	Aim_Ref.fire_flag = 0;
		 /* 动态打弹阈值 */
	float thersold;
	if(ReceiveVisionData.data.armors_num == ARMOR_NUM_OUTPOST)
	{
	    thersold = 0;
	} 
	else
	{
		if(ReceiveVisionData.data.id == ARMOR_HERO)
			thersold = atan2(AM12_ARMOR_X/2.0,sqrt(pointGun[0] * pointGun[0] + pointGun[1] * pointGun[1])) * 180 / M_PI * 1.6;
		else 	
			thersold = atan2(AM02_ARMOR_X/2.0,sqrt(pointGun[0] * pointGun[0] + pointGun[1] * pointGun[1])) * 180 / M_PI * 1.6;
	}
	
	if(Aim_Ref.aim_action >= AIM_STOP)
	{
		 Aim_Ref.fire_on = 1;
		 if(ABS(Aim_Ref.yaw - ins->ContinuousYaw) <= thersold && ReceiveVisionData.data.tracking && Aim_Ref.aim_action == AIM_AUTO && AIM_SHOOT_CNT >= 100)
		 {
			Aim_Ref.fire_flag = 1;
			AIM_SHOOT_CNT = 0;
	        LAST_AIM_SHOOT_CNT = HAL_GetTick();
		 }
	}
	LastData = Data;
}
/*****************************************  功能函数 *****************************/
/* 坐标点到原点水平距离 */
float DistanceHorizontal(Pose_t pose){
    return sqrt(  pose.x * pose.x + pose.y * pose.y);
}

/* 坐标点到原点的距离 */
float DistanceToOrigin(Pose_t pose){
    return sqrt(  pose.x * pose.x + pose.y * pose.y + pose.z * pose.z);
}

/* 求得Pitch轴角度补偿(单方向空气阻力，适用于远距离，小角度) */
//float Get_Pitch_Angle_Compensation(float horizontal, float vertical, float bullet_speed, float k) {
//    float temp_vertical, actual_vertical, error_vertical;
//    float pitch, pitch_new;
//    
//    pitch = atan2(vertical, horizontal);
//    temp_vertical = vertical;
//    //迭代重力法
//    for (uint8_t i = 0; i < 20; i++){
//        pitch_new = atan2(temp_vertical, horizontal);
//        actual_vertical = monoAirResistance_Model(horizontal, bullet_speed, pitch_new, k);
//        error_vertical = 0.3f * (vertical - actual_vertical);
//        temp_vertical = temp_vertical + error_vertical;
//        if (fabsf(error_vertical) < 0.0001f)
//            break;
//    }
//    return (pitch_new - pitch) * 180 / PI;
//}

/* 单方向空气阻力弹道模型 利用飞行速度求Pitch轴竖直高度  */
//float monoAirResistance_Model(float horizontal, float bullet_speed, float angle_pitch, float k){
//    float actual_vertical, t;
//    
////    t = Get_bullet_fly_time(horizontal, bullet_speed, angle_pitch);
//    actual_vertical = bullet_speed * sin(angle_pitch) * t - GRAVITY * t * t / 2; //得出子弹会打到的竖直高度
////    actual_vertical = (1 / k) * log(k * bullet_speed * sin(angle_pitch) * t + 1) - (GRAVITY * t*t)/( ( 1/k ) * (2 * t * bullet_speed * sin(angle_pitch) + 2) ); // 弹丸上升阻力模型
//    return actual_vertical;
//}

/**  二分法进行弹道补偿  **/
float computePitchBisection(float horizontal, float vertical, float bulletSpeed) {
    // 重力加速度
    const float g = 9.8;
    float low = 0.0;

    // 在大部分实际场景下，0~45度足够
    float high = PI / 4.0;

    // 收敛精度角度差小于某个范围就停止
    while (high - low > 1e-7) {
        float mid ;
			  mid = (low + high) / 2.0;

        float vx;
			  vx = bulletSpeed * cos(mid);
        float vy;
    		vy = bulletSpeed * sin(mid);

        float flightTime;
			  flightTime = (vy + sqrt(vy * vy + 2 * g * vertical)) / g;
        float computedHorizontal;
		    computedHorizontal	= vx * flightTime;

        // 计算水平误差
        float error ;
			  error = computedHorizontal - horizontal;

        // 如果误差足够小就退出
        if (fabs(error) < 0.00001) {
            return mid;
        }

        // 二分查找的区间缩放
        if (error > 0) {
            // 表示打得太远，需要减小角度
            high = mid;
        }
        else {
            // 表示打得太近，需要增大角度
            low = mid;
        }
    }

    // 最终 low 或 high 都可作为结果，这里取平均值。
    return (low + high) / 2.0;
}


float Bullet_Offset(float horizontal, float vertical, float bullet_speed, float k)
{
	static float k1_u, k1_p, k1_u_sum, k1_p_sum;
	static float k2_u, k2_p, k2_u_sum, k2_p_sum;
	static float k3_u, k3_p, k3_u_sum, k3_p_sum;
	static float k4_u, k4_p;
	
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
        if(fabs(error) < 0.001f)
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

/**  (参考狼牙战队24英雄自瞄代码重力补偿)基于发射时间的重力补偿  K = 0.022928514188 // 0.097 bullet_speed = 15**/
float Get_Pitch_Angle_Compensation(float horizontal, float vertical, float bullet_speed)
{
	float flyTime = 0.0, angle_pitch = 0.0, delta_z = 0.0;
	
	angle_pitch = atan2(horizontal, vertical);
	for (int i = 0; i < 100; i++) 
	{
		// 计算炮弹的飞行时间
		flyTime = (pow(2.718281828, Aim_Rx.K * horizontal) - 1) / (Aim_Rx.K * bullet_speed * arm_cos_f32(angle_pitch));
		delta_z = vertical - bullet_speed * arm_sin_f32(angle_pitch) * flyTime / arm_cos_f32(angle_pitch) +
				    0.5 * GRAVITY * flyTime * flyTime / arm_cos_f32(angle_pitch) / arm_cos_f32(angle_pitch);
		if (fabs(delta_z) < 0.0001)
			break;
		angle_pitch -= delta_z / (-(bullet_speed * flyTime) / pow(arm_cos_f32(angle_pitch), 2) +
							GRAVITY * flyTime * flyTime / (bullet_speed * bullet_speed) * arm_sin_f32(angle_pitch) / pow(arm_cos_f32(angle_pitch), 3));
	}
	return angle_pitch / M_PI *180.0f;
}

/*旋转矩阵*/
/**
 * 将三维点 pointInGimbal 从 gimbal 坐标系
 * 转换到 gun_link 坐标系。
 *
 * param:
 *  - pointInGimbal: 输入装甲板三维点(在 gimbal 坐标系下)
 *  - roll, pitch, yaw: gimbal_link -> gimbal 的欧拉角
 *                      (即正向时, P_gimbal = R * P_gimbal_link)
 *  - pointOutGimbalLink: 输出装甲板三维点(在 gun_link 坐标系下)
 */
float R[3][3];
void transformPointGimbalToGunLink(float pointInGimbal[3],
    float roll, float pitch, float yaw,
    float pointOutGunLink[3])
{
    // 先获取正向旋转矩阵 R (gimbal_link -> gimbal)
    float x,y,z;
    rotationMatrixFromRPY(roll, pitch, yaw, R);

    // 因为要反向(gimbal -> gun_link)，
    //    对纯旋转来说，逆矩阵 R_inv = R^T

    // 用 R^T * pointInGimbal 得到 pointInGimbalLink
    // R^T = [ R[0][0], R[1][0], R[2][0]
    //         R[0][1], R[1][1], R[2][1]
    //         R[0][2], R[1][2], R[2][2] ]
    x = R[0][0] * pointInGimbal[0] + R[1][0] * pointInGimbal[1] + R[2][0] * pointInGimbal[2];
    y = R[0][1] * pointInGimbal[0] + R[1][1] * pointInGimbal[1] + R[2][1] * pointInGimbal[2];
    z = R[0][2] * pointInGimbal[0] + R[1][2] * pointInGimbal[1] + R[2][2] * pointInGimbal[2];

    pointOutGunLink[0] = x;
    pointOutGunLink[1] = y;
    pointOutGunLink[2] = z;
}
/**
 * 根据欧拉角(roll, pitch, yaw)生成旋转矩阵R (3x3)。
 * 旋转顺序为 Z-Y-X(航向-俯仰-滚转)。
 */
float sr,cr,sp,cp,sy,cy;
void rotationMatrixFromRPY(float roll, float pitch, float yaw, float R[3][3])
{
	
    sr = arm_sin_f32(roll);
    cr = arm_cos_f32(roll);
    sp = arm_sin_f32(pitch);
    cp = arm_cos_f32(pitch);
    sy = arm_sin_f32(yaw);
    cy = arm_cos_f32(yaw);

    // Rz(yaw) * Ry(pitch) * Rx(roll)
    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;

    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;

    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}
void transformPointGunLinkToGun(float pointInGunLink[3],float xOffest, float yOffest, float zOffest, float pointInGun[3]) {
	pointInGun[0] = pointInGunLink[0] + xOffest;
	pointInGun[1] = pointInGunLink[1] + yOffest;
	pointInGun[2] = pointInGunLink[2] + zOffest;
}

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


