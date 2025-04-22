/**
   @brief  RM裁判系统串口通信解包文件
   @data     2024-5-1
   @version    V1.6.2
*/

#ifndef _REFEREE_SYSTEM_H_
#define _REFEREE_SYSTEM_H_

#include "RMQueue.h"
#include "CRC.h"
#define REFEREE_SYSTEM_DISABLE 0
#define REFEREE_SYSTEM_ENABLE  1

/* 获取数据使能 */
#define Match_status_data                                       REFEREE_SYSTEM_ENABLE      //!<@brief 比赛状态
#define Match_result_data                                       REFEREE_SYSTEM_DISABLE      //!<@brief 比赛结果
#define Match_all_robot_HP_data                                 REFEREE_SYSTEM_ENABLE       //!<@brief 全体机器人血量
#define Site_event_data                                         REFEREE_SYSTEM_DISABLE      //!<@brief 场地事件
#define Site_supply_station_action_identification_data          REFEREE_SYSTEM_DISABLE      //!<@brief 场地补给站动作标识
#define Referee_warning_data                                    REFEREE_SYSTEM_DISABLE      //!<@brief 裁判警告
#define Darts_info_data                                         REFEREE_SYSTEM_DISABLE      //!<@brief 飞镖发射相关
#define Robot_state_data                                        REFEREE_SYSTEM_ENABLE       //!<@brief 本机器人状态
#define Real_time_chassis_power_and_shoot_heat_data             REFEREE_SYSTEM_ENABLE       //!<@brief 实时底盘功率和枪口热量
#define Robot_position_data                                     REFEREE_SYSTEM_DISABLE       //!<@brief 本机器人位置
#define Robot_gain_data                                         REFEREE_SYSTEM_ENABLE       //!<@brief 本机器人增益
#define Air_robot_energy_status_data                            REFEREE_SYSTEM_DISABLE       //!<@brief 空中机器人状态
#define Damage_status_data                                      REFEREE_SYSTEM_ENABLE       //!<@brief 扣血原因
#define Real_time_shooting_data                                 REFEREE_SYSTEM_ENABLE       //!<@brief 实时射击信息
#define Ammo_remind_data                                        REFEREE_SYSTEM_ENABLE       //!<@brief 允许发弹量
#define Robot_RFID_state_data                                   REFEREE_SYSTEM_ENABLE       //!<@brief 机器人RFID状态
#define Darts_client_cmd                                        REFEREE_SYSTEM_DISABLE       //!<@brief 飞镖选手端指令
#define Robot_position_to_Sentry_data                           REFEREE_SYSTEM_DISABLE       //!<@brief 本方地面机器人位置
#define Radar_marking_progress_data                             REFEREE_SYSTEM_DISABLE       //!<@brief 雷达标记进度数据
#define Sentry_info_data                                        REFEREE_SYSTEM_DISABLE      //!<@brief 哨兵自主决策信息同步
#define Radar_info_data                                         REFEREE_SYSTEM_DISABLE      //!<@brief 雷达自主决策信息同步
/* CMD_id命令代码内容同上 */
#define CMD_id_Match_status_data                                ((uint16_t)0x0001)          //!<@brief 比赛状态
#define CMD_id_Match_result_data                                ((uint16_t)0x0002)          //!<@brief 比赛结果
#define CMD_id_Match_all_robot_HP_data                          ((uint16_t)0x0003)          //!<@brief 全体机器人血量
#define CMD_id_Site_event_data                                  ((uint16_t)0x0101)          //!<@brief 场地事件
#define CMD_id_Site_supply_station_action_identification_data   ((uint16_t)0x0102)          //!<@brief 补给站动作标识
#define CMD_id_Referee_warning_data                             ((uint16_t)0x0104)          //!<@brief 裁判警告
#define CMD_id_Darts_info_data                                  ((uint16_t)0x0105)          //!<@brief 飞镖发射相关
#define CMD_id_Robot_state_data                                 ((uint16_t)0x0201)          //!<@brief 本机器人状态
#define CMD_id_Real_time_chassis_power_and_shoot_heat_data      ((uint16_t)0x0202)          //!<@brief 实时底盘功率和枪口热量
#define CMD_id_Robot_position_data                              ((uint16_t)0x0203)          //!<@brief 本机器人位置
#define CMD_id_Robot_gain_data                                  ((uint16_t)0x0204)          //!<@brief 本机器人增益
#define CMD_id_Air_robot_status_data                            ((uint16_t)0x0205)          //!<@brief 空中机器人状态
#define CMD_id_Damage_status_data                               ((uint16_t)0x0206)          //!<@brief 扣血原因
#define CMD_id_Real_time_shooting_data                          ((uint16_t)0x0207)          //!<@brief 实时射击信息
#define CMD_id_Ammo_amount_data                                 ((uint16_t)0x0208)          //!<@brief 允许发弹量
#define CMD_id_Robot_RFID_state_data                            ((uint16_t)0x0209)          //!<@brief 机器人RFID状态
#define CMD_id_Darts_client_cmd_data                            ((uint16_t)0x020A)          //!<@brief 飞镖选手端指令
#define CMD_id_Robot_position_to_Sentry_data                    ((uint16_t)0x020B)          //!<@brief 本方地面机器人位置
#define CMD_id_Radar_marking_progress_data                      ((uint16_t)0x020C)          //!<@brief 雷达标记进度数据
#define CMD_id_Sentry_info_data                                 ((uint16_t)0x020D)          //!<@brief 哨兵自主决策信息同步
#define CMD_id_Radar_info_data                                  ((uint16_t)0x020E)          //!<@brief 雷达自主决策信息同步
/* 数据包长度 */
#define Len_Match_status_data                                    (9 +  11)                   //!<@brief 比赛状态
#define Len_Match_result_data                                    (9 +  1 )                   //!<@brief 比赛结果
#define Len_Match_all_robot_HP_data                              (9 +  32)                   //!<@brief 全体机器人血量
#define Len_Site_event_data                                      (9 +  4 )                   //!<@brief 场地事件
#define Len_Site_supply_station_action_identification_data       (9 +  4 )                   //!<@brief 补给站动作标识
#define Len_Referee_warning_data                                 (9 +  3 )                   //!<@brief 裁判警告
#define Len_Darts_info_data                                      (9 +  3 )                   //!<@brief 飞镖发射相关
#define Len_Robot_state_data                                     (9 +  13)                   //!<@brief 本机器人状态
#define Len_Real_time_chassis_power_and_shoot_heat_data          (9 +  16)                   //!<@brief 实时底盘功率和枪口热量
#define Len_Robot_position_data                                  (9 +  16)                   //!<@brief 本机器人位置
#define Len_Robot_gain_data                                      (9 +  6 )                   //!<@brief 本机器人增益
#define Len_Air_robot_status_data                                (9 +  2 )                   //!<@brief 空中机器人状态
#define Len_Damage_status_data                                   (9 +  1 )                   //!<@brief 扣血原因
#define Len_Real_time_shooting_data                              (9 +  7 )                   //!<@brief 实时射击信息
#define Len_Ammo_amount_data                                     (9 +  6 )                   //!<@brief 允许发弹量
#define Len_Robot_RFID_state_data                                (9 +  4 )                   //!<@brief 机器人RFID状态
#define Len_Darts_client_cmd_data                                (9 +  6 )                   //!<@brief 飞镖选手端指令
#define Len_Robot_position_to_Sentry_data                        (9 +  40)                   //!<@brief 本方地面机器人位置
#define Len_Radar_marking_progress_data                          (9 +  6 )                   //!<@brief 雷达标记进度数据
#define Len_Sentry_info_data                                     (9 +  4 )                   //!<@brief 哨兵自主决策信息同步
#define Len_Radar_info_data                                      (9 +  1 )                   //!<@brief 雷达自主决策信息同步

/* 缓冲区长度应为使能数据包最大长度 */
#if Robot_position_to_Sentry_data == 1
    #define REFEREE_BUFFER_LEN    Len_Robot_position_to_Sentry_data
    #define RM_QUEUE_LEN          6
#elif Match_all_robot_HP_data == 1
    #define REFEREE_BUFFER_LEN    Len_Match_all_robot_HP_data
    #define RM_QUEUE_LEN          8
#else
    #define REFEREE_BUFFER_LEN    25
    #define RM_QUEUE_LEN          10
#endif

#define REFEREE_BUFFER_LENx2  (REFEREE_BUFFER_LEN * 2)

/*
* 裁判系统信息结构体定义（1字节对齐）
*/
#pragma pack(1)
/* frame_header */
typedef struct{
    uint8_t SOF;            //!< @brief 数据帧起始字节，固定值为 0xA5
    uint16_t data_length;   //!< @brief 数据帧中 data 的长度
    uint8_t seq;            //!< @brief 包序号
    uint8_t CRC8;           //!< @brief 帧头 CRC8 校验
} PACK_HEAD_t;

/* 比赛状态：0x0001 发送频率：1Hz 发送范围：全体机器人 */
typedef struct{
    uint8_t game_type: 4;       //!< @brief 比赛类型 1:超级对抗赛 2:高校单项赛 3:人工智能挑战赛 4:高校联盟赛 3V3 对抗 5:高校联盟赛步兵对抗
    uint8_t game_progress: 4;   //!< @brief 当前比赛阶段 0：未开始比赛 1：准备阶段 2：十五秒裁判系统自检阶段 3：五秒倒计时 4：比赛中 5：比赛结算中
    uint16_t stage_remain_time; //!< @brief 当前阶段剩余时间，单位：秒
    uint64_t SyncTimeStamp;     //!< @brief UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效
} game_status_t;

/* 比赛结果：0x0002 比赛结束触发 发送范围：全体机器人 */
typedef struct{
    uint8_t winner;            //!< @brief 0：平局 1：红方胜利 2：蓝方胜利
} game_result_t;

/* 全体机器人血量：0x0003 发送频率：3Hz 发送范围：全体机器人 */
typedef struct{
    uint16_t red_1_robot_HP;    //!< @brief 红 1 英雄机器人血量。若该机器人未上场或者被罚下，则血量为 0
    uint16_t red_2_robot_HP;    //!< @brief 红 2 工程机器人血量
    uint16_t red_3_robot_HP;    //!< @brief 红 3 步兵机器人血量
    uint16_t red_4_robot_HP;    //!< @brief 红 4 步兵机器人血量
    uint16_t red_5_robot_HP;    //!< @brief 红 5 步兵机器人血量
    uint16_t red_7_robot_HP;    //!< @brief 红 7 哨兵机器人血量
    uint16_t red_outpost_HP;    //!< @brief 红方前哨站血量
    uint16_t red_base_HP;       //!< @brief 红方基地血量
    uint16_t blue_1_robot_HP;   //!< @brief 蓝 1 英雄机器人血量
    uint16_t blue_2_robot_HP;   //!< @brief 蓝 2 工程机器人血量
    uint16_t blue_3_robot_HP;   //!< @brief 蓝 3 步兵机器人血量
    uint16_t blue_4_robot_HP;   //!< @brief 蓝 4 步兵机器人血量
    uint16_t blue_5_robot_HP;   //!< @brief 蓝 5 步兵机器人血量
    uint16_t blue_7_robot_HP;   //!< @brief 蓝 7 哨兵机器人血量
    uint16_t blue_outpost_HP;   //!< @brief 蓝方前哨站血量
    uint16_t blue_base_HP;      //!< @brief 蓝方基地血量
} game_robot_HP_t;

/* 场地事件：0x0101 发送频率：1Hz 发送范围：己方全体机器人 */
typedef struct{ 
    uint32_t event_data;//!< @brief 场地事件
} event_data_t;

/* 补给站动作标识：0x0102 补给站弹丸释放时触发发送 发送范围：己方全体机器人 */
typedef struct{
    uint8_t reserved;                   //!< @brief 保留
    uint8_t supply_robot_id;            //!< @brief 补弹机器人ID  0：当前无机器人补弹 1：红方英雄机器人补弹 3/4/5：红方步兵机器人补弹 101：蓝方英雄机器人补弹 103/104/105：蓝方步兵机器人补弹
    uint8_t supply_projectile_step;     //!< @brief 出弹口开闭状态 0：关闭  1：弹丸准备中 2：弹丸释放
    uint8_t supply_projectile_num;      //!< @brief 补弹数量 50：50 颗弹丸 100：100 颗弹丸 150：150 颗弹丸 200：200 颗弹丸
} ext_supply_projectile_action_t;

/* 裁判警告：0x0104 己方判罚/判负时触发发送，其余时间以 1Hz 频率发送 发送范围：被判罚方全体机器人 */
typedef struct{
    uint8_t level;              //!< @brief 己方最后一次受到判罚的等级 1：双方黄牌2：黄牌 3：红牌 4：判负
    uint8_t offending_robot_id; //!< @brief 己方最后一次受到判罚的违规机器人ID（如红 1 机器人 ID 为 1，蓝1 机器人 ID 为 101）判负和双方黄牌时，该值为 0
    uint8_t count;              //!< @brief 己方最后一次受到判罚的违规机器人对应判罚等级的违规次数（开局默认为 0）
}referee_warning_t;          

/* 飞镖发射相关：0x0105 发送频率：1Hz 发送范围：己方全体机器人 */
typedef struct{
    uint8_t dart_remaining_time;         //!< @brief 己方飞镖发射剩余时间，单位：秒
    uint16_t dart_info; 
}dart_info_t;

/* 本机器人状态：0x0201 发送频率：10Hz 发送范围：本机器人 */
typedef struct{
    uint8_t robot_id;                             //!< @brief 本机器人 ID
    uint8_t robot_level;                          //!< @brief 机器人等级
    uint16_t current_HP;                          //!< @brief 机器人当前血量
    uint16_t maximum_HP;                          //!< @brief 机器人血量上限
    uint16_t shooter_barrel_cooling_value;        //!< @brief 机器人枪口热量每秒冷却值
    uint16_t shooter_barrel_heat_limit;           //!< @brief 机器人枪口热量上限
    uint16_t chassis_power_limit;                 //!< @brief 机器人底盘功率上限
    struct power_management_s{                    //!< @brief 电源管理模块的输出情况：
        uint8_t gimbal_output:1;                  //!< @briefgimbal  口输出：0 为无输出，1 为 24V 输出
        uint8_t chassis_output:1;                 //!< @brief  chassis 口输出：0 为无输出，1 为 24V 输出
        uint8_t shooter_output:1;                 //!< @brief  shooter 口输出：0 为无输出，1 为 24V 输出
    } power_management;
}robot_status_t;

/* 实时底盘功率和枪口热量：0x0202 发送频率：50Hz 发送范围：本机器人 */
typedef struct{
    uint16_t chassis_voltage;              //!< @brief 电源管理模块的 chassis 口输出电压（单位：mV）
    uint16_t chassis_current;              //!< @brief 电源管理模块的 chassis 口输出电流（单位：mA）
    float chassis_power;                   //!< @brief 底盘功率（单位：W）
    uint16_t buffer_energy;                //!< @brief 缓冲能量（单位：J）
    uint16_t shooter_17mm_1_barrel_heat;   //!< @brief 第 1 个 17mm 发射机构的枪口热量
    uint16_t shooter_17mm_2_barrel_heat;   //!< @brief 第 2 个 17mm 发射机构的枪口热量
    uint16_t shooter_42mm_barrel_heat;     //!< @brief 42mm 发射机构的枪口热量
}power_heat_data_t;
/* 本机器人位置：0x0203 发送频率：1Hz 发送范围：本机器人 */
typedef struct{
    float x;                //!< @brief 本机器人位置 x 坐标，单位：m
    float y;                //!< @brief 本机器人位置 y 坐标，单位：m
    float angle;            //!< @brief 本机器人测速模块的朝向，单位：度。正北为 0 度
}robot_pos_t;

/* 本机器人增益：0x0204 发送频率：3Hz 发送范围：本机器人 */
typedef struct{
    uint8_t recovery_buff;         //!< @brief 机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
    uint8_t cooling_buff;          //!< @brief 机器人枪口冷却倍率（直接值，值为 5 表示 5 倍冷却）
    uint8_t defence_buff;          //!< @brief 机器人防御增益（百分比，值为 50 表示 50%防御增益）
    uint8_t vulnerability_buff;    //!< @brief 机器人负防御增益（百分比，值为 30 表示-30%防御增益）
    uint16_t attack_buff;          //!< @brief 机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
}buff_t;

/* 空中机器人状态：0x0205 发送频率：1Hz 发送范围：己方空中机器人 */
typedef struct{
    uint8_t airforce_status;//!< @brief 空中机器人状态（0 为正在冷却，1 为冷却完毕，2 为正在空中支援）
    uint8_t time_remain;    //!< @brief 此状态的剩余时间（单位为：秒，向下取整，即冷却时间剩余 1.9 秒时，此值为 1）若冷却时间为 0，但未呼叫空中支援，则该值为 0
} air_support_data_t;

/* 扣血原因：0x0206，伤害发生后发送 发送范围：本机器人 */
typedef struct{
    uint8_t armor_id :4;            //!< @brief bit 0-3：当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，该 4 bit 组成的数值为装甲模块或测速模块的 ID 编号；当其他原因导致扣血时，该数值为 0
    uint8_t HP_deduction_reason :4; //!< @brief bit 4-7：血量变化  0：装甲模块被弹丸攻击 1：裁判系统重要模块离线 2：射击初速度超限 3：枪口热量超限 4：底盘功率超限 5：装甲模块受到撞击
} hurt_data_t;

/* 实时射击信息：0x0207 射击后发送 发送范围：本机器人 */
typedef struct{
    uint8_t bullet_type;               //!< @brief 弹丸类型：1：17mm 弹丸 2：42mm 弹丸
    uint8_t shooter_number;            //!< @brief 发射机构ID：1：第 1 个 17mm 发射机构  2：第 2 个 17mm 发射机构 3：42mm 发射机构
    uint8_t launching_frequency;       //!< @brief 弹丸射速（单位：Hz）
    float initial_speed;               //!< @brief 弹丸初速度（单位：m/s）
}shoot_data_t;

/* 允许发弹量：0x0208 发送频率：10Hz 发送范围：己方英雄、步兵、哨兵、空中机器人 */
typedef struct{
    uint16_t projectile_allowance_17mm;    //!< @brief 17mm 弹丸允许发弹量
    uint16_t projectile_allowance_42mm;    //!< @brief 42mm 弹丸允许发弹量
    uint16_t remaining_gold_coin;          //!< @brief 剩余金币数量
}projectile_allowance_t;

/* 机器人RFID状态：0x0209 发送频率：3Hz 发送范围：己方装有 RFID模块的机器人*/
typedef struct{
    uint32_t rfid_status;                   //!< @brief RFID状态
}rfid_status_t;
/* 飞镖选手端指令：0x020A 发送频率：3Hz 发送范围：己方飞镖机器人 */
typedef struct{
    uint8_t dart_launch_opening_status;    //!< @brief 当前飞镖发射站的状态：1：关闭 2：正在开启或者关闭中 0：已经开启
    uint8_t reserved;                      //!< @brief 保留位
    uint16_t target_change_time;           //!< @brief 切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为 0。
    uint16_t latest_launch_cmd_time;       //!< @brief 最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为 0。
}dart_client_cmd_t;

/* 本方地面机器人位置：0x020B 发送频率：1Hz 发送范围：己方哨兵机器人 */
typedef struct{
    float hero_x;                  //!< @brief 己方英雄机器人位置 x 轴坐标，单位：m
    float hero_y;                  //!< @brief 己方英雄机器人位置 y 轴坐标，单位：m
    float engineer_x;              //!< @brief 己方工程机器人位置 x 轴坐标，单位：m
    float engineer_y;              //!< @brief 己方工程机器人位置 y 轴坐标，单位：m
    float standard_3_x;            //!< @brief 己方 3 号步兵机器人位置 x 轴坐标，单位：m
    float standard_3_y;            //!< @brief 己方 3 号步兵机器人位置 y 轴坐标，单位：m
    float standard_4_x;            //!< @brief 己方 4 号步兵机器人位置 x 轴坐标，单位：m
    float standard_4_y;            //!< @brief 己方 4 号步兵机器人位置 y 轴坐标，单位：m
    float standard_5_x;            //!< @brief 己方 5 号步兵机器人位置 x 轴坐标，单位：m
    float standard_5_y;            //!< @brief 己方 5 号步兵机器人位置 y 轴坐标，单位：m
}ground_robot_position_t;

/* 雷达标记进度：0x020C 发送频率：1Hz 发送范围：己方雷达机器人*/
typedef struct{
    uint8_t mark_hero_progress;        //!< @brief 对方英雄机器人被标记进度：0-120
    uint8_t mark_engineer_progress;    //!< @brief 对方工程机器人被标记进度：0-120
    uint8_t mark_standard_3_progress;  //!< @brief 对方 3 号步兵机器人被标记进度：0-120
    uint8_t mark_standard_4_progress;  //!< @brief 对方 4 号步兵机器人被标记进度：0-120
    uint8_t mark_standard_5_progress;  //!< @brief 对方 5 号步兵机器人被标记进度：0-120
    uint8_t mark_sentry_progress;      //!< @brief 对方哨兵机器人被标记进度：0-120
}radar_mark_data_t;

/* 哨兵自主决策信息同步：0x020D 发送频率：1Hz 发送范围：己方哨兵机器人*/
typedef struct{
    uint32_t sentry_info;           //!< @brief 哨兵信息
} sentry_info_t;

/* 雷达自主决策信息同步：0x020E 发送频率：1Hz 发送范围：己方雷达机器人*/
typedef  struct{
    uint8_t radar_info;                 //!< @brief 雷达信息
} radar_info_t;

#pragma pack()

/**
 * 数据包解包
 * @param[in] *RxMsg  裁判系统的一个完整的数据包
 * @return 数据包长度
 * @retval  CRC错误 或 数据包超过缓存区的长度 则返回0
 */
uint8_t referee_system_Rx(uint8_t *RxMsg);
/**
 *  
 *
 */
uint8_t unpack_referee_system_data(RMQueue_Handle *Queue);

/* 声明外部变量 */
extern game_status_t                           Match_status;
extern game_result_t                           Match_result;
extern game_robot_HP_t                         Match_all_robot_HP;
extern event_data_t                            Site_event;
extern ext_supply_projectile_action_t          Site_supply_station_action_identification;
extern referee_warning_t                       Referee_warning;
extern dart_info_t                             Darts_info;
extern robot_status_t                          Robot_state;
extern power_heat_data_t                       Real_time_chassis_power_and_shoot_heat;
extern robot_pos_t                             Robot_position;
extern buff_t                                  Robot_gain;
extern air_support_data_t                      Air_robot_energy_status;
extern hurt_data_t                             Damage_status;
extern shoot_data_t                            Real_time_shooting;
extern projectile_allowance_t                  Ammo_amount;
extern rfid_status_t                           Robot_RFID_state;
extern dart_client_cmd_t                       Darts_client;
extern ground_robot_position_t                 Robot_position_to_Sentry;
extern radar_mark_data_t                       Radar_marking_progress;
extern sentry_info_t                           Sentry_info;
extern radar_info_t                            Radar_info;

#endif
