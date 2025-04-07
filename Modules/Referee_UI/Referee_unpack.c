#include "Referee_unpack.h"

/* 缓冲区定义 */
uint8_t referee_system_buffer     [REFEREE_BUFFER_LENx2];

/* 裁判系统信息结构体定义 */
game_status_t                           Match_status;
game_result_t                           Match_result;
game_robot_HP_t                         Match_all_robot_HP;
event_data_t                            Site_event;
ext_supply_projectile_action_t          Site_supply_station_action_identification;
referee_warning_t                       Referee_warning;
dart_info_t                             Darts_info;
robot_status_t                          Robot_state;
power_heat_data_t                       Real_time_chassis_power_and_shoot_heat;
robot_pos_t                             Robot_position;
buff_t                                  Robot_gain;
air_support_data_t                      Air_robot_energy_status;
hurt_data_t                             Damage_status;
shoot_data_t                            Real_time_shooting;
projectile_allowance_t                  Ammo_amount;
rfid_status_t                           Robot_RFID_state;
dart_client_cmd_t                       Darts_client;
ground_robot_position_t                 Robot_position_to_Sentry;
radar_mark_data_t                       Radar_marking_progress;
sentry_info_t                           Sentry_info;
radar_info_t                            Radar_info;

/* 读取数据包 */
uint8_t unpack_referee_system_data(RMQueue_Handle *Queue) {
    uint8_t *referee_ptr = RMQueueTop(Queue);
    if (referee_ptr) {
        memcpy(referee_system_buffer, referee_system_buffer + REFEREE_BUFFER_LEN, REFEREE_BUFFER_LEN);
        memcpy(referee_system_buffer + REFEREE_BUFFER_LEN, referee_ptr, REFEREE_BUFFER_LEN);
        RMQueuePop(Queue);
        for (uint8_t i = 0; i < REFEREE_BUFFER_LEN; i++) {
            if (referee_system_buffer[i] == 0xA5)
                i += referee_system_Rx(&referee_system_buffer[i]);
        }
        return 1;
    }
    return 0;
}

/* 数据包解包 */
uint8_t referee_system_Rx(uint8_t *RxMsg) {
    PACK_HEAD_t *head = (PACK_HEAD_t *)RxMsg;
    uint16_t CMD_id   = *(uint16_t *)(RxMsg + 5);
    uint8_t *Msg      = &RxMsg[7];
    if (head->data_length > (REFEREE_BUFFER_LEN-9) || head->CRC8 != get_CRC8_check_sum(RxMsg, 4, 0xff))
        return 0;
    uint16_t Referee_CRC = *(uint16_t *)(RxMsg + 7 + head->data_length);
    if (Referee_CRC == get_CRC16_check_sum(RxMsg, 7 + head->data_length, 0xffff)) 
    {
        switch (CMD_id) 
        {
#if Match_status_data == REFEREE_SYSTEM_ENABLE
            case CMD_id_Match_status_data:{
                memcpy(&Match_status, Msg, sizeof(game_status_t));
            break;}
#endif

#if Match_result_data == REFEREE_SYSTEM_ENABLE
            case CMD_id_Match_result_data:
                memcpy(&Match_result, Msg, sizeof(game_result_t));
            break;
#endif

#if Match_all_robot_HP_data == REFEREE_SYSTEM_ENABLE
            case CMD_id_Match_all_robot_HP_data:
                memcpy(&Match_all_robot_HP, Msg, sizeof(game_robot_HP_t));
            break;
#endif

#if Site_event_data == REFEREE_SYSTEM_ENABLE
            case CMD_id_Site_event_data:
                memcpy(&Site_event, Msg, sizeof(event_data_t));
            break;
#endif

#if Site_supply_station_action_identification_data == REFEREE_SYSTEM_ENABLE
            case CMD_id_Site_supply_station_action_identification_data:
                memcpy(&Site_supply_station_action_identification, Msg, sizeof(ext_supply_projectile_action_t));
            break;
#endif

#if Referee_warning_data == REFEREE_SYSTEM_ENABLE
            case CMD_id_Referee_warning_data:
                memcpy(&Referee_warning, Msg, sizeof(referee_warning_t));
            break;
#endif
        
#if Darts_info_data == REFEREE_SYSTEM_ENABLE
            case CMD_id_Darts_info_data:
                memcpy(&Darts_info, Msg, sizeof(dart_info_t));
            break;
#endif
            
#if Robot_state_data == REFEREE_SYSTEM_ENABLE
            case CMD_id_Robot_state_data:
                memcpy(&Robot_state, Msg,  sizeof(robot_status_t));
            break;
#endif

#if Real_time_chassis_power_and_shoot_heat_data == REFEREE_SYSTEM_ENABLE
            case CMD_id_Real_time_chassis_power_and_shoot_heat_data:
                memcpy(&Real_time_chassis_power_and_shoot_heat, Msg, sizeof(power_heat_data_t));
            break;
#endif

#if Robot_position_data == REFEREE_SYSTEM_ENABLE
            case CMD_id_Robot_position_data:
                memcpy(&Robot_position, Msg, sizeof(robot_pos_t));
            break;
#endif

#if Robot_gain_data == REFEREE_SYSTEM_ENABLE
            case CMD_id_Robot_gain_data:
                memcpy(&Robot_gain, Msg, sizeof(buff_t));
            break;
#endif

#if Air_robot_energy_status_data == REFEREE_SYSTEM_ENABLE
            case CMD_id_Air_robot_status_data:
                memcpy(&Air_robot_energy_status, Msg, sizeof(air_support_data_t));
            break;
#endif

#if Damage_status_data == REFEREE_SYSTEM_ENABLE
            case CMD_id_Damage_status_data:
                memcpy(&Damage_status, Msg, sizeof(hurt_data_t));
            break;
#endif

#if Real_time_shooting_data== REFEREE_SYSTEM_ENABLE
            case CMD_id_Real_time_shooting_data:
                memcpy(&Real_time_shooting, Msg, sizeof(shoot_data_t));
            break;
#endif
   
#if Ammo_remind_data==REFEREE_SYSTEM_ENABLE
            case CMD_id_Ammo_amount_data:
                memcpy(&Ammo_amount, Msg, sizeof(projectile_allowance_t));
            break;
#endif

#if Robot_RFID_state_data == REFEREE_SYSTEM_ENABLE
            case CMD_id_Robot_RFID_state_data:
                memcpy(&Robot_RFID_state, Msg, sizeof(rfid_status_t));
            break;
#endif

#if Darts_client_cmd == REFEREE_SYSTEM_ENABLE
            case CMD_id_Darts_client_cmd_data:
                memcpy(&Darts_client, Msg, sizeof(dart_client_cmd_t));
            break;
#endif

#if Robot_position_to_Sentry_data == REFEREE_SYSTEM_ENABLE
            case CMD_id_Robot_position_to_Sentry_data:
                memcpy(&Robot_position_to_Sentry, Msg, sizeof(ground_robot_position_t));
            break;
#endif

#if Radar_marking_progress_data == REFEREE_SYSTEM_ENABLE
            case CMD_id_Radar_marking_progress_data:
                memcpy(&Radar_marking_progress, Msg, sizeof(radar_mark_data_t));
            break;
#endif

#if Sentry_info_data == REFEREE_SYSTEM_ENABLE
            case CMD_id_Sentry_info_data:
                memcpy(&Sentry_info, Msg, sizeof(sentry_info_t));
            break;
#endif

#if Radar_info_data == REFEREE_SYSTEM_ENABLE
            case CMD_id_Radar_info_data:
                memcpy(&Radar_info, Msg, sizeof(radar_info_t));
            break;
#endif
        }
        return  9 + head->data_length;
    }
    else 
        return 0;
}
