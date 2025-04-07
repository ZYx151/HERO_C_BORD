#include "servo_motor.h"

static uint8_t servo_angle_read[6]={0x55 ,0x55 ,0x04, 0x15 ,0x01 ,0x01 ,};
static uint8_t servo_angle_write[16]={0x55 ,0x55, 0x08, 0x03, 0x01 ,0xF4 ,0x01 ,0x01 ,0x20 ,0x03,0x55 ,0x55 ,0x04, 0x15 ,0x01 ,0x01 ,};
static uint8_t servo_unload[6]={0x55,0x55,0x04,0x14,0x01,0x01};

static ServoInstance *servo_motor_instance[SERVO_MOTOR_CNT];
static uint8_t servo_idx = 0; // register servo_idx,是该文件的全局舵机索引,在注册时使用
static void DecodeServo(uint8_t *buff);

// 通过此函数注册一个舵机
ServoInstance *ServoInit(Servo_Init_Config_s *Servo_Init_Config)
{
    ServoInstance *servo = (ServoInstance *)malloc(sizeof(ServoInstance));
    memset(servo, 0, sizeof(ServoInstance));
    USART_Init_Config_s config;
    servo->servo_type = Servo_Init_Config->servo_type;
    switch (Servo_Init_Config->servo_type)
    {
    case Bus_Servo:
        config.module_callback = DecodeServo;
        config.data_len = Servo_MAX_BUFF;
        config.usart_handle = Servo_Init_Config->_handle;
        servo->usart_instance = USARTRegister(&config);
        break;
    case PWM_Servo:
        servo->pwm_instance = PWMRegister(&Servo_Init_Config->pwm_init_config);
        break;
    default:
		return 0;
        break;
    }
    servo_motor_instance[servo_idx++] = servo;

    return servo;
}
//@todo PWM舵机的角度设置需要根据相应定时器PWM等参数进行计算(是否需要规范定时器PWM的初始化参数，以便于计算)
void ServoSetAngle(ServoInstance *servo, float angle)
{

    switch (servo->servo_type)
    {
    case Bus_Servo:
        servo_angle_write[8] = (uint16_t)angle&0xff;
        servo_angle_write[9] = (uint16_t)angle>>8;
	    USART_Transmit(servo->usart_instance, servo_angle_write, 16, USART_TRANSFER_DMA);
       // USARTSend(servo->usart_instance, servo_angle_read, 6, USART_TRANSFER_DMA);
        break;
    case PWM_Servo:
        servo->angle = angle;
        PWMSetDutyRatio(servo->pwm_instance, angle);
        break;
    default:
        break;
    }
}

//@todo 只读取了角度 还有电压，动作是否完成等 且只支持一个串口
static void DecodeServo(uint8_t *buff)
{
    for (uint8_t i = 0; i < servo_idx; i++)
    {
        if (servo_motor_instance[i]->servo_type == Bus_Servo)
        {
            if (buff[0] == Servo_Frame_First && buff[1] == Servo_Frame_Second)
            {
                if (buff[3] == 21)
                {
                    servo_motor_instance[i]->recv_angle = (buff[7] << 8 | buff[6]);
                }
            }
        }
    }
}
