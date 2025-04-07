#include "motor_def.h"
#include "PID.h"

Controller_s* create_controller(controller_Init_config_t* _config)
{
    Controller_s* obj = (Controller_s *)malloc(sizeof(Controller_s));
    memset(obj, 0, sizeof(Controller_s));
	obj->motor_setting = _config->setting_config;
    return obj;
}
