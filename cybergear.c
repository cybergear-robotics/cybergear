#include <string.h>
#include "driver/twai.h"
#include "cybergear.h"

esp_err_t _send_can_package(cybergear_motor_t *motor, uint8_t cmd_id, uint8_t len, uint8_t* data);
esp_err_t _send_can_option_package(cybergear_motor_t *motor, uint8_t cmd_id, uint8_t option, uint8_t len, uint8_t* data);
esp_err_t _send_can_float_package(cybergear_motor_t *motor, uint16_t addr, float value, float min, float max);
uint16_t _float_to_uint(float x, float x_min, float x_max, int bits);
float _uint_to_float(uint16_t x, float x_min, float x_max);
esp_err_t _process_motor_message(cybergear_motor_t *motor, twai_message_t *message);
esp_err_t _process_fault_message(cybergear_motor_t *motor, twai_message_t *message);
esp_err_t _process_param_message(cybergear_motor_t *motor, twai_message_t *message);

esp_err_t cybergear_init(cybergear_motor_t *motor, uint8_t master_can_id, uint8_t can_id, TickType_t transmit_ticks_to_wait) {
    motor->master_can_id = master_can_id;
    motor->can_id = can_id;
    motor->transmit_ticks_to_wait = transmit_ticks_to_wait;
    motor->faults.fault_bitmask = 0; /* reset faults */
    return ESP_OK;
}

esp_err_t cybergear_enable(cybergear_motor_t *motor)
{
    uint8_t data[8] = {0x00};
    return _send_can_package(motor, CMD_ENABLE, 8, data);
}

esp_err_t cybergear_stop(cybergear_motor_t *motor)
{
    uint8_t data[8] = {0x00};
    return _send_can_package(motor, CMD_RESET, 8, data);
}

esp_err_t cybergear_set_mode(cybergear_motor_t *motor, cybergear_mode_e mode)
{
    uint8_t data[8] = {0x00};
    data[0] = ADDR_RUN_MODE & 0x00FF;
    data[1] = ADDR_RUN_MODE >> 8;
    data[4] = mode;
    return _send_can_package(motor, CMD_RAM_WRITE, 8, data);
}

esp_err_t cybergear_get_param(cybergear_motor_t *motor, uint16_t index)
{
    motor->params.updated = false;
    uint8_t data[8] = {0x00};
    memcpy(&data[0], &index, 2);
    return _send_can_package(motor, CMD_RAM_READ, 8, data);
}

esp_err_t cybergear_set_motor_can_id(cybergear_motor_t *motor, uint8_t can_id)
{
    uint8_t data[8] = {0x00};
    uint16_t option = can_id << 8 | motor->master_can_id;
    esp_err_t err = _send_can_option_package(motor, CMD_SET_CAN_ID, option, 8, data);
    if(err == ESP_OK)
    {
        motor->can_id = can_id;
    }
    return err;    
}

esp_err_t cybergear_set_mech_position_to_zero(cybergear_motor_t *motor)
{
    uint8_t data[8] = {0x00};
    data[0] = 0x01;
    return _send_can_package(motor, CMD_SET_MECH_POSITION_TO_ZERO, 8, data);
}

esp_err_t cybergear_request_status(cybergear_motor_t *motor)
{
    uint8_t data[8] = {0x00};
    return _send_can_package(motor, CMD_GET_STATUS, 8, data);
}

esp_err_t cybergear_process_message(cybergear_motor_t *motor, twai_message_t *message)
{
    uint8_t can_id = (message->identifier & 0xFF00) >> 8;
    uint8_t packet_type = (message->identifier & 0x3F000000) >> 24;
    if(can_id != motor->can_id)
    {
        return ESP_ERR_NOT_FOUND;
    }

    switch(packet_type)
    {
        case CMD_REQUEST:
            return _process_motor_message(motor, message);
        case CMD_RAM_READ:
            return _process_param_message(motor, message);
        case CMD_GET_MOTOR_FAIL:
            return _process_fault_message(motor, message);
        default:
            return ESP_ERR_INVALID_RESPONSE;
    }
}

esp_err_t cybergear_set_limit_speed(cybergear_motor_t *motor, float speed)
{
    return _send_can_float_package(motor, ADDR_LIMIT_SPEED, speed, 0.0f, V_MAX);
}

esp_err_t cybergear_set_limit_current(cybergear_motor_t *motor, float current)
{
    return _send_can_float_package(motor, ADDR_LIMIT_CURRENT, current, 0.0f, I_MAX);
}

esp_err_t cybergear_set_limit_torque(cybergear_motor_t *motor, float torque)
{
    return _send_can_float_package(motor, ADDR_LIMIT_TORQUE, torque, 0.0f, T_MAX);
}

esp_err_t cybergear_set_motion_cmd(cybergear_motor_t *motor, cybergear_motion_cmd_t *cmd)
{
    uint8_t data[8] = {0x00};

    uint16_t position = _float_to_uint(cmd->position, POS_MIN, POS_MAX, 16);
    data[0] = position >> 8;
    data[1] = position & 0x00FF;

    uint16_t speed = _float_to_uint(cmd->speed, V_MIN, V_MAX, 16);
    data[2] = speed >> 8;
    data[3] = speed & 0x00FF;

    uint16_t kp = _float_to_uint(cmd->kp, KP_MIN, KP_MAX, 16);
    data[4] = kp >> 8;
    data[5] = kp & 0x00FF;

    uint16_t kd = _float_to_uint(cmd->kd, KD_MIN, KD_MAX, 16);
    data[6] = kd >> 8;
    data[7] = kd & 0x00FF;

    uint16_t torque = _float_to_uint(cmd->torque, T_MIN, T_MAX, 16);

    return _send_can_option_package(motor, CMD_POSITION, torque, 8, data);
}

esp_err_t cybergear_set_current_kp(cybergear_motor_t *motor, float kp)
{
    return _send_can_float_package(motor, ADDR_CURRENT_KP, kp, KP_MIN, KP_MAX);
}

esp_err_t cybergear_set_current_ki(cybergear_motor_t *motor, float ki)
{
    return _send_can_float_package(motor, ADDR_CURRENT_KI, ki, KI_MIN, KI_MAX);
}

esp_err_t cybergear_set_current_filter_gain(cybergear_motor_t *motor, float gain)
{
    return _send_can_float_package(motor, ADDR_CURRENT_FILTER_GAIN, gain, CURRENT_FILTER_GAIN_MIN, CURRENT_FILTER_GAIN_MAX);
}

esp_err_t cybergear_set_current(cybergear_motor_t *motor, float current)
{
    return _send_can_float_package(motor, ADDR_I_REF, current, I_MIN, I_MAX);
}

esp_err_t cybergear_set_position_kp(cybergear_motor_t *motor, float kp)
{
    return _send_can_float_package(motor, ADDR_POSITION_KP, kp, KP_MIN, KP_MAX);
}

esp_err_t cybergear_set_position(cybergear_motor_t *motor, float position)
{
    return _send_can_float_package(motor, ADDR_POSITION_REF, position, POS_MIN, POS_MAX);
}

esp_err_t cybergear_set_speed_kp(cybergear_motor_t *motor, float kp)
{
    return _send_can_float_package(motor, ADDR_SPEED_KP, kp, KP_MIN, KP_MAX);
}

esp_err_t cybergear_set_speed_ki(cybergear_motor_t *motor, float ki)
{
    return _send_can_float_package(motor, ADDR_SPEED_KI, ki, KI_MIN, KI_MAX);
}

esp_err_t cybergear_set_speed(cybergear_motor_t *motor, float speed)
{
    return _send_can_float_package(motor, ADDR_SPEED_REF, speed, V_MIN, V_MAX);
}

void cybergear_get_status(cybergear_motor_t *motor, cybergear_status_t *status)
{
    memcpy(status, &motor->status, sizeof(cybergear_status_t));
}

void cybergear_get_faults(cybergear_motor_t *motor, cybergear_fault_t *faults)
{
    memcpy(faults, &motor->faults.fault_bits, sizeof(cybergear_fault_t));
}

bool cybergear_has_faults(cybergear_motor_t *motor)
{
    return motor->faults.fault_bitmask > 0;
}

esp_err_t _send_can_package(cybergear_motor_t *motor, uint8_t cmd_id, uint8_t len, uint8_t* data)
{
    return _send_can_option_package(motor, cmd_id, motor->master_can_id, len, data);
}

esp_err_t _send_can_option_package(cybergear_motor_t *motor, uint8_t cmd_id, uint8_t option, uint8_t len, uint8_t* data)
{
    uint32_t id = cmd_id << 24 | option << 8 | motor->can_id;
    
    twai_message_t message;
    message.extd = id;
    message.identifier = id;
    message.data_length_code = len;
    for (int i = 0; i < len; i++) {
        message.data[i] = data[i];
    }
    return twai_transmit(&message, motor->transmit_ticks_to_wait);
}

esp_err_t _send_can_float_package(cybergear_motor_t *motor, uint16_t addr, float value, float min, float max)
{
    uint8_t data[8] = {0x00};
    data[0] = addr & 0x00FF;
    data[1] = addr >> 8;

    float val = (min > value) ? min : value;
    memcpy(&data[4], &value, 4);
    return _send_can_package(motor, CMD_RAM_WRITE, 8, data);
}

uint16_t _float_to_uint(float x, float x_min, float x_max, int bits)
{
    if (bits>16) bits=16;
    float span = x_max - x_min;
    float offset = x_min;
    if(x > x_max) x = x_max;
    else if(x < x_min) x = x_min;
    return (int) ((x-offset)*((float)((1<<bits)-1))/span);
}

float _uint_to_float(uint16_t x, float x_min, float x_max)
{
    uint16_t type_max = 0xFFFF;
    float span = x_max - x_min;
    return (float) x / type_max * span + x_min;
}

esp_err_t _process_motor_message(cybergear_motor_t *motor, twai_message_t *message)
{
    esp_err_t err = ESP_OK;
    uint16_t raw_position = message->data[1] | message->data[0] << 8;
    uint16_t raw_speed = message->data[3] | message->data[2] << 8;
    uint16_t raw_torque = message->data[5] | message->data[4] << 8;
    uint16_t raw_temperature = message->data[7] | message->data[6] << 8;
    uint16_t run_mode = (message->identifier & 0xC00000) >> 22;
    switch(run_mode) {
        case 0x0:
            motor->status.state = CYBERGEAR_STATE_RESET;
            break;
        case 0x1:
            motor->status.state = CYBERGEAR_STATE_CALIBRATION;
            break;
        case 0x2:
            motor->status.state = CYBERGEAR_STATE_RUNNING;
            break;
        default: /* unknown run state */
            err = ESP_ERR_INVALID_RESPONSE;
            break;
    }
    motor->status.position = _uint_to_float(raw_position, POS_MIN, POS_MAX);
    motor->status.speed = _uint_to_float(raw_speed, V_MIN, V_MAX);
    motor->status.torque = _uint_to_float(raw_torque, T_MIN, T_MAX);
    motor->status.temperature = ((float) raw_temperature)/10;
    motor->faults.fault_bits.under_voltage = message->identifier & (1 << 16);
    motor->faults.fault_bits.overload = message->identifier & (1 << 17);
    motor->faults.fault_bits.over_temperature = message->identifier & (1 << 18);
    motor->faults.fault_bits.magnetic_code_failure = message->identifier & (1 << 19);
    motor->faults.fault_bits.hall_coded_faults = message->identifier & (1 << 20);
    motor->faults.fault_bits.uncalibrated = message->identifier & (1 << 21);
    return err;
}

esp_err_t _process_fault_message(cybergear_motor_t *motor, twai_message_t *message)
{   
    uint32_t fault = message->data[0] << 24 | 
                     message->data[1] << 16 | 
                     message->data[2] << 8  | 
                     message->data[3];
    uint32_t warning = message->data[4] << 24 | 
                       message->data[5] << 16 | 
                       message->data[6] << 8  | 
                       message->data[7];
    motor->faults.fault_bits.over_current_phase_a = fault & (1 << 16);
    //motor->fault.overload = 0; // TODO: fault[8:15]
    motor->faults.fault_bits.uncalibrated = fault & (1 << 7);    
    motor->faults.fault_bits.over_current_phase_c = fault & (1 << 5);
    motor->faults.fault_bits.over_current_phase_b = fault & (1 << 4);
    motor->faults.fault_bits.over_voltage = fault & (1 << 3);
    motor->faults.fault_bits.under_voltage = fault & (1 << 2);
    motor->faults.fault_bits.driver_chip = fault & (1 << 1);
        motor->faults.fault_bits.over_temperature = warning & (1 << 0);
    return ESP_OK;
}

esp_err_t _process_param_message(cybergear_motor_t *motor, twai_message_t *message)
{
    uint16_t index = message->data[1] << 8 | message->data[0];

    uint8_t uint8_data;
    memcpy(&uint8_data, &message->data[4], sizeof(uint8_t));

    int16_t int16_data;
    memcpy(&int16_data, &message->data[4], sizeof(int16_t));

    float float_data;
    memcpy(&float_data, &message->data[4], sizeof(float));

    switch (index)
    {
        case ADDR_RUN_MODE:
            motor->params.run_mode = uint8_data;
            break;
        case ADDR_IQ_REF:
            motor->params.iq_ref = float_data;
            break;
        case ADDR_SPEED_REF:
            motor->params.spd_ref = float_data;
            break;
        case ADDR_LIMIT_TORQUE:
            motor->params.limit_torque = float_data;
            break;
        case ADDR_CURRENT_KP:
            motor->params.cur_kp = float_data;
            break;
        case ADDR_CURRENT_KI:
            motor->params.cur_ki = float_data;
            break;
        case ADDR_CURRENT_FILTER_GAIN:
            motor->params.cur_filt_gain = float_data;
            break;
        case ADDR_LOC_REF:
            motor->params.loc_ref = float_data;
            break;
        case ADDR_LIMIT_SPEED:
            motor->params.limit_spd = float_data;
            break;
        case ADDR_LIMIT_CURRENT:
            motor->params.limit_cur = float_data;
            break;
        case ADDR_MECH_POS:
            motor->params.mech_pos = float_data;
            break;
        case ADDR_IQF:
            motor->params.iqf = float_data;
            break;
        case ADDR_MECH_VEL:
            motor->params.mech_vel = float_data;
            break;
        case ADDR_VBUS:
            motor->params.vbus = float_data;
            break;
        case ADDR_ROTATION:
            motor->params.rotation = int16_data;
            break;
        case ADDR_LOC_KP:
            motor->params.loc_kp = float_data;
            break;
        case ADDR_SPD_KP:
            motor->params.spd_kp = float_data;
            break;
        case ADDR_SPD_KI:
            motor->params.spd_ki = float_data;
            break;
        default:
            return ESP_ERR_INVALID_RESPONSE;
        break;
    }
    motor->params.updated = true;
    return ESP_OK;
}

