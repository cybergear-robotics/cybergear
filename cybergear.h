#ifndef CYBERGEAR_H
#define CYBERGEAR_H

#include <unistd.h>
#include <stdbool.h>

#include "esp_err.h"
#include "driver/twai.h"

#include "cybergear_defs.h"

typedef struct
{
    float position; /* 4π to 4π */
    float speed; /* -30 rad/s to 30 rad/s */
    float torque; /* -12Nm to 12Nm */
    float temperature; /* 0..200 °C */
    cybergear_state_e state;
} cybergear_status_t;

typedef struct
{
    float position;
    float speed;
    float torque;
    float kp;
    float kd;
} cybergear_motion_cmd_t;

typedef struct
{
    bool overload;
    bool uncalibrated;
    bool over_current_phase_a;
    bool over_current_phase_b;
    bool over_current_phase_c;    
    bool over_voltage;
    bool under_voltage;
    bool driver_chip;
    bool over_temperature;
    bool magnetic_code_failure;
    bool hall_coded_faults;
} cybergear_fault_t;

typedef struct
{
  uint16_t run_mode;
  float iq_ref;
  float spd_ref;
  float limit_torque;
  float cur_kp;
  float cur_ki;
  float cur_filt_gain;
  float loc_ref;
  float limit_spd;
  float limit_cur;
  float mech_pos;
  float iqf;
  float mech_vel;
  float vbus;
  int16_t rotation;
  float loc_kp;
  float spd_kp;
  float spd_ki;
  bool updated; /* indicator if the struct is updated*/
} cybergear_params_t;

typedef struct
{
    uint8_t master_can_id;
    uint8_t can_id;
    TickType_t transmit_ticks_to_wait;
    cybergear_params_t params;    
    cybergear_status_t status;
    union {
        cybergear_fault_t fault_bits;
        uint16_t fault_bitmask;
    } faults;
    
} cybergear_motor_t;

esp_err_t cybergear_init(cybergear_motor_t *motor, uint8_t master_can_id, uint8_t can_id, TickType_t transmit_ticks_to_wait);

esp_err_t cybergear_enable(cybergear_motor_t *motor);
esp_err_t cybergear_stop(cybergear_motor_t *motor);
esp_err_t cybergear_stop(cybergear_motor_t *motor);
esp_err_t cybergear_set_mode(cybergear_motor_t *motor, cybergear_mode_e mode);

esp_err_t cybergear_get_param(cybergear_motor_t *motor, uint16_t index);

esp_err_t cybergear_set_motor_can_id(cybergear_motor_t *motor, uint8_t can_id);
esp_err_t cybergear_set_mech_position_to_zero(cybergear_motor_t *motor);

esp_err_t cybergear_request_status(cybergear_motor_t *motor);
esp_err_t cybergear_process_message(cybergear_motor_t *motor, twai_message_t *message);

esp_err_t cybergear_set_limit_speed(cybergear_motor_t *motor, float speed);
esp_err_t cybergear_set_limit_current(cybergear_motor_t *motor, float current);
esp_err_t cybergear_set_limit_torque(cybergear_motor_t *motor, float torque);

/* motion mode */
esp_err_t cybergear_set_motion_cmd(cybergear_motor_t *motor, cybergear_motion_cmd_t *cmd);

/* current mode */
esp_err_t cybergear_set_current_kp(cybergear_motor_t *motor, float kp);
esp_err_t cybergear_set_current_ki(cybergear_motor_t *motor, float ki);
esp_err_t cybergear_set_current_filter_gain(cybergear_motor_t *motor, float gain);
esp_err_t cybergear_set_current(cybergear_motor_t *motor, float current);

/* position mode */
esp_err_t cybergear_set_position_kp(cybergear_motor_t *motor, float kp);
esp_err_t cybergear_set_position(cybergear_motor_t *motor, float position);

/* speed mode */
esp_err_t cybergear_set_speed_kp(cybergear_motor_t *motor, float kp);
esp_err_t cybergear_set_speed_ki(cybergear_motor_t *motor, float ki);
esp_err_t cybergear_set_speed(cybergear_motor_t *motor, float speed);

void cybergear_get_status(cybergear_motor_t *motor, cybergear_status_t *status);
void cybergear_get_faults(cybergear_motor_t *motor, cybergear_fault_t *faults);
bool cybergear_has_faults(cybergear_motor_t *motor);

#endif
