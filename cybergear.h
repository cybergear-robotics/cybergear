#ifndef CYBERGEAR_H
#define CYBERGEAR_H

#include <unistd.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

#include "cybergear_defs.h"

/** @brief Latest motor feedback received from the CAN bus. */
typedef struct
{
    float position; /* 4π to 4π */
    float speed; /* -30 rad/s to 30 rad/s */
    float torque; /* -12Nm to 12Nm */
    float temperature; /* 0..200 °C */
    cybergear_state_e state;
} cybergear_status_t;

/** @brief Target values encoded in a motion-mode command. */
typedef struct
{
    float position;
    float speed;
    float torque;
    float kp;
    float kd;
} cybergear_motion_cmd_t;

/** @brief Fault flags reported by the motor. */
typedef struct
{
    uint16_t overload : 1;
    uint16_t uncalibrated : 1;
    uint16_t over_current_phase_a : 1;
    uint16_t over_current_phase_b : 1;
    uint16_t over_current_phase_c : 1;
    uint16_t over_voltage : 1;
    uint16_t under_voltage : 1;
    uint16_t driver_chip : 1;
    uint16_t over_temperature : 1;
    uint16_t magnetic_code_failure : 1;
    uint16_t hall_coded_faults : 1;
    uint16_t reserved : 5;
} cybergear_fault_t;

_Static_assert(sizeof(cybergear_fault_t) == sizeof(uint16_t), "CyberGear faults must fit the bitmask");

/** @brief Warning flags reported by the motor. */
typedef struct
{
    uint16_t over_temperature : 1;
    uint16_t reserved : 15;
} cybergear_warning_t;

_Static_assert(sizeof(cybergear_warning_t) == sizeof(uint16_t), "CyberGear warnings must fit the bitmask");

/** @brief Values returned by RAM parameter-read responses. */
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

/** @brief Device identity returned in a type-0 CAN response. */
typedef struct
{
    uint8_t motor_can_id;
    uint8_t recipient_can_id;
    uint8_t unique_id[8];
    bool updated;
} cybergear_device_info_t;

/** @brief One received extended CAN frame for the driver to process. */
typedef struct
{
    uint32_t identifier;
    const uint8_t *data;
    size_t data_length;
} cybergear_message_t;

/**
 * @brief Transmits one extended CAN frame.
 * @param context User context from cybergear_config_t::send_context.
 * @param identifier Extended CAN identifier.
 * @param data Frame payload that must be copied or sent before returning.
 * @param data_length Frame payload length in bytes.
 * @return ESP_OK on success.
 * @return Other error returned by the transport implementation.
 */
typedef esp_err_t (*cybergear_send_fn_t)(void *context, uint32_t identifier, const uint8_t *data, size_t data_length);

/** @brief Driver configuration supplied during initialization. */
typedef struct
{
    cybergear_send_fn_t send;
    void *send_context;
    cybergear_mode_e mode;
    uint8_t master_can_id;
    uint8_t can_id;
    float speed_limit;
    float current_limit;
    float torque_limit;
    bool enable_on_init;
} cybergear_config_t;

/** @brief Driver state for one CyberGear motor instance. */
typedef struct
{
    cybergear_config_t *config;
    cybergear_params_t params;    
    cybergear_status_t status;
    cybergear_device_info_t device_info;
    union {
        cybergear_fault_t fault_bits;
        uint16_t fault_bitmask;
    } faults;
    union {
        cybergear_warning_t warning_bits;
        uint16_t warning_bitmask;
    } warnings;
} cybergear_motor_t;


/**
 * @brief Initializes a motor instance and applies configured limits.
 * @param motor Motor instance to initialize.
 * @param config Transport and motor configuration that must remain valid while used.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or config is null.
 * @return ESP_ERR_INVALID_STATE if config->send is null.
 * @return Other error returned by config->send.
 */
esp_err_t cybergear_init(cybergear_motor_t *motor, cybergear_config_t *config);

/**
 * @brief Enables motor output.
 * @param motor Motor instance to enable.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
  * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_enable(cybergear_motor_t *motor);
/**
 * @brief Stops the motor without clearing active motor faults.
 * @param motor Motor instance to stop.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_stop(cybergear_motor_t *motor);
/**
 * @brief Stops the motor and clears active motor faults.
 * @param motor Motor instance to stop.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_stop_and_clear_faults(cybergear_motor_t *motor);
/**
 * @brief Sets the motor run mode.
 * @param motor Motor instance to configure.
 * @param mode Requested motor control mode.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_set_mode(cybergear_motor_t *motor, cybergear_mode_e mode);

/**
 * @brief Requests a RAM parameter by address.
 * @param motor Motor instance to query.
 * @param index RAM parameter address.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_get_param(cybergear_motor_t *motor, uint16_t index);

/**
 * @brief Changes the motor CAN ID.
 * @param motor Motor instance to readdress.
 * @param can_id New motor CAN ID.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_set_motor_can_id(cybergear_motor_t *motor, uint8_t can_id);
/**
 * @brief Sets the current mechanical position as zero.
 * @param motor Motor instance to calibrate.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_set_mech_position_to_zero(cybergear_motor_t *motor);

/**
 * @brief Requests a status feedback frame.
 * @param motor Motor instance to query.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_request_status(cybergear_motor_t *motor);
/**
 * @brief Decodes a received eight-byte extended CAN frame.
 * @param motor Motor instance to update.
 * @param message Received CAN frame.
 * @return ESP_OK if the frame was decoded.
 * @return ESP_ERR_INVALID_ARG if motor, motor->config, message, or message->data is null.
 * @return ESP_ERR_INVALID_SIZE if the frame length is not eight bytes.
 * @return ESP_ERR_NOT_FOUND if the frame motor ID does not match motor->config->can_id.
 * @return ESP_ERR_INVALID_RESPONSE if the frame contains an unsupported packet or value.
 */
esp_err_t cybergear_process_message(cybergear_motor_t *motor, const cybergear_message_t *message);

/**
 * @brief Sets the maximum speed in rad/s.
 * @param motor Motor instance to configure.
 * @param speed Maximum speed in rad/s.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_set_limit_speed(cybergear_motor_t *motor, float speed);
/**
 * @brief Sets the maximum current in A.
 * @param motor Motor instance to configure.
 * @param current Maximum current in A.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_set_limit_current(cybergear_motor_t *motor, float current);
/**
 * @brief Sets the maximum torque in Nm.
 * @param motor Motor instance to configure.
 * @param torque Maximum torque in Nm.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_set_limit_torque(cybergear_motor_t *motor, float torque);

/**
 * @brief Sends a motion-mode command.
 * @param motor Motor instance to command.
 * @param cmd Motion targets and gains.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor, motor->config, or cmd is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_set_motion_cmd(cybergear_motor_t *motor, cybergear_motion_cmd_t *cmd);

/**
 * @brief Sets the current-controller proportional gain.
 * @param motor Motor instance to configure.
 * @param kp Proportional gain.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_set_current_kp(cybergear_motor_t *motor, float kp);
/**
 * @brief Sets the current-controller integral gain.
 * @param motor Motor instance to configure.
 * @param ki Integral gain.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_set_current_ki(cybergear_motor_t *motor, float ki);
/**
 * @brief Sets the current feedback filter gain.
 * @param motor Motor instance to configure.
 * @param gain Filter gain.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_set_current_filter_gain(cybergear_motor_t *motor, float gain);
/**
 * @brief Sets the current target in A.
 * @param motor Motor instance to command.
 * @param current Current target in A.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_set_current(cybergear_motor_t *motor, float current);

/**
 * @brief Sets the position-controller proportional gain.
 * @param motor Motor instance to configure.
 * @param kp Proportional gain.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_set_position_kp(cybergear_motor_t *motor, float kp);
/**
 * @brief Sets the position target in rad.
 * @param motor Motor instance to command.
 * @param position Position target in rad.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_set_position(cybergear_motor_t *motor, float position);

/**
 * @brief Sets the speed-controller proportional gain.
 * @param motor Motor instance to configure.
 * @param kp Proportional gain.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_set_speed_kp(cybergear_motor_t *motor, float kp);
/**
 * @brief Sets the speed-controller integral gain.
 * @param motor Motor instance to configure.
 * @param ki Integral gain.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_set_speed_ki(cybergear_motor_t *motor, float ki);
/**
 * @brief Sets the speed target in rad/s.
 * @param motor Motor instance to command.
 * @param speed Speed target in rad/s.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or motor->config is null.
 * @return ESP_ERR_INVALID_STATE if motor->config->send is null.
 * @return Other error returned by motor->config->send.
 */
esp_err_t cybergear_set_speed(cybergear_motor_t *motor, float speed);

/**
 * @brief Copies the latest status.
 * @param motor Motor instance to query.
 * @param status Destination for the latest status.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or status is null.
 */
esp_err_t cybergear_get_status(cybergear_motor_t *motor, cybergear_status_t *status);
/**
 * @brief Copies the latest type-0 device identity response.
 * @param motor Motor instance to query.
 * @param device_info Destination for the decoded identity.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or device_info is null.
 */
esp_err_t cybergear_get_device_info(cybergear_motor_t *motor, cybergear_device_info_t *device_info);
/**
 * @brief Copies the latest fault flags.
 * @param motor Motor instance to query.
 * @param faults Destination for the latest fault flags.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or faults is null.
 */
esp_err_t cybergear_get_faults(cybergear_motor_t *motor, cybergear_fault_t *faults);
/**
 * @brief Copies the latest warning flags.
 * @param motor Motor instance to query.
 * @param warnings Destination for the latest warning flags.
 * @return ESP_OK on success.
 * @return ESP_ERR_INVALID_ARG if motor or warnings is null.
 */
esp_err_t cybergear_get_warnings(cybergear_motor_t *motor, cybergear_warning_t *warnings);
/**
 * @brief Returns whether the latest fault flags contain an active fault.
 * @param motor Motor instance to query.
 * @return true if a fault is active.
 * @return false if no fault is active or motor is null.
 */
bool cybergear_has_faults(cybergear_motor_t *motor);
/**
 * @brief Returns whether the latest warning flags contain an active warning.
 * @param motor Motor instance to query.
 * @return true if a warning is active.
 * @return false if no warning is active or motor is null.
 */
bool cybergear_has_warnings(cybergear_motor_t *motor);

#endif
