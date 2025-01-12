#include "esp_log.h"
#include "cybergear.h"

#define TAG "CyberGear"
#define btoa(x) ((x)?"true":"false")


void cybergear_print_faults(cybergear_fault_t *faults)
{
    ESP_LOGI(TAG, "Fault Overload: %s",btoa(faults->overload));
    ESP_LOGI(TAG, "Fault Uncalibrated: %s",btoa(faults->uncalibrated));
    ESP_LOGI(TAG, "Fault Over-Current Phase A: %s",btoa(faults->over_current_phase_a));
    ESP_LOGI(TAG, "Fault Over-Current Phase B: %s",btoa(faults->over_current_phase_b));
    ESP_LOGI(TAG, "Fault Over-Current Phase C: %s",btoa(faults->over_current_phase_c));
    ESP_LOGI(TAG, "Fault Over Voltage: %s",btoa(faults->over_voltage));
    ESP_LOGI(TAG, "Fault Under Voltage: %s",btoa(faults->under_voltage));
    ESP_LOGI(TAG, "Fault Driver-Chip: %s",btoa(faults->driver_chip));
    ESP_LOGI(TAG, "Fault Over-Temperature: %s",btoa(faults->over_temperature));
    ESP_LOGI(TAG, "Fault Magnetic Encoder: %s",btoa(faults->magnetic_code_failure));
    ESP_LOGI(TAG, "Fault Hall-Coded: %s",btoa(faults->hall_coded_faults));
}

const char *as_string(cybergear_state_e state)
{
    switch ((state))
    {
    case CYBERGEAR_STATE_RESET:
        return "RESET";
    case CYBERGEAR_STATE_CALIBRATION:
        return "CALIBRATION";
    case CYBERGEAR_STATE_RUNNING:
        return "RUNNING";
    default:
        return "UNKNOWN";
    }
}

void cybergear_print_status(cybergear_status_t *status)
{
    ESP_LOGI(TAG, "Temp: %f [°C] Mode: %s Pos: %f Speed: %f [rad/s] Torgue: %f [Nm]", 
		    status->temperature, 
			as_string(status->state),
			status->position,
            status->speed,
            status->torque
	);
}