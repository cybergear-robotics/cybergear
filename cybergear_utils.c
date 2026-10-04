#include <inttypes.h>

#include "esp_log.h"
#include "cybergear.h"

#define TAG "CyberGear"

static void print_active(const char *source, const char *name)
{
    ESP_LOGW(TAG, "%s: %s", source, name);
}

void cybergear_print_diagnostics(cybergear_diagnostics_t *diagnostics)
{
    const uint32_t status_known = CYBERGEAR_STATUS_FAULT_UNDER_VOLTAGE |
                                  CYBERGEAR_STATUS_FAULT_OVER_CURRENT |
                                  CYBERGEAR_STATUS_FAULT_OVER_TEMPERATURE |
                                  CYBERGEAR_STATUS_FAULT_MAGNETIC_ENCODER |
                                  CYBERGEAR_STATUS_FAULT_HALL |
                                  CYBERGEAR_STATUS_FAULT_UNCALIBRATED;
    const uint32_t fault_known = CYBERGEAR_FAULT_DRIVER_CHIP |
                                 CYBERGEAR_FAULT_UNDER_VOLTAGE |
                                 CYBERGEAR_FAULT_OVER_VOLTAGE |
                                 CYBERGEAR_FAULT_OVER_CURRENT_PHASE_B |
                                 CYBERGEAR_FAULT_OVER_CURRENT_PHASE_C |
                                 CYBERGEAR_FAULT_UNCALIBRATED |
                                 CYBERGEAR_FAULT_OVERLOAD_MASK |
                                 CYBERGEAR_FAULT_OVER_CURRENT_PHASE_A;

    ESP_LOGI(TAG, "Status Fault Mask: 0x%08" PRIx32, diagnostics->status_fault_mask);
    ESP_LOGI(TAG, "Fault Mask: 0x%08" PRIx32, diagnostics->fault_mask);
    ESP_LOGI(TAG, "Warning Mask: 0x%08" PRIx32, diagnostics->warning_mask);

    if (diagnostics->status_fault_mask & CYBERGEAR_STATUS_FAULT_UNDER_VOLTAGE) print_active("Status fault", "under voltage");
    if (diagnostics->status_fault_mask & CYBERGEAR_STATUS_FAULT_OVER_CURRENT) print_active("Status fault", "over current");
    if (diagnostics->status_fault_mask & CYBERGEAR_STATUS_FAULT_OVER_TEMPERATURE) print_active("Status fault", "over temperature");
    if (diagnostics->status_fault_mask & CYBERGEAR_STATUS_FAULT_MAGNETIC_ENCODER) print_active("Status fault", "magnetic encoder failure");
    if (diagnostics->status_fault_mask & CYBERGEAR_STATUS_FAULT_HALL) print_active("Status fault", "hall failure");
    if (diagnostics->status_fault_mask & CYBERGEAR_STATUS_FAULT_UNCALIBRATED) print_active("Status fault", "uncalibrated");

    if (diagnostics->fault_mask & CYBERGEAR_FAULT_DRIVER_CHIP) print_active("Detailed fault", "driver chip");
    if (diagnostics->fault_mask & CYBERGEAR_FAULT_UNDER_VOLTAGE) print_active("Detailed fault", "under voltage");
    if (diagnostics->fault_mask & CYBERGEAR_FAULT_OVER_VOLTAGE) print_active("Detailed fault", "over voltage");
    if (diagnostics->fault_mask & CYBERGEAR_FAULT_OVER_CURRENT_PHASE_A) print_active("Detailed fault", "phase A over current");
    if (diagnostics->fault_mask & CYBERGEAR_FAULT_OVER_CURRENT_PHASE_B) print_active("Detailed fault", "phase B over current");
    if (diagnostics->fault_mask & CYBERGEAR_FAULT_OVER_CURRENT_PHASE_C) print_active("Detailed fault", "phase C over current");
    if (diagnostics->fault_mask & CYBERGEAR_FAULT_UNCALIBRATED) print_active("Detailed fault", "uncalibrated");
    if (diagnostics->fault_mask & CYBERGEAR_FAULT_OVERLOAD_MASK) print_active("Detailed fault", "overload");

    if (diagnostics->warning_mask & CYBERGEAR_WARNING_OVER_TEMPERATURE) print_active("Warning", "over temperature");

    if (diagnostics->status_fault_mask & ~status_known) ESP_LOGW(TAG, "Status fault: unknown bits 0x%08" PRIx32, diagnostics->status_fault_mask & ~status_known);
    if (diagnostics->fault_mask & ~fault_known) ESP_LOGW(TAG, "Detailed fault: unknown bits 0x%08" PRIx32, diagnostics->fault_mask & ~fault_known);
    if (diagnostics->warning_mask & ~CYBERGEAR_WARNING_OVER_TEMPERATURE) ESP_LOGW(TAG, "Warning: unknown bits 0x%08" PRIx32, diagnostics->warning_mask & ~CYBERGEAR_WARNING_OVER_TEMPERATURE);
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
