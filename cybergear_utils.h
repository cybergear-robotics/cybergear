#ifndef CYBERGEAR_UTILS_H
#define CYBERGEAR_UTILS_H

#include "cybergear.h"

/**
 * @brief Logs each active and inactive motor fault flag.
 * @param faults Fault flags to log.
 */
void cybergear_print_faults(cybergear_fault_t *faults);
/**
 * @brief Logs each active and inactive motor warning flag.
 * @param warnings Warning flags to log.
 */
void cybergear_print_warnings(cybergear_warning_t *warnings);
/**
 * @brief Logs the latest motor status values.
 * @param status Status values to log.
 */
void cybergear_print_status(cybergear_status_t *status);

#endif
