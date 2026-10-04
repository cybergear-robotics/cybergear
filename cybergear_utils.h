#ifndef CYBERGEAR_UTILS_H
#define CYBERGEAR_UTILS_H

#include "cybergear.h"

/**
 * @brief Logs the raw motor diagnostic masks.
 * @param diagnostics Raw diagnostic masks to log.
 */
void cybergear_print_diagnostics(cybergear_diagnostics_t *diagnostics);
/**
 * @brief Logs the latest motor status values.
 * @param status Status values to log.
 */
void cybergear_print_status(cybergear_status_t *status);

#endif
