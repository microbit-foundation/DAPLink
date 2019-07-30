/**
 * @file power_monitor.h
 * @brief
 */

#ifndef PWR_MON_H
#define PWR_MON_H

#include "IO_Config.h"

#ifdef __cplusplus
extern "C" {
#endif


void pwr_mon_init(void);
bool pwr_mon_battery_powered(void);
uint32_t pwr_mon_vcc_mv(void);


#ifdef __cplusplus
}
#endif

#endif
