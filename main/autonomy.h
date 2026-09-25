#pragma once

#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* The out-and-back mission on the boat (2026-09-25).  A 20 Hz task on core 1
 * runs main/mission.c on the live GPS and heading and hands the control task
 * its setpoint (motor_control_set_auto_setpoint).  The radio only carries
 * START and STOP: the mission finishes through any outage.
 * Spec: docs/superpowers/specs/2026-09-24-out-and-back-mission-design.md */

/* Registers the MissionCommand handler and starts the task.  Call after
 * pipeline_init() and motor_control_init(), before the radio starts.  Never
 * fatal: without the task the boat has no mission (START gets no answer) and
 * drives exactly as before. */
esp_err_t autonomy_init(void);

/* The core-1 diagnostics task, every loop: publishes MissionStatus when there
 * is a new one (5 Hz while a mission runs, 1 Hz otherwise) and writes a
 * finished run to the SD card -- both far too slow for the mission task. */
void autonomy_diagnostics_tick(bool periodic);

#ifdef __cplusplus
}
#endif
