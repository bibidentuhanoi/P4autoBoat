#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t motor_control_init(void);
esp_err_t motor_control_arm(void);
esp_err_t motor_control_disarm(void);

#ifdef __cplusplus
}
#endif
