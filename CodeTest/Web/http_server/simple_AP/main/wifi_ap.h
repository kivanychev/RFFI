#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_err.h"
#include "esp_netif.h"


esp_err_t wifi_init_softap(void);

#ifdef __cplusplus
}
#endif
