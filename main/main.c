#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "svpwm.h"

void app_main() {
    ESP_LOGI("MAIN", "Starting SVPWM application");
    
    setupPWM();
    startSvpwmTask();
}