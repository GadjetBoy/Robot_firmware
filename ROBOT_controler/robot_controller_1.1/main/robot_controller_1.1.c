#include "robot_controller.h"

void app_main(void)
{
  BLE_app_main();
  vTaskDelay(pdMS_TO_TICKS(100));
  pcnt_app_main();
  pwm_app_main();
  pid_app_main();

  //esp_task_wdt_add(NULL); // Add watchdog

}
