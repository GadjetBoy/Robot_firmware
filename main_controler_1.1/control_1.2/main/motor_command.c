#include "motor_command.h"
#include "PID.h"
#include "BLE.h"

#define TAG "MOTOR_COMMANDS"

EventGroupHandle_t crawl_event_group;

 uint8_t phaseA_remaining = 0;
 uint8_t phaseB_remaining = 0;

// BLE inputs
int32_t ble_pos_setpoint[NUM_MOTORS] = {gate_pos_A,gate_pos_A,gate_pos_A,gate_pos_A,gate_pos_A,gate_pos_A,gate_pos_A,gate_pos_A};
int32_t ble_neg_setpoint[NUM_MOTORS] = {(-gate_pos_B),(-gate_pos_B),(-gate_pos_B),(-gate_pos_B),(-gate_pos_B),(-gate_pos_B),(-gate_pos_B),(-gate_pos_B)};

int32_t pos_target[NUM_MOTORS];
int32_t neg_target[NUM_MOTORS];

volatile SequenceMode current_mode ;
WaitFlags wait_flags = {0};

uint8_t gait_state = 0;

SemaphoreHandle_t motors_mutex = NULL;

// ---- Utility: update target arrays from BLE ----
void get_set_point(void) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        taskENTER_CRITICAL(&ble_mux);
        pos_target[i] = (int32_t)float_val[0];
        neg_target[i] = (int32_t)float_val[1];
        taskEXIT_CRITICAL(&ble_mux);
    }
}

uint8_t get_mode(void){
      
      taskENTER_CRITICAL(&ble_mux);
      current_mode = byte_val[0];
      taskEXIT_CRITICAL(&ble_mux);
    

    return current_mode ;
}

// ---- Example gait runner: Crawl ----
void run_crawl_gait(uint8_t *mode) {
  ESP_LOGI(TAG, "Mode[%d]",(int)*mode);
  ESP_LOGI(TAG, "crawl gate running");

  // Clear event bits at the start
  xEventGroupClearBits(crawl_event_group, PHASE_A_DONE_BIT | PHASE_B_DONE_BIT);
  

     //Phase A: even motors forward
  if (xSemaphoreTake(motors_mutex,pdMS_TO_TICKS(100)) == pdTRUE) {
     phaseA_remaining = NUM_MOTORS / 2;   // number of even motors
     for (int i = 0; i < NUM_MOTORS; i += 2) {
        motor_set(i, pos_target[i], true);
      }
      xSemaphoreGive(motors_mutex);
    }
     //ESP_LOGE(TAG, "waiting for PHASE_A_DONE_BIT ");
     //Block until all even motors done
        xEventGroupWaitBits(
            crawl_event_group,
            PHASE_A_DONE_BIT,
            pdTRUE,  // clear bit on exit
            pdTRUE,  // wait for all bits
            portMAX_DELAY
        );

     //Phase B: odd motors forward
   if (xSemaphoreTake(motors_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
     phaseB_remaining = NUM_MOTORS / 2;   // number of odd motors
     for (int i = 1; i < NUM_MOTORS; i += 2) {
         motor_set(i, pos_target[i], true);
        }
     xSemaphoreGive(motors_mutex);
    }
    //ESP_LOGE(TAG, "waiting for PHASE_B_DONE_BIT ");
    // Block until all odd motors done
        xEventGroupWaitBits(
            crawl_event_group,
            PHASE_B_DONE_BIT,
            pdTRUE,
            pdTRUE,
            portMAX_DELAY
        );

    // Phase A reverse
   if (xSemaphoreTake(motors_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    phaseA_remaining = NUM_MOTORS / 2; 
     for (int i = 0; i < NUM_MOTORS; i += 2) {
        motor_set(i, neg_target[i], true);
     }
     xSemaphoreGive(motors_mutex);
    }
    //ESP_LOGE(TAG, "waiting for PHASE_A_DONE_BIT ");
    // Block until all even motors done
        xEventGroupWaitBits(
            crawl_event_group,
            PHASE_A_DONE_BIT,
            pdTRUE,  // clear bit on exit
            pdTRUE,  // wait for all bits
            portMAX_DELAY
        );

     //Phase B reverse
   if (xSemaphoreTake(motors_mutex, pdMS_TO_TICKS(100)) == pdTRUE) {
     phaseB_remaining = NUM_MOTORS / 2; 
     for (int i = 1; i < NUM_MOTORS; i += 2) {
        motor_set(i, neg_target[i], true);
     }
     xSemaphoreGive(motors_mutex);
    }
    //ESP_LOGE(TAG, "waiting for PHASE_B_DONE_BIT ");
    // Block until all odd motors done
        xEventGroupWaitBits(
            crawl_event_group,
            PHASE_B_DONE_BIT,
            pdTRUE,
            pdTRUE,
            portMAX_DELAY
        );
}

void IDLE_gait(uint8_t *mode) {
     ESP_LOGI(TAG, "Mode[%d]",(int)*mode);

     // Clear event bits at the start
     xEventGroupClearBits(crawl_event_group, PHASE_A_DONE_BIT | PHASE_B_DONE_BIT);
     
     ESP_LOGI(TAG, "IDLE gate running");
      //Phase A: even motors 0
      phaseA_remaining = NUM_MOTORS / 2;   // number of even motors
     if (xSemaphoreTake(motors_mutex, pdMS_TO_TICKS(100)) == pdTRUE){
     for (int i = 0; i < NUM_MOTORS; i += 2) {
        motor_set(i, 0, true);
     }
     xSemaphoreGive(motors_mutex);
    }
    ESP_LOGW(TAG, "waiting for PHASE_A_DONE_BIT ");
    // Block until all even motors done
        xEventGroupWaitBits(
            crawl_event_group,
            PHASE_A_DONE_BIT,
            pdTRUE,  // clear bit on exit
            pdTRUE,  // wait for all bits
            portMAX_DELAY
        );

    // Phase B: odd motors 0
     phaseB_remaining = NUM_MOTORS / 2;   // number of odd motors
     if (xSemaphoreTake(motors_mutex, pdMS_TO_TICKS(100)) == pdTRUE){ 
     for (int i = 1; i < NUM_MOTORS; i += 2) {
          motor_set(i, 0, true);
        }
     xSemaphoreGive(motors_mutex);
    }

    ESP_LOGW(TAG, "waiting for PHASE_B_DONE_BIT ");
    // Block until all odd motors done
        xEventGroupWaitBits(
            crawl_event_group,
            PHASE_B_DONE_BIT,
            pdTRUE,
            pdTRUE,
            portMAX_DELAY
        );

     ESP_LOGE(TAG, "ALL motors STOPED");
    
}

void run_STANDBY_gait(void){

  for (int i = 0; i < NUM_MOTORS; i+=2) {
        motor_set(i,neg_target[i], true);
    }

}

// ---- Sequence task dispatcher ----
void sequence_runner_task(void *arg) {
    
    uint8_t mode;
    while (1) {
        get_set_point();

        mode = get_mode();

        switch (mode) {
            case MODE_CRAWL:
                run_crawl_gait(&mode);
                break;

            case MODE_TURTLE:
                // implement turtle gait here
                break;

            case MODE_STANDBY:
                run_STANDBY_gait();
                break;

            case MODE_IDLE:
                IDLE_gait(&mode);
                break;

            default:
                vTaskDelay(pdMS_TO_TICKS(50));
                break;
        }

      vTaskDelay(pdMS_TO_TICKS(10));
    }


}

void sequence_runner_task(void *arg) {
    uint8_t mode;
    uint8_t last_mode = 255; // Force an update on first run

    while (1) {
        // get_set_point(); // You might not need this anymore,
                         // Amplitudes/Offsets are set by the gait function.
                         // You could use BLE to tune them, though!

        mode = get_mode();

        if (mode != last_mode) { // Only update when the mode *changes*
            ESP_LOGI(TAG, "Mode change detected: %d", mode);
            switch (mode) {
                case MODE_CRAWL:
                    // You'll need to write set_gait_crawl()
                    // set_gait_crawl(); 
                    ESP_LOGI(TAG, "Setting CRAWL gait");
                    break;

                case MODE_TURTLE: // Let's map this to TROT
                    set_gait_trot();
                    ESP_LOGI(TAG, "Setting TROT gait");
                    break;

                case MODE_STANDBY:
                    break;
                case MODE_IDLE:
                    set_gait_idle();
                    ESP_LOGI(TAG, "Setting IDLE gait");
                    break;

                default:
                    // Maybe just default to IDLE
                    set_gait_idle();
                    break;
            }
            last_mode = mode; // Remember the last mode
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Check for a new mode 10 times/sec
    }
}

// ---- Main controller task ----
void mot_command_app_main(void) {

   ESP_LOGI(TAG, "settingup the motor comnads");

   motors_mutex = xSemaphoreCreateMutex();
    if (motors_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create BLE data mutex");
        return;
    }

    crawl_event_group = xEventGroupCreate();
    if (crawl_event_group == NULL) {
        ESP_LOGE(TAG, "Failed to create event group");
        return;
    }

   TaskHandle_t seq_runner_handle = NULL;

   xTaskCreatePinnedToCore(sequence_runner_task, 
                            "seq_runner",
                             4096, 
                             NULL, 
                             8, 
                             &seq_runner_handle, 
                             1);

   ESP_LOGI(TAG, "settingup the motor comnads complete");
    
}
