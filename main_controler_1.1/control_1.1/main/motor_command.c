#include "control_1.1.h"

#define NUM_GATES 4
#define TAG "MOTOR_COMMANDS"

#define gate_pos_A 28000//5088
#define gate_pos_B 28000//5088

 EventGroupHandle_t crawl_event_group;
 uint8_t phaseA_remaining = 0;
 uint8_t phaseB_remaining = 0;

typedef enum {
    MODE_IDLE = 0,
    MODE_CRAWL,
    MODE_STANDBY,
    MODE_TURTLE
} SequenceMode;

typedef struct {
    bool phaseA;
    bool phaseB;
    bool phaseC;
    bool phaseD;
} WaitFlags;

// BLE inputs
int ble_pos_setpoint[NUM_MOTORS] = {gate_pos_A,gate_pos_A,gate_pos_A,gate_pos_A,gate_pos_A,gate_pos_A,gate_pos_A,gate_pos_A};
int ble_neg_setpoint[NUM_MOTORS] = {(-gate_pos_B),(-gate_pos_B),(-gate_pos_B),(-gate_pos_B),(-gate_pos_B),(-gate_pos_B),(-gate_pos_B),(-gate_pos_B)};

int pos_target[NUM_MOTORS];
int neg_target[NUM_MOTORS];

volatile SequenceMode current_mode ;
WaitFlags wait_flags = {0};

uint8_t gait_state = 0;

// ---- Utility: update target arrays from BLE ----
void get_set_point(void) {
    for (int i = 0; i < NUM_MOTORS; i++) {
        //taskENTER_CRITICAL(&ble_mux);
        pos_target[i] = ble_pos_setpoint[i]; //(int)float_val[i];
        neg_target[i] = ble_neg_setpoint[i] ;//(int)float_val[i+NUM_MOTORS];
        //taskEXIT_CRITICAL(&ble_mux);
    }
}

uint8_t get_mode(void){
      
      taskENTER_CRITICAL(&ble_mux);
      current_mode = byte_val[0];
      taskEXIT_CRITICAL(&ble_mux);
    

    return current_mode ;
}

// ---- Example gait runner: Crawl ----
void run_crawl_gait(void) {
     ESP_LOGI(TAG, "crawl gate running");
    // Phase A: even motors forward
     //phaseA_remaining = NUM_MOTORS / 2;   // number of even motors
    //for (int i = 0; i < NUM_MOTORS; i += 2) {
        motor_set(&motors[1], pos_target[1], true);
    //}
    // Block until all even motors done
        xEventGroupWaitBits(
            crawl_event_group,
            PHASE_A_DONE_BIT,
            pdTRUE,  // clear bit on exit
            pdTRUE,  // wait for all bits
            portMAX_DELAY
        );

    // Phase B: odd motors forward
    // phaseB_remaining = NUM_MOTORS / 2;   // number of odd motors
    //for (int i = 1; i < NUM_MOTORS; i += 2) {
      //  motor_set(&motors[1], pos_target[1], true);
    //}
    // Block until all odd motors done
      /*  xEventGroupWaitBits(
            crawl_event_group,
            PHASE_B_DONE_BIT,
            pdTRUE,
            pdTRUE,
            portMAX_DELAY
        );*/

    // Phase A reverse
    phaseA_remaining = NUM_MOTORS / 2; 
    //for (int i = 0; i < NUM_MOTORS; i += 2) {
        motor_set(&motors[1], neg_target[1], true);
   // }
    // Block until all even motors done
        xEventGroupWaitBits(
            crawl_event_group,
            PHASE_A_DONE_BIT,
            pdTRUE,  // clear bit on exit
            pdTRUE,  // wait for all bits
            portMAX_DELAY
        );

    // Phase B reverse
   // phaseB_remaining = NUM_MOTORS / 2; 
    //for (int i = 1; i < NUM_MOTORS; i += 2) {
       // motor_set(&motors[1], neg_target[1], true);
    //}
    // Block until all odd motors done
       /* xEventGroupWaitBits(
            crawl_event_group,
            PHASE_B_DONE_BIT,
            pdTRUE,
            pdTRUE,
            portMAX_DELAY
        );*/
}

void IDLE_gait(void) {
     ESP_LOGI(TAG, "IDLE gate running");
    // Phase A: even motors 0
    // phaseA_remaining = NUM_MOTORS / 2;   // number of even motors
    //for (int i = 0; i < NUM_MOTORS; i += 2) {
        motor_set(&motors[1], 0, true);
    //}
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
    for (int i = 1; i < NUM_MOTORS; i += 2) {
        motor_set(&motors[i], 0, true);
    }
    // Block until all odd motors done
       /* xEventGroupWaitBits(
            crawl_event_group,
            PHASE_B_DONE_BIT,
            pdTRUE,
            pdTRUE,
            portMAX_DELAY
        );*/

     ESP_LOGE(TAG, "ALL motors STOPED");
    
}

void run_STANDBY_gait(void){

  for (int i = 0; i < NUM_MOTORS; i+=2) {
        motor_set(&motors[i],neg_target[i], true);
    }

}

// ---- Sequence task dispatcher ----
void sequence_runner_task(void *arg) {
    
    uint8_t mode;
    while (1) {
        get_set_point();

        ESP_LOGW(TAG, "recieve mode");
        mode = get_mode();
        ESP_LOGI(TAG, "Mode[%d]",mode);

        switch (mode) {
            case MODE_CRAWL:
                run_crawl_gait();
                break;

            case MODE_TURTLE:
                // implement turtle gait here
                break;

            case MODE_STANDBY:
                run_STANDBY_gait();
                break;

            case MODE_IDLE:
                IDLE_gait();
                break;
            default:
                vTaskDelay(pdMS_TO_TICKS(50));
                break;
        }

      vTaskDelay(pdMS_TO_TICKS(10));
    }


}

// ---- Main controller task ----
void mot_command_app_main(void) {

   ESP_LOGI(TAG, "settingup the motor comnads");

   crawl_event_group = xEventGroupCreate();
   TaskHandle_t seq_runner_handle = NULL;

   xTaskCreatePinnedToCore(sequence_runner_task, 
                            "seq_runner",
                             4096, 
                             NULL, 
                             21, 
                             &seq_runner_handle, 
                             0);

   ESP_LOGI(TAG, "settingup the motor comnads complete");
    
}
