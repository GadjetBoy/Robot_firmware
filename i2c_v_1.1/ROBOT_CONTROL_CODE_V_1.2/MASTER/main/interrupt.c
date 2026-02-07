#include <stdio.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_intr_types.h"
#include "esp_intr_alloc.h"
#include "soc/gpio_reg.h"

#define TAG "MASTER"
#define NUM_ENCORDERS 8
static spinlock_t spinlock = portMUX_INITIALIZER_UNLOCKED;

#define ENCORDER1_PINA 1
#define ENCORDER2_PINA 2
#define ENCORDER3_PINA 3
#define ENCORDER4_PINA 4
#define ENCORDER5_PINA 5
#define ENCORDER6_PINA 6
#define ENCORDER7_PINA 7
#define ENCORDER8_PINA 8
#define ENCORDER1_PINB 9
#define ENCORDER2_PINB 10
#define ENCORDER3_PINB 11
#define ENCORDER4_PINB 12
#define ENCORDER5_PINB 13
#define ENCORDER6_PINB 14
#define ENCORDER7_PINB 15
#define ENCORDER8_PINB 16

#define GPIO_ENCORDER_INPUTS ((1ULL<<ENCORDER1_PINA)|(1ULL<<ENCORDER1_PINB)|(1ULL<<ENCORDER2_PINA)|(1ULL<<ENCORDER2_PINB)\
                             |(1ULL<<ENCORDER3_PINA)|(1ULL<<ENCORDER3_PINB)|(1ULL<<ENCORDER4_PINA)|(1ULL<<ENCORDER4_PINB)\
                             |(1ULL<<ENCORDER5_PINA)|(1ULL<<ENCORDER5_PINB)|(1ULL<<ENCORDER6_PINA)|(1ULL<<ENCORDER6_PINB)\
                             |(1ULL<<ENCORDER7_PINA)|(1ULL<<ENCORDER7_PINB)|(1ULL<<ENCORDER8_PINA)|(1ULL<<ENCORDER8_PINB))


#define ISR_FLAGS  (ESP_INTR_FLAG_LEVEL3|ESP_INTR_FLAG_EDGE|ESP_INTR_FLAG_IRAM)

const static uint8_t INTERRUPT_GPIO[]={ENCORDER1_PINA,ENCORDER2_PINA, \
                                       ENCORDER3_PINA,ENCORDER4_PINA, \
                                       ENCORDER5_PINA,ENCORDER6_PINA, \
                                       ENCORDER7_PINA,ENCORDER8_PINA};

const static uint8_t COMP_GPIO[]= {ENCORDER1_PINB,ENCORDER2_PINB, \
                                   ENCORDER3_PINB,ENCORDER4_PINB, \
                                   ENCORDER5_PINB,ENCORDER6_PINB, \
                                   ENCORDER7_PINB,ENCORDER8_PINB};

volatile DRAM_ATTR int16_t ENCORDER_COUNT[NUM_ENCORDERS]={0};

TaskHandle_t T_Handle[NUM_ENCORDERS];

volatile uint8_t DRAM_ATTR ISR_BEGIN[8] = {0}; 


// macro to generate ISR_function for each interrupt 
#define ISR_ENCORDER(n)   static void IRAM_ATTR ISR_ENCORDER_HANDLER_##n(void*arg){            \
                                   BaseType_t xYieldRequired;                                  \
                                   taskENTER_CRITICAL_ISR(&spinlock) ;                                                           \
                                   if(ISR_BEGIN[n-1] == 0){                                     \
                                     ISR_BEGIN[n-1] = 1 ;                                        \
                                    }                                                          \
                                  taskEXIT_CRITICAL_ISR(&spinlock) ;                                                           \
                                   xYieldRequired = xTaskResumeFromISR( T_Handle[n-1] );       \
                                                                                           \
                                   portYIELD_FROM_ISR( xYieldRequired );                       \
                                                              \
                                  }          
        


#define Tasks_for_ISR(n)  static void TASK_OF_EN_##n( void * pvParameters ){              \
                                                                                          \
                                 for (;;) {                                               \
                                     taskENTER_CRITICAL(&spinlock);                                                      \
                                   if (ISR_BEGIN[n-1] == 1){                                \
                                                                 \
                                     uint8_t comp_val = gpio_get_level(COMP_GPIO[n-1]);   \
                                     if (comp_val > 0){                                   \
                                     ENCORDER_COUNT[n-1]++;                               \
                                     }                                                    \
                                     else{                                                \
                                     ENCORDER_COUNT[n-1]--;                               \
                                    } }                                                   \
                                     taskEXIT_CRITICAL(&spinlock);                        \
                                   vTaskSuspend(NULL);                                    \
                                                                    \
                                  }                                                       \
                                }                                                     

// generate ISRs
ISR_ENCORDER(1)
ISR_ENCORDER(2)
ISR_ENCORDER(3)
ISR_ENCORDER(4)
ISR_ENCORDER(5)
ISR_ENCORDER(6)
ISR_ENCORDER(7)
ISR_ENCORDER(8)

//generate tasks
Tasks_for_ISR(1)
Tasks_for_ISR(2)
Tasks_for_ISR(3)
Tasks_for_ISR(4)
Tasks_for_ISR(5)
Tasks_for_ISR(6)
Tasks_for_ISR(7)
Tasks_for_ISR(8)

 
 typedef void (*Function_Pointer)(void *arg);

 Function_Pointer ISR_functions[] ={ISR_ENCORDER_HANDLER_1,ISR_ENCORDER_HANDLER_2,\
                                    ISR_ENCORDER_HANDLER_3,ISR_ENCORDER_HANDLER_4,\
                                    ISR_ENCORDER_HANDLER_5,ISR_ENCORDER_HANDLER_6,\
                                    ISR_ENCORDER_HANDLER_7,ISR_ENCORDER_HANDLER_8} ;


 Function_Pointer TASKS[] = {  TASK_OF_EN_1,\
                              TASK_OF_EN_2,\
                              TASK_OF_EN_3,\
                              TASK_OF_EN_4,\
                              TASK_OF_EN_5,\
                              TASK_OF_EN_6,\
                              TASK_OF_EN_7,\
                              TASK_OF_EN_8} ;

// setting up GPIO pins

void PIN_setup(void){

 gpio_config_t GPIO_Config={

     //set as output mode
     .mode = GPIO_MODE_INPUT,
     //bit mask of the pins that you want to set
     .pin_bit_mask = GPIO_ENCORDER_INPUTS,
     //enable pull-up mode
     .pull_up_en = 0,
     //disable pull-down mode
     .pull_down_en = 0,
     //enable interrupt
     .intr_type = GPIO_INTR_POSEDGE,

    };

 ESP_ERROR_CHECK(gpio_config(&GPIO_Config));
 

}

// setting up interrupts

void INTERRUPT_SET(void){

 ESP_ERROR_CHECK(gpio_install_isr_service(ISR_FLAGS));

 for (int i =0;i < NUM_ENCORDERS ;i++ ){

    ESP_ERROR_CHECK(gpio_isr_handler_add(INTERRUPT_GPIO[i],ISR_functions[i],NULL));

  }

  
}


void CREATE_TASKS(void){

 BaseType_t ret ;
 for (int i = 0; i<8;i++){
    
   ret =xTaskCreatePinnedToCore( TASKS[i],                   //Task implimentation  
                                "ENCORDER Task",             //name 
                                 2096 ,                      //allocated heap size of the task
                                 NULL,                       //argument for the function
                                 (2 | portPRIVILEGE_BIT ) ,  //task prority
                                 &T_Handle[i] ,              //task handle
                                 1);            //core affinity:set run on any core
   
   if (ret != pdPASS) {
        ESP_LOGE(TAG, "Task creation failed for encoder %d", i+1);
        // handle error or abort if necessary
    }
  } 
 
  
}

