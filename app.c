/*
*********************************************************************************************************
*                                            EXAMPLE CODE
*
*               This file is provided as an example on how to use Micrium products.
*
*               Please feel free to use any application code labeled as 'EXAMPLE CODE' in
*               your application products.  Example code may be used as is, in whole or in
*               part, or may be used as a reference only. This file can be modified as
*               required to meet the end-product requirements.
*
*               Please help us continue to provide the Embedded community with the finest
*               software available.  Your honesty is greatly appreciated.
*
*               You can find our product's user manual, API reference, release notes and
*               more information at https://doc.micrium.com.
*               You can contact us at www.micrium.com.
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*
*                                            EXAMPLE CODE
*
*                                        Freescale Kinetis K64
*                                               on the
*
*                                         Freescale FRDM-K64F
*                                          Evaluation Board
*
* Filename      : app.c
* Version       : V1.00
* Programmer(s) : FF
*********************************************************************************************************
*/

/*
*********************************************************************************************************
*                                             INCLUDE FILES
*********************************************************************************************************
*/
#include "fsl_interrupt_manager.h"

#include  <math.h>
#include  <lib_math.h>
#include  <cpu_core.h>

#include  <app_cfg.h>
#include  <os.h>

#include  <fsl_os_abstraction.h>
#include  <system_MK64F12.h>
#include  <board.h>

#include  <bsp_ser.h>

//#include "fsl_gpio_driver.h"
#include "fsl_gpio_common.h"  //gport_baseAddr function is using this library
//#include "fsl_clock_manager.h"
//#include "fsl_interrupt_manager.h"

/*
*********************************************************************************************************
*                                            LOCAL DEFINES
*********************************************************************************************************
*/


/*
*********************************************************************************************************
*                                       LOCAL GLOBAL VARIABLES
*********************************************************************************************************
*/

static  OS_TCB       AppTaskStartTCB;
static  CPU_STK      AppTaskStartStk[APP_CFG_TASK_START_STK_SIZE];

static  OS_TCB       TaskMeasureTCB;
static  CPU_STK      TaskMeasureStack[APP_CFG_TASK_START_STK_SIZE];


/*
*********************************************************************************************************
*                                      LOCAL DATA STRUCTURES
*********************************************************************************************************
*/


//define color of the led
typedef enum{
  BLUE=BOARD_GPIO_LED_BLUE,
  GREEN=BOARD_GPIO_LED_GREEN,
  RED=BOARD_GPIO_LED_RED,
  NO_LED
}led_color;

typedef enum{
  RISE,
  FALL,
  OFF,
  OVERFLOW
}echo_state;


//basic sensor data stored by this structure
typedef struct {
  led_color current_led; //current blinking led
  led_color new_led; //
  uint16_t freq;
  echo_state state;
  float distance;
  uint8_t start;
}sensor_state;


static sensor_state sensor_data;

/*
*********************************************************************************************************
*                                      LOCAL FUNCTION PROTOTYPES
*********************************************************************************************************
*/

static  void  AppTaskStart (void  *p_arg); //task
static  void  TaskMeasure (void  *p_arg);
void BSP_inputB9_int_hdlr( void ) ;
void init_sensor_state();
/*
*********************************************************************************************************
*                                                main()
*
* Description : This is the standard entry point for C code.  It is assumed that your code will call
*               main() once you have performed all necessary initialization.
*
* Argument(s) : none.
*
* Return(s)   : none.
*
* Caller(s)   : This the main standard entry point.
*
* Note(s)     : none.
*********************************************************************************************************
*/

int  main (void)
{
  OS_ERR   err;
  
#if (CPU_CFG_NAME_EN == DEF_ENABLED)
  CPU_ERR  cpu_err;
#endif
  
  hardware_init();
  GPIO_DRV_Init(switchPins, ledPins);
  
  
#if (CPU_CFG_NAME_EN == DEF_ENABLED)
  CPU_NameSet((CPU_CHAR *)"MK64FN1M0VMD12",
              (CPU_ERR  *)&cpu_err);
#endif
  
  OSA_Init();                                                 /* Init uC/OS-III.                                      */                     
  
  OSTaskCreate(&AppTaskStartTCB,                              /* Create the start task                                */
               "App Task Start",
               AppTaskStart,
               0u,
               APP_CFG_TASK_START_PRIO,
               &AppTaskStartStk[0u],
               (APP_CFG_TASK_START_STK_SIZE / 10u),
               APP_CFG_TASK_START_STK_SIZE,
               0u,
               0u,
               0u,
               (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
               &err);
  
  
  OSA_Start();                                                /* Start multitasking (i.e. give control to uC/OS-III). */
  
  while (DEF_ON) {                                            /* Should Never Get Here                                */
    
  }    
}


/*
*********************************************************************************************************
*                                          STARTUP TASK
*
* Description : This is an example of a startup task.  As mentioned in the book's text, you MUST
*               initialize the ticker only once multitasking has started.
*
* Argument(s) : p_arg   is the argument passed to 'App_TaskStart()' by 'OSTaskCreate()'.
*
* Return(s)   : none.
*
* Caller(s)   : This is a task.
*
* Notes       : (1) The first line of code is used to prevent a compiler warning because 'p_arg' is not
*                   used.  The compiler should not generate any code for this statement.
*********************************************************************************************************
*/

static  void  AppTaskStart (void *p_arg) 
{ 
  OS_ERR      err;
  (void)p_arg;           
  
  CPU_Init(); 
  Mem_Init(); 
  Math_Init(); 
  BSP_Ser_Init(115200u);  //without this call APP_TRACE_DBG does not work
  init_sensor_state();
  //create measusrement task
  OSTaskCreate(&TaskMeasureTCB,                              /* Create the start task                                */
               "App Task Start",
               TaskMeasure,
               0u,
               APP_CFG_TASK_START_PRIO,
               &TaskMeasureStack[0u],
               (APP_CFG_TASK_START_STK_SIZE / 10u),
               APP_CFG_TASK_START_STK_SIZE,
               0u,
               0u,
               0u,
               (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR | OS_OPT_TASK_SAVE_FP),
               &err);
  
  INT_SYS_InstallHandler(PORTB_IRQn, BSP_inputB9_int_hdlr);//install interrupt
  while (DEF_ON){
    if(sensor_data.distance >= 200){
      sensor_data.new_led=RED;
      sensor_data.freq=2000;
    }else if(sensor_data.distance >= 100){
      sensor_data.new_led=BLUE;
      sensor_data.freq=((((int)sensor_data.distance-100)/20)+1)*200;
    }else if(sensor_data.distance >= 10){
      sensor_data.new_led=GREEN;
      sensor_data.freq=((int)sensor_data.distance/25+2)*200;
    }
    else{
      sensor_data.new_led=GREEN;
      sensor_data.freq=0;
    }
    if(sensor_data.current_led != sensor_data.new_led){
      GPIO_DRV_SetPinOutput( sensor_data.current_led );
      GPIO_DRV_ClearPinOutput( sensor_data.new_led );
      sensor_data.current_led=sensor_data.new_led;
    }else{
      GPIO_DRV_TogglePinOutput(sensor_data.current_led);
    }
    OSTimeDlyHMSM(0u, 0u, 0u, sensor_data.freq,
                  OS_OPT_TIME_HMSM_STRICT,
                  &err);
  } 
} 

//function that initializes 
void init_sensor_state(){
  sensor_data.current_led = NO_LED;
  sensor_data.state=OFF;
  sensor_data.distance=0;
  sensor_data.start=0;
}



void BSP_inputB9_int_hdlr( void ) 
{ 
  OS_ERR      err;
  static uint32_t ifsr;         /* interrupt flag status register */ 
  uint32_t portBaseAddr = g_portBaseAddr[GPIO_EXTRACT_PORT(inPTB9)]; 
  
  CPU_CRITICAL_ENTER(); 
  OSIntEnter();  /* Tell the OS that we are starting an ISR */ 
  
  ifsr = PORT_HAL_GetPortIntFlag(portBaseAddr); 
  
  if( (ifsr & 0x200u) ) /* Check if inPTB9 generated the interrupt */ 
  {
    //OSTaskSemPost(&TaskMeasureTCB, OS_OPT_POST_1, &err); //defines measurement
    if(sensor_data.state==OFF){
      sensor_data.state=RISE;
    }else if(sensor_data.state==RISE){
      sensor_data.state=FALL;
    }
    GPIO_DRV_ClearPinIntFlag( inPTB9 ); 
  } 
  
  CPU_CRITICAL_EXIT(); 
  
  OSIntExit(); 
}


static  void  TaskMeasure (void *p_arg) 
{
  CPU_ERR     cpu_err;
  char        tmp[80];
  CPU_TS64    before, after,current;
  uint32_t    frq = CPU_TS_TmrFreqGet( &cpu_err );
  CPU_TS      ts;
  OS_ERR    err;
  while(DEF_ON){
    GPIO_DRV_ClearPinOutput( outPTB23 ); //pin out = 1  
    while(DEF_ON){
      if(sensor_data.state==RISE && !sensor_data.start){
        before = CPU_TS_Get64();
        sensor_data.start=1;
        GPIO_DRV_SetPinOutput(outPTB23);
      }else if(sensor_data.state==FALL){
        after = CPU_TS_Get64();
        sensor_data.distance = (float)((1000.0/58*(after-before))/(frq/1000));
        sprintf( tmp, "DISTANCE = %f cm\n\r",sensor_data.distance );
        APP_TRACE_DBG(( tmp ));
        sensor_data.state=OFF;
        sensor_data.start=0;
        break;
      }
      if(sensor_data.start){
        current=CPU_TS_Get64();
        if(43*(current-before)>frq){//overflow state
          sprintf( tmp, "OVERFLOW: %llu\n\r",current);
          APP_TRACE_DBG(( tmp ));
          sensor_data.state=OFF;
          sensor_data.start=0;
          GPIO_DRV_SetPinOutput(outPTB23);
          break;
        }
      }
    }
    OSTimeDlyHMSM(0u, 0u, 0u, 50u, OS_OPT_TIME_HMSM_STRICT, &err);//wait 10 ms for the next execution
  }
  
}