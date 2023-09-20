/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MYLIB_H
#define __MYLIB_H

#ifdef __cplusplus
extern "C"
{
#endif

/*INCLUDE---------------------------------------------------------------------*/
#include "stm32f1xx_hal.h"

    /* Private typedef -----------------------------------------------------------*/
    /* USER CODE BEGIN PTD */
    typedef enum state
    {
        red,
        yellow,
        green
    } STATE;
    typedef struct traffic_light
    {
        GPIO_TypeDef *LED_RED_Port;
        GPIO_TypeDef *LED_YELLOW_Port;
        GPIO_TypeDef *LED_GREEN_Port;
        uint16_t LED_RED_Pin;
        uint16_t LED_YELLOW_Pin;
        uint16_t LED_GREEN_Pin;
        uint16_t red_time;
        uint16_t yellow_time;
        uint16_t green_time;
    } TRAFFIC_LIGHT;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RED 5
#define YELLOW 2
#define GREEN 3

    /* USER CODE END PD */

    ///*Private function-----------------------------------------------------------*/
    ///*toggle port*/
    // void toggle_port(GPIO_TypeDef *port, uint16_t pin)
    //{
    //	HAL_GPIO_TogglePin(port, pin);
    // }
    //
    ///*Public function------------------------------------------------------------*/
    ///*run traffic light counter*/
    // void run_traffic_light(TRAFFIC_LIGHT *light, STATE light_state)
    //{
    //	  switch (light_state)
    //	  {
    //	  case red:
    //		  if (light->red_time == 0)
    //		  {
    //			  toggle_port(light->LED_RED_Port, light->LED_RED_Pin);
    //			  light_state = green;
    //			  light->green_time = GREEN;
    //			  toggle_port(light->LED_GREEN_Port, light->LED_GREEN_Pin);
    //			  run_traffic_light(light, light_state);
    //		  }
    //		  else
    //		  {
    //			  light->red_time--;
    //		  }
    //		  break;
    //	  case green:
    //		  if (light->green_time == 0)
    //		  {
    //			  toggle_port(light->LED_GREEN_Port, light->LED_GREEN_Pin);
    //			  light_state = yellow;
    //			  light->yellow_time = YELLOW;
    //			  toggle_port(light->LED_YELLOW_Port, light->LED_YELLOW_Pin);
    //			  run_traffic_light(light, light_state);
    //		  }
    //		  else
    //		  {
    //			  light->green_time--;
    //		  }
    //		  break;
    //	  case yellow:
    //		  if (light->yellow_time == 0)
    //		  {
    //			  toggle_port(light->LED_YELLOW_Port, light->LED_YELLOW_Pin);
    //			  light_state = red;
    //			  light->red_time = RED;
    //			  toggle_port(light->LED_RED_Port, light->LED_RED_Pin);
    //			  run_traffic_light(light, light_state);
    //		  }
    //		  else
    //		  {
    //			  light->yellow_time--;
    //		  }
    //	  	  break;
    //	  }
    // }

#ifdef __cplusplus
}
#endif
#endif
