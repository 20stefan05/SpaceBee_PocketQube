#include "LED.h"

#define LED_GPIO_PORT GPIOB
#define LED_GPIO_PIN GPIO_PIN_3
#define ON SET
#define OFF RESET


void LED_TurnOn(void){
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, ON);
}

void LED_TurnOff(void){
    HAL_GPIO_WritePin(LED_GPIO_PORT, LED_GPIO_PIN, OFF);
}

void LED_Toggle(void){
    HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_GPIO_PIN);
}

void LED_Blink(int delay, int times){
    while(times--){
        LED_Toggle();
        HAL_Delay(delay);
    }
}
