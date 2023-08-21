#include "BUZZER.h"

#define BUZZER_GPIO_PORT GPIOA
#define BUZZER_GPIO_PIN GPIO_PIN_8
#define ON SET
#define OFF RESET


void BUZZER_TurnOn(void){
    HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, ON);
}

void BUZZER_TurnOff(void){
    HAL_GPIO_WritePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN, OFF);
}

void BUZZER_Toggle(void){
    HAL_GPIO_TogglePin(BUZZER_GPIO_PORT, BUZZER_GPIO_PIN);
}

void BUZZER_Beep(int delay, int times){
    while(times--){
        BUZZER_Toggle();
        HAL_Delay(delay);
    }
}
