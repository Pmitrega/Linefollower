#include "motors.h"
#include "string.h"
#include "stdio.h"
#include "main.h"
extern UART_HandleTypeDef huart1;
extern int16_t velocity_LEFT;
extern int16_t velocity_RIGHT;
int set_PWM(enum Motor mot, uint16_t value){
    if(value>1000){
        return -1;
    }
    if(mot == MOTOR_LEFT){
        TIM11->CCR1 = value;
    }
    else if (mot == MOTOR_RIGHT)
    {
        TIM10->CCR1 = value;
    }
    return 0;
    
}
int set_direction(enum Motor mot,enum Direction dir){

    if(mot == MOTOR_LEFT){
        if(dir == BACKWARD){
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 1);
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN2_Pin, 0);
        }
        else if(dir == FORWARD){
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN1_Pin, 0);
            HAL_GPIO_WritePin(AIN1_GPIO_Port, AIN2_Pin, 1);
        }
    }
    else if(mot == MOTOR_RIGHT){
        if(dir == BACKWARD){
            HAL_GPIO_WritePin(AIN1_GPIO_Port, BIN1_Pin, 1);
            HAL_GPIO_WritePin(AIN1_GPIO_Port, BIN2_Pin, 0);
        }
        else if(dir == FORWARD){
            HAL_GPIO_WritePin(AIN1_GPIO_Port, BIN1_Pin, 0);
            HAL_GPIO_WritePin(AIN1_GPIO_Port, BIN2_Pin, 1);
        }
    }
    return 0;
};

void test_engine(enum Motor mot){
    uint16_t pwm_value = 0;
    static char buffer[20];
    int l;
    set_direction(mot, FORWARD);
    for(int i = 0; i<100; i+=1){
        set_PWM(mot, pwm_value);
        l = sprintf(buffer, "%d %d \n",pwm_value,mot == MOTOR_LEFT ? velocity_LEFT:velocity_RIGHT);
        HAL_UART_Transmit(&huart1, buffer, l, 100);
        pwm_value+=10;
        HAL_Delay(500);
    } 
    set_PWM(mot, 0);
}
int calc_velocity(uint16_t last_enc_reading, uint16_t enc_reading);
int set_velocity(enum Motor mot, uint16_t value){

};

void test_step_engine(enum Motor mot){
    uint16_t pwm_value = 1000;
    static char buffer[20];
    int l;
    set_direction(mot, FORWARD);
    set_PWM(mot, pwm_value);
    set_direction(MOTOR_RIGHT, FORWARD);
    set_PWM(MOTOR_RIGHT, pwm_value);
    for(int i = 0; i<100; i+=1){
        l = sprintf(buffer, "%d %d \n",0,mot == MOTOR_LEFT ? velocity_LEFT:velocity_RIGHT);
        HAL_UART_Transmit(&huart1, buffer, l, 100);
        HAL_Delay(2);
    } 
    set_PWM(mot, 0);
    set_PWM(MOTOR_RIGHT, 0);
}
int calc_velocity(uint16_t last_enc_reading, uint16_t enc_reading);