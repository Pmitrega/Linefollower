#include "motors.h"
#include "string.h"
#include "stdio.h"
#include "main.h"
extern UART_HandleTypeDef huart1;
extern int velocity_LEFT;
extern int velocity_RIGHT;
int desired_velocity_left;
int desired_velocity_right;
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
int set_control(enum Motor mot, int16_t value){
    //SATURATION
    if(value > 1000)
        value = 1000;
    if(value < -1000)
        value = -1000;

    if (value < 0)
        set_direction(mot, BACKWARD);
    else
        set_direction(mot, FORWARD);
    if(value < 0)
        value = -value;
    
    set_PWM(mot, value);

};

void test_step_engine(enum Motor mot){
    uint16_t pwm_value = 1000;
    char buffer[20];
    int l;
    set_direction(mot, FORWARD);
    set_PWM(mot, pwm_value);
    set_direction(MOTOR_RIGHT, FORWARD);
    set_PWM(MOTOR_RIGHT, pwm_value);
    for(int i = 0; i<100; i+=1){
        l = sprintf(buffer, "%d %d \n",velocity_LEFT,velocity_RIGHT);
        HAL_UART_Transmit(&huart1, buffer, l, 100);
        HAL_Delay(4);
    } 
    set_PWM(MOTOR_LEFT, 0);
    set_PWM(MOTOR_RIGHT, 0);
}

void print_encoders(){
    int lenght;
    char buffer[20];
    lenght = sprintf(buffer, "%d %d\n",ENCODER_LEFT, ENCODER_RIGHT);
    HAL_UART_Transmit(&huart1, buffer, lenght, 100);
}
void pid_left(int error, int point_of_work){
static int last_point_of_work = 0;
    static int error_integral = 0;
    if(last_point_of_work != point_of_work){
        error_integral = 0;
        error = 0;
    }
    last_point_of_work = point_of_work;
    error_integral+= error;
    if(error_integral > 200)error_integral =200;
    if(error_integral < -200)error_integral = -200;
    int base_value = point_of_work * 4;
    float P = 5;
    float I = 1;
    set_control(MOTOR_LEFT, base_value + error * P+ error_integral * I);
}
void pid_right(int error, int point_of_work){
    static int last_point_of_work = 0;
    static int error_integral = 0;
    if(last_point_of_work != point_of_work){
        error_integral = 0;
        error = 0;
    }
    last_point_of_work = point_of_work;
    error_integral+= error;
    if(error_integral > 200)error_integral =200;
    if(error_integral < -200)error_integral = -200;
    int base_value = point_of_work * 4;
    float P = 5;
    float I = 1;
    set_control(MOTOR_RIGHT, base_value + error * P+ error_integral * I);
}
void velocity_pid(){
    pid_left(desired_velocity_left - velocity_LEFT, desired_velocity_left);
    pid_right(desired_velocity_right - velocity_RIGHT, desired_velocity_right);
}

void set_velocity(enum Motor mot, int desired_velocity){
    if(mot == MOTOR_RIGHT){
        desired_velocity_right = desired_velocity;
    }
    if(mot == MOTOR_LEFT){
        desired_velocity_left = desired_velocity;
    }
}