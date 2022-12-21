#ifndef __MOTORS_H
#define __MOTORS_H
#include "main.h"
#define ENCODER_LEFT TIM4->CNT
#define ENCODER_RIGHT TIM3->CNT
#define IMPULSE_PER_ROTATION 270.f
#define MEASSUREMENT_PERIOD 0.050f
#define CONVERSION_COEFFICIENT (int)(60.f/IMPULSE_PER_ROTATION/MEASSUREMENT_PERIOD)
enum Motor{
    MOTOR_LEFT,
    MOTOR_RIGHT
};

enum Direction{
    FORWARD,
    BACKWARD
};
/**
  * @brief  Sets PWM on motor channel
  * @param  value PMW CCR, from 0 to 1000
  * @retval -1 if error value was to high else 0
  */
int set_PWM(enum Motor mot, uint16_t value);
/**
  * @brief  Sets motor direction
  * @retval -1 if error value was to high else 0
  */
int set_direction(enum Motor mot,enum Direction dir);

/**
  * @brief  Sets motor direction
  * @retval -1 if error value was to high else 0
  */
int set_control(enum Motor mot, int16_t value);

/**
  * @brief  Run PWM test form 0 to 1000 for engine.
  * prints PWM and enc_value on uart.
  */
void test_engine(enum Motor mot);
void test_step_engine(enum Motor mot);
void print_encoders();
void velocity_pid();
void set_velocity(enum Motor mot, int desired_velocity);
#endif