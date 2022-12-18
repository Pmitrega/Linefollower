#include "main.h"

#define ENCODER_LEFT TIM4->CNT
#define ENCODER_RIGHT TIM3->CNT
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
int set_velocity(enum Motor mot, uint16_t value);

/**
  * @brief  Run PWM test form 0 to 1000 for engine.
  * prints PWM and enc_value on uart.
  */
void test_engine(enum Motor mot);
void test_step_engine(enum Motor mot);
int calc_velocity(uint16_t last_enc_reading, uint16_t enc_reading);