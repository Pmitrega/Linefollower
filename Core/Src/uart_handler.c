#include "uart_handler.h"

extern UART_HandleTypeDef huart1;
extern int BASE_SPEED;
uint8_t command;
uint8_t num_cmd[10];
uint8_t num_mess[10];
uint8_t recieve;
extern int auto_control_flag;
void print_int(int number){
  int l;
  l = sprintf(num_mess, "\n%d\n", number);
  HAL_UART_Transmit_IT(&huart1, num_mess, l);
}
void handle_command(uint8_t cmd,int number){
  if(number > 300 && cmd == 't'){
    HAL_GPIO_TogglePin(GPIOB, LED_BLUE1_Pin);
  }
  if(cmd == 'p'){
    print_int(number);
  }
  if(cmd == 'a'){
    if(auto_control_flag == 0)auto_control_flag =1;
    else{ auto_control_flag = 0;set_velocity(MOTOR_LEFT,0);set_velocity(MOTOR_RIGHT,0);}
    BASE_SPEED = number;
    HAL_UART_Transmit_IT(&huart1, "CHANGED MODE", 13);
  }
  if(cmd == 'l'){
    if(!auto_control_flag)
        set_velocity(MOTOR_LEFT, number);
  }
  if(cmd == 'r'){
    if(!auto_control_flag)
        set_velocity(MOTOR_RIGHT, number);
  }
}
int num_ready = 0;
int num_pointer;
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  HAL_GPIO_WritePin(GPIOB, LED_BLUE2_Pin,1);
  if(recieve >= 'a' && recieve <= 'z'){
    num_ready = 1;
    command = recieve;
    num_pointer = 0;
  }
  else if(num_ready && ((recieve >= '0' && recieve <='9') || recieve == '-') && num_pointer < 10){
    num_cmd[num_pointer] = recieve;
    num_pointer++;
  }
  else if(num_pointer > 0 && num_ready == 1){
    num_cmd[num_pointer] = '\0';
    handle_command(command, atoi(num_cmd));
    num_ready = 0;
  }
  else{
    num_ready = 0;
  }
  HAL_UART_Receive_IT(&huart1, &recieve, 1);
}