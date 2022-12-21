#include "sensors.h"
#include "main.h"

uint16_t sensor_readings[8];
uint16_t sensors_white[8] = {SENSOR0_WHITE, SENSOR1_WHITE,SENSOR2_WHITE, SENSOR3_WHITE,SENSOR4_WHITE, SENSOR5_WHITE,SENSOR6_WHITE, SENSOR7_WHITE};
uint16_t sensors_black[8] = {SENSOR0_BLACK,SENSOR1_BLACK,SENSOR2_BLACK,SENSOR3_BLACK,SENSOR4_BLACK,SENSOR5_BLACK,SENSOR6_BLACK,SENSOR7_BLACK};
extern BASE_SPEED;
int estimate_angle(uint16_t* sensors){
    float angle_sum = 0;
    static int last_angle;
    int angle = 0;
    int total_detection = 0;    
    for(int i =0; i<8; i++){
        if(sensors[i] > (sensors_white[i] + SENSOR_THRE)){
            angle_sum += (float)(i - 4)*DIFF_PER_SENSOR;
            total_detection +=1;
        };
    }
    if(total_detection == 0){
        angle = last_angle;
        return angle;
    }
    else{
    angle = (int)(angle_sum/total_detection);
    last_angle = angle;
    return angle;
    }
}