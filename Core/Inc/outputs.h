/*
 * outputs.h
 *
 *  Created on: Nov 12, 2020
 *      Author: Mati
 */

#ifndef INC_OUTPUTS_H_
#define INC_OUTPUTS_H_

#include "main.h"
#include "stdlib.h"

typedef enum {
	Left, Right, Stop
} Direction_t;



/* exported functions  */

void setMotors(int8_t valueA, int8_t valueB);
void setMotorsDirection(Direction_t dirA, Direction_t dirB);
void setMotorSpeedA(uint8_t speedA);
void setMotorSpeedB(uint8_t speedB);
void setMotorsSpeed(uint8_t speedA, uint8_t speedB);

#endif /* INC_OUTPUTS_H_ */
