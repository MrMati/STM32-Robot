/*
 * outputs.c
 *
 *  Created on: Nov 12, 2020
 *      Author: Mati
 */

#include "outputs.h"

void setMotorDirA(Direction_t dirA) {
	if (dirA == Right) {
		HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_RESET);
	} else if (dirA == Left) {
		HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(A1_GPIO_Port, A1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(A2_GPIO_Port, A2_Pin, GPIO_PIN_RESET);
	}
}

void setMotorDirB(Direction_t dirB) {

	if (dirB == Right) {
		HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, GPIO_PIN_RESET);
	} else if (dirB == Left) {
		HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, GPIO_PIN_SET);
	} else {
		HAL_GPIO_WritePin(B1_GPIO_Port, B1_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(B2_GPIO_Port, B2_Pin, GPIO_PIN_RESET);
	}
}

void setMotorsDirection(Direction_t dirA, Direction_t dirB) {
	setMotorDirA(dirA);
	setMotorDirB(dirB);
}

void setMotorSpeedA(uint8_t speedA) {
	TIM3->CCR3 = keepInRange(speedA, 0, 100);
}

void setMotorSpeedB(uint8_t speedB) {
	TIM3->CCR4 = keepInRange(speedB, 0, 100);
}

void setMotorsSpeed(uint8_t speedA, uint8_t speedB) {
	setMotorSpeedA(speedA);
	setMotorSpeedB(speedB);
}

void setMotors(int8_t valueA, int8_t valueB) {
	if (valueA < 0) {
		setMotorDirA(Left);
	} else if (valueA > 0) {
		setMotorDirA(Right);
	} else {
		setMotorDirA(Stop);
	}
	if (valueB < 0) {
		setMotorDirB(Left);
	} else if (valueB > 0) {
		setMotorDirB(Right);
	} else {
		setMotorDirB(Stop);
	}
	setMotorsSpeed(abs(valueA), abs(valueB));
}

