/**
 ******************************************************************************
 * @file    BSP/Src/mems.c
 * @author  MCD Application Team
 * @brief   This example code shows how to use MEMS features.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2017 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "mems.h"

/** @addtogroup STM32F4xx_HAL_Examples
 * @{
 */

/** @addtogroup BSP
 * @{
 */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Init af threshold to detect acceleration on MEMS */

/* Private function prototypes -----------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

float read_heading(void) {
	float sum = 0;
	int tries=0;
	for (int i = 0; i < SAMPLES_NUM_COMPASS; i++) {

		float reads[3] = { };

		while ((0x00000001 & LSM303DLHC_MagGetDataStatus()) == 0) {
			if(tries > 5) {
				return -1;
			}
			tries++;
		}
		BSP_COMPASS_GetXYZ(&reads);
		float heading = atan2(reads[1], reads[0]);
		if (heading < 0) {
			heading += 2 * M_PI;
		}

		if (heading > 2 * M_PI) {
			heading -= 2 * M_PI;
		}
		heading *= 180 / M_PI;
		sum += heading;
	}
	return sum / SAMPLES_NUM_COMPASS;
}

Vector read_accel(void) {
	int16_t buffer[3] = { 0 };
	BSP_ACCELERO_GetXYZ(buffer);

	Vector out;
	out.x = ((float) buffer[0] * 2.0) / (float) INT16_MAX;
	out.y = ((float) buffer[1] * 2.0) / (float) INT16_MAX;
	out.z = ((float) buffer[2] * 2.0) / (float) INT16_MAX;
	return out;
}

Vector read_gyro(void) {
	Vector sum;
	for (int i = 0; i < SAMPLES_NUM_COMPASS; i++) {
		float reads[3] = { };
		while ((0x00000001 & L3GD20_GetDataStatus()) == 0) {
		}
		BSP_GYRO_GetXYZ(reads);
		sum.x += reads[0];
		sum.y += reads[1];
		sum.z += reads[2];
	}
	sum.x /= SAMPLES_NUM_GYRO * 100;
	sum.y /= SAMPLES_NUM_GYRO * 100;
	sum.z /= SAMPLES_NUM_GYRO * 100;
	return sum;
}

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
