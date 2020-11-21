/*
 * utils.h
 *
 *  Created on: 10 lis 2020
 *      Author: Mati
 */

#ifndef INC_UTILS_H_
#define INC_UTILS_H_

//#include <string.h>
#include <stdint.h>
#include <stdlib.h>


int range(int current, int change);
int keepInRange(int value, int min, int max);
int calc_diff(int current, int wanted);
void remove_char_from_string( char *str, char c);
int prefix(char *str, char *pre);

#endif /* INC_UTILS_H_ */
