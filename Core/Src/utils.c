/*
 * utils.c
 *
 *  Created on: 10 lis 2020
 *      Author: Mati
 */

int range(int current, int change) {
	int temp = current + change;
	if (temp > 360) {
		return temp - 360;
	} else if (temp < 0) {
		return 361 + temp;
	}
	return temp;
}

int keepInRange(int value, int min, int max) {
	if (value < min) {
		value = min;
	} else if (value > max) {
		value = max;
	}
	return value;
}

//https://www.arduino.cc/en/Tutorial/LibraryExamples/RobotCompass
int calc_diff(int current, int wanted) {
	int diff = current - wanted;

	// modify degress

	if (diff > 180) {

		diff = -360 + diff;

	} else if (diff < -180) {

		diff = 360 + diff;

	}

	return diff;

}

void remove_char_from_string(char *str, char c)
{
    int i=0;
    int len = strlen(str)+1;

    for(i=0; i<len; i++)
    {
        if(str[i] == c)
        {
            // Move all the char following the char "c" by one to the left.
            strncpy(&str[i],&str[i+1],len-i);
        }
    }
}

int prefix(char *str, char *pre)
{
    return strncmp(pre, str, strlen(pre)) == 0;
}

