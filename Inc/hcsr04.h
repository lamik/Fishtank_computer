/*
 * hcsr04.h
 *
 *  Created on: 4 gru 2017
 *      Author: Mateusz Salamon
 */

#ifndef HCSR04_H_
#define HCSR04_H_

//###################################################################################

#define	_HCSR04_TIMER			htim3 // 1us tick configured in CubeMX
#define _HCSR04_TIMEOUT			1000000
#define _HCSR04_CONST			((float)0.0171821)
//###################################################################################

uint8_t HCSR04_Init();
float HCSR04_Read();

#endif /* HCSR04_H_ */
