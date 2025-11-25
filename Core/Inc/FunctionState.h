/*
 * FunctionState.h
 *
 *  Created on: Nov 17, 2025
 *      Author: gbara
 */

#ifndef INC_FUNCTIONSTATE_H_
#define INC_FUNCTIONSTATE_H_

#include <stdint.h>   // for uint32_t

void Starth(uint32_t* counter);
void TransH();
void TapsH(volatile uint32_t* counter);
void RotH(volatile uint32_t* counter);
void EndH(void);


#endif /* INC_FUNCTIONSTATE_H_ */
