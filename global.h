/*
 * global.h
 *
 * Created: 2018/4/29 上午 10:11:32
 *  Author: Saki
 */ 

#ifndef _GLOBAL_H_
#define _GLOBAL_H_

#include <stdint.h>

// Type definitions
//  Shorten description
#define uchar       unsigned char
//  Boolean type supports.
#define BOOL        unsigned char
#define TRUE        1
#define FALSE       0

// Functions
//  Transfer a value to the bit value
#define M_BIT(x)            (1<<(x))
#define M_SET_BIT(v, x)     v |= M_BIT(x)
#define M_UNSET_BIT(v, x)   (v &= ~M_BIT(x))
#define M_IS_SET(v, x)      (v & M_BIT(x))

// Configurations
//#define MODE_ARDUINO
//#define KARMAN_ENABLE
// For 3213 prototype
//  CPU frequency.
//  3213 prototype board, 20MHz
//#define F_CPU 16000000

#endif /* _GLOBAL_H_ */
