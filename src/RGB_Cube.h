#ifndef RGB_CUBE_H
#define RGB_CUBE_H


// Configure the cube here, which hardware is used?
#define RGB_CUBE_4
//#define RGB_CUBE_8

// Architecture
#define RGB_CUBE_TLC59116  // I2C 
//#define RGB_CUBE_TLC5949 

#ifdef RGB_CUBE_TLC59116
	#include "I2C_parallelBus.h"
#endif

#ifdef RGB_CUBE_8
	#undef RGB_CUBE_4
#endif
#if !defined(RGB_CUBE_4) && !defined(RGB_CUBE_8)
	#error "Cubesize is not defined. define RGB_CUBE_8 or RGB_CUBE_3"
#endif

#endif