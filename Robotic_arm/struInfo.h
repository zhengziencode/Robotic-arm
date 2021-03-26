#pragma once

#define COM_ERROR 1
#define COM_NORMAL 2
#define RED 1
#define BLACK 2
#define YELLOW 3
//The size scale for an object mapping to the image, and the shrink scale or streath scale.
float SCALE = -1;
//The real dimension for the diameter of the object, 10mm
float realDimension = 10;
//The dimension for the diameter of the object in pixels
float pixelDimension = 0;
//The area for one circle in pixels
float oneObjArea = 5500;


typedef struct position_info {
	byte flag;//The value will be set to 5.
	byte type;//1.red 2.black 3.yellow
	byte count;//counting the mission times.
	float pos[6];//The angle movement setting for each motor.
}pInfo;//28 bytes
