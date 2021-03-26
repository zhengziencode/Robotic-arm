#pragma once
#include <math.h>
//The default height 
#define DEFAULT_H 49
#define CAMERA_H 110
#define PI	3.1416
//The value of angle alpha was set to 30 degrees
const float ALPHA = 30;

//For the further extension purpose, a class is created rather than a function
class svAng {
private:
	float angle[6];	//The result angle;
					//corresponding to the component of:
					//1.waist
					//2.shoulder
					//3.elbow
					//4.wrist
					//5.tip
					//6.open and close of gripper
	float roAngle[6];//The real angle needed for the motor to rotate
protected:
	float x;
	float y;
	float z;
	float link[3];
	bool rlt;
public:
	svAng(float x_x, float y_y, float z_z) :x(x_x), y(y_y), z(z_z), rlt(false) { memset(angle, 0, sizeof(angle)); memset(link, 0, sizeof(link));memset(roAngle, 0, sizeof(roAngle));}
	svAng(float x_x, float y_y) :x(x_x), y(y_y), z(DEFAULT_H), rlt(false) { memset(angle, 0, sizeof(angle));memset(link, 0, sizeof(link));memset(roAngle, 0, sizeof(roAngle));}
	svAng() :x(0.0), y(0.0), z(DEFAULT_H), rlt(false) { memset(angle, 0, sizeof(angle)); memset(link, 0, sizeof(link));memset(roAngle, 0, sizeof(roAngle));}
	float roundTo3Decimal(float number);//Round the result to three decimal
	void coordination(float x, float y, float z);//for setting the coordination
	void coordination(float x, float y);//for setting the coordination
	void linkLength(float link_1, float link_2, float link_3);//Asssing the value of the links
	bool isSolved() { return this->rlt; }
	float* getAngle();
	float* getAngle4Rot();
	void solveAngle();//The thertical calculated angle
	void solveAngle4Joint();//The real rotation angle needed for motor
	~svAng();
};
float svAng::roundTo3Decimal(float number) {
	int integer = (int)((number) * 1000.0);
	float newValue = (float)(integer / 1000.0);
	return newValue;
}

/*
* Calculate the angle that the motor should move corresponding to the linked part.
*/
void svAng::solveAngle() {
	do {
		angle[5] = 0;//Close the gripper
		angle[4] = 0;//The tip will remain without rotate

		//The rotation angle for waist
		angle[0] = this->roundTo3Decimal((atan(y / x) * 180.0 / PI));
		//The rotation angle for elbow
		angle[2] = this->roundTo3Decimal((acos((pow(x, 2) + pow(z, 2) - pow(link[0], 2) - pow(link[1], 2)) / (2.0 * link[1] * link[0])) * 180.0 / PI));
		//The rotation angle for shoulder
		angle[1] = this->roundTo3Decimal((atan(((z * (link[0] + link[1] * cos(angle[2]))) - (x * (link[1] * sin(angle[2])))) 
									/ ((x * (link[0] + link[1] * cos(angle[2]))) + (z * (link[1] * sin(angle[2]))))) * 180.0 / PI));
		//The rotation angle for wrist, the angle of alpha is set to 30 degrees
		angle[3] = ALPHA - angle[1] - angle[2];

		this->rlt = true;
	} while (false);
	return;
}

void svAng::solveAngle4Joint() {
	if (this->rlt) {
		//Angle 1 remains
		roAngle[0] = angle[0];
		//angle 2 = abs(angle2)
		roAngle[1] = abs(angle[1]);
		/*
		Angle 3: shoulder
		if is positive
		rotation angle = angle 1-angle 2+90
		if is negative
		rotation angle = angle 1+90+abs(angle 2)
		*/
		if (angle[2] >= 0) {
			roAngle[2] = angle[1] - angle[2] + 90;
		}
		else {
			roAngle[2] = angle[1] + 90 + abs(angle[2]);
		}

		/*
		Angle 4: elbow
		if positive
		rotation angle = angle 3 + 90deg - angle 2
		if negative && |angle 3| smaller than 90deg
		rotation angle = 90deg - abs(angle 3)
		if negative && |angle 3| greater than 90deg
		rotation angle =  abs(angle 3)- 90deg
		*/
		if (angle[3] < 0 && abs(angle[3])>180) {
			angle[3] = 360 + angle[3];
		}
		if (angle[3] >= 0) {
			roAngle[3] = angle[3] + 90 - angle[2];
		}
		else if (angle[3] < 0) {
			roAngle[3] = 90- abs(angle[3])-angle[2];
		}

		/*
		* angle 4,5 remains
		*/
		roAngle[4] = angle[4];
		roAngle[5] = angle[5];
	}
	else{
		return;
	}
}

void svAng::coordination(float x, float y, float z) {
	this->x = x;
	this->y = y;
	this->z = z;
}

void svAng::coordination(float x, float y) {
	this->x = x;
	this->y = y;
	this->z = DEFAULT_H;
}

void svAng::linkLength(float link_1, float link_2, float link_3) {
	this->link[0] = link_1;
	this->link[1] = link_2;
	this->link[2] = link_3;
}

float* svAng::getAngle() {
	if (this->rlt) {
		return this->angle;
	}
	else
	{
		return nullptr;
	}
}

float* svAng::getAngle4Rot() {
	if (this->rlt) {
		return this->roAngle;
	}
	else
	{
		return nullptr;
	}
}

svAng::~svAng() {
	for (int i = 0;i < 6;i++) {
		angle[i] = -1;
		roAngle[i] = -1;
		x = y = z = -1;
		rlt = false;
	}
}

