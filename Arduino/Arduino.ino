/*
The communication and image processing for robotic arm project
Created by Zien Zheng
Athlone Institue of Technology
*/
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <SoftwareSerial.h>

#define SERVOMIN  102 // this is the 'minimum' pulse length count (out of 4096)
#define SERVOMAX  512 // this is the 'maximum' pulse length count (out of 4096)
#define RED 1
#define BLACK 2
#define YELLOW 3

SoftwareSerial s(5,6);//RX,TX
int LED=9;//red
int LED2=10;//yellow
int LED3=11;//white
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

typedef struct position_info{
  byte flag;//showing the status of ESP8266
           //1.Connecting to wifi.
           //2.Connected to wifi.
           //3.Connected to PC
           //4.Disconnected
           //5.Move the arm
  byte colour;//Type of colour
  byte count;//counting the mission times.
  byte correction;//Correction value
  float pos[6];//The angle movement setting for each motor.
}pInfo;

//Record the angle that expected to move
double movePos[6]={0};
//Record the current position
double currPos[6]={0};
//Store the angle for storage
double storePos[6]={0};
//Define a auxiliary point
double auxPos[6]={0};

//The angle information for each colour
double redPos[6]={180,70,150,40,0,180};
double blackPos[6]={120,20,115,80,60,180};
double yellowPos[6]={180,70 ,120,80,60,180};

//echo
char Echo[]="NEXT";

//Mapping the angle to the PWM format
bool setAngle(double *p,float a1, float a2, float a3, float a4, float a5, float a6){
  //The rotation range for motor 1 is 270 deg, and for rest of motor is 180 deg.
  p[0] = map(a1,0,270,SERVOMIN,SERVOMAX);
  p[1] = map(a2,0,180,SERVOMIN,SERVOMAX);
  p[2] = map(a3,0,180,SERVOMIN,SERVOMAX);
  p[3] = map(a4,0,180,SERVOMIN,SERVOMAX);
  p[4] = map(a5,0,180,SERVOMIN,SERVOMAX);
  p[5] = map(a6,0,180,SERVOMIN,SERVOMAX);
  for (int t = 0; t<6; t++){
    if (p[t]>SERVOMAX){
      return false;
    }
  }
  return true;
}

//Based on the received result, set the store position
bool setStoragePos(int colour){
  bool rlt=false;
  memset(storePos,0,sizeof(double)*6);
  do{
    if(colour==RED){
      memcpy(storePos,redPos,sizeof(double)*6);
    }
    else if(colour==BLACK){
      memcpy(storePos,blackPos,sizeof(double)*6);
    }
    else if(colour==YELLOW){
      memcpy(storePos,yellowPos,sizeof(double)*6);
    }
    else{
      break;
    }
    rlt=true;
  }while(false);
  return rlt;
}

//Moving to the expected position
void movingToDst(double *initPos,double *dstPos){
  Serial.println("Start moving");
  //Find the maximum moving range
  double maxRange=-1;
  for(int i=0;i<5;i++){
    double t=abs(dstPos[i]-initPos[i]);
    //Serial.print(dstPos[i]);
    //Serial.print("-");
    //Serial.print(initPos[i]);
    //Serial.print("=");
    //Serial.println(t);
    if(t>maxRange){
      maxRange=t;
    }
  }
  //Variable for saving the current position
  double cPos[6]={0};
  memcpy(cPos,initPos,sizeof(double)*6);
  //Start to move
  int range=(int)maxRange;//Convert to integer
  //Serial.print("Maximum looping range:");
  //Serial.println(range);
  for(uint16_t pulselen = SERVOMIN;pulselen<SERVOMIN+range;pulselen++){
    //The control of gripper will be last step
    delay(3);
    for(int n=0;n<5;n++){
      //Judge if the current position was reached the aimed angle
      if(abs(cPos[n]-dstPos[n])>1){
        if(dstPos[n]-cPos[n]>1){
          //Rotate in positive direction
          cPos[n]++;
          pwm.setPWM(n, 0,cPos[n]);
        }
        else if(dstPos[n]-cPos[n]<1){
          //Rotate in negative direction
          cPos[n]--;
          pwm.setPWM(n, 0,cPos[n]);
        }
        
      }
      else{
        delay(3);
      }
    }
  }
  
  //Make sure that all the motor exclude gripper rotate to the expected position
  for(int i=0;i<5;i++){
    pwm.setPWM(i, 0, dstPos[i]);
  }

  //Control gripper
  if(dstPos[5]==SERVOMIN){
    //Angle is 0 deg. Close the gripper
    pwm.setPWM(5, 0, SERVOMIN);
  }
  else{
    //Open the gripper
    pwm.setPWM(5, 0, SERVOMAX);
  }
}

//Go to auxiliary point
void moveToAux(double *initPos,double *dstPos){
  memset(auxPos,0,sizeof(double)*6);
  memcpy(auxPos,dstPos,sizeof(double)*6);
  //To the auxiliary point
  auxPos[1]=239;
  movingToDst(initPos,auxPos);
  //Update the position status
  memcpy(currPos,auxPos,sizeof(double)*6);
  return;
}


//Pick up the object
void pickUpObj(double *initPos,double *dstPos){
  //Go to the auxiliary point first
  //moveToAux(initPos,dstPos);
  
  //Move to the destination position
  movingToDst(currPos,dstPos);
  
  //Update the position status
  memcpy(currPos,dstPos,sizeof(double)*6);
  memset(movePos,0,sizeof(double)*6);
  return;
}

void releaseObj(double *initPos,double *dstPos){
  //Go to the auxiliary point first
  moveToAux(initPos,dstPos);
  
  //Move to the destination position
  movingToDst(currPos,dstPos);
  
  //Update the position status
  memcpy(currPos,dstPos,sizeof(double)*6);
  memset(storePos,0,sizeof(double)*6);
  memset(movePos,0,sizeof(double)*6);
  return;
}

bool ClassifyObj(float *pos){
  bool rlt=false;
  //Reset the expected position (the PWM for each motor for picking up object)
  memset(movePos,0,sizeof(double)*6);
  do{
  //Map the angle to PWM first
  if(!setAngle(movePos,pos[0],pos[1],pos[2],pos[3],pos[4],pos[5])){
    break;//Error
  }
  else{
    Serial.print("For each motor, to pick up the object, the angle is set as:");
      for(int t=0;t<6;t++){
        Serial.print(movePos[t]);
        Serial.print(",");
      }
   }
  Serial.print("\n");
  
  Serial.println("Picking up the object");
  //Pick up the object
  pickUpObj(currPos,movePos);
  delay(5000);
  Serial.println("Moving to the storage");
  
  //Release the object to the correct position depend on the colour of the object
  releaseObj(currPos,storePos);
  delay(5000);
  rlt = true;  
  }while(false);
  return rlt;
}

//Return to initial position
void rtnToInitialPos(double *initPos){
  double dstPos[6]={0}; 
  setAngle(dstPos,0,0,0,0,0,0); //Set the initial angle
  //Go to the auxiliary point first
  moveToAux(initPos,dstPos);
  //Move to the destination position
  movingToDst(currPos,dstPos);
  
  //Update the position status
  memcpy(currPos,dstPos,sizeof(double)*6);
  memset(movePos,0,sizeof(double)*6);
  return;
}
//Determine if the object location is safe to go or is reachable
bool safeToMove(float *pos){
  for(int i=0;i<6;i++){
    if(pos[i]<0 || (i!=0 && pos[i]>180)){
      return false;
    }
  }
  return true;
}

void setup() {
  s.begin(115200);
  pinMode(LED,OUTPUT);
  pinMode(LED2,OUTPUT);
  pinMode(LED3,OUTPUT);
  Serial.begin(115200);
  pwm.begin();
  pwm.setPWMFreq(50);  //Set the frequency to 50Hz
  setAngle(currPos,0,0,0,0,0,0); //Set the initial angle
  setAngle(redPos,redPos[0],redPos[1],redPos[2],redPos[3],redPos[4],redPos[5]); //Set the angle
  setAngle(blackPos,blackPos[0],blackPos[1],blackPos[2],blackPos[3],blackPos[4],blackPos[5]); //Set the angle
  setAngle(yellowPos,yellowPos[0],yellowPos[1],yellowPos[2],yellowPos[3],yellowPos[4],yellowPos[5]); //Set the angle
}

void loop() {
  //Set the angle for each motor, these value will obtain from image processing
  float objAngle[6];
  int colour=-1;
  int taskNum=0;
  int taskType=-1;
  //-------------------
  byte recvBuff[sizeof(pInfo)]={0};
  pInfo msg;
  if(msg.flag==5){
    msg.flag=0;
  }
  if(s.available()>0){
    memset(&msg,0,sizeof(msg));
    for (int i = 0;i<sizeof(pInfo);i++){
      recvBuff[i]=s.read();
      delay(20);
    }
    Serial.println("......................................\nReading data:");
    memcpy(&msg,recvBuff,sizeof(msg));
    Serial.println("Data received");
    Serial.print("Flag status:");
    Serial.println(msg.flag);
    Serial.print("Colour type:");
    switch(msg.colour){
      case 1:
      Serial.println("Red");
      break;
      case 2:
      Serial.println("Black");
      break;
      case 3:
      Serial.println("Yellow");
      break;
      default:
      Serial.println("Unknow");
    }
    
    
    Serial.print("Count status:");
    Serial.println(msg.count);
    Serial.print("Position:");
    for(int i=0;i<6;i++){
        Serial.print(msg.pos[i]);
        Serial.print(",");
    }
    Serial.println("\n............................\n");
  }
  //showing the status of ESP8266
  //1.Connecting to wifi.
  //2.Connected to wifi.
  //3.Connected to PC
  //4.Disconnected
  //5.Move the arm
  switch(msg.flag){
    case 1://blinking red, yellow, white.(Connecting to wifi.)
      digitalWrite(LED,HIGH);
      digitalWrite(LED2,HIGH);
      digitalWrite(LED3,HIGH);
      delay(400);
      digitalWrite(LED,LOW);
      digitalWrite(LED2,LOW);
      digitalWrite(LED3,LOW);
      delay(400); 
      break;
    case 2://blinking red,yellow.(Connected to wifi.)
      digitalWrite(LED,HIGH);
      digitalWrite(LED2,HIGH);
      delay(400);
      digitalWrite(LED,LOW);
      digitalWrite(LED2,LOW);
      delay(400); 
      break;
    case 3://Blinking red, white(Connected to PC)
      digitalWrite(LED,HIGH);
      digitalWrite(LED3,HIGH);
      delay(400);
      digitalWrite(LED,LOW);
      digitalWrite(LED3,LOW);
      delay(400); 
      break;
    case 4://Disconnected
      digitalWrite(LED,HIGH);
      delay(200);
      digitalWrite(LED,LOW);
      digitalWrite(LED2,HIGH);
      delay(200);
      digitalWrite(LED2,LOW);
      digitalWrite(LED3,HIGH);
      delay(200);
      digitalWrite(LED3,LOW);
      delay(200); 
      break;
    case 5://turn on white LED 
      digitalWrite(LED3,HIGH);
      break;
    default://Unknow error,waiting for next command,yellow LED turn on.(flag=0)
      digitalWrite(LED,LOW);
      digitalWrite(LED3,LOW);
      digitalWrite(LED2,HIGH);
      delay(200);
      digitalWrite(LED2,LOW);
      break;
  }

  taskType=msg.flag;
  //If type=5, then start pickings
  if(taskType==5){
    memcpy(objAngle,msg.pos,sizeof(float)*6);
    colour=msg.colour;
    taskNum=msg.count;
    if(!safeToMove(objAngle)){
      Serial.println("the position of the object located cannot reach!");
      s.write(Echo,sizeof(Echo));
      return;
    }
    //Set the expected position via colour type
    if(!setStoragePos(colour)){
      Serial.println("ERROR! Failed to recognize the type of the object!");
    }
    //Move to the object and pick up the object.
    Serial.print("Task ");
    Serial.print(taskNum);
    Serial.println(", start");
    ClassifyObj(objAngle);
    Serial.println("Task finished, return to initial position");
    //Return to the initial position
    rtnToInitialPos(currPos);
    Serial.println("Waiting for next task");
    
    s.write(Echo,sizeof(Echo));
  }
}
