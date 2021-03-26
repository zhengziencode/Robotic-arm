/*
The communication and image processing for robotic arm project
Created by Zien Zheng
Athlone Institue of Technology
*/
#include <ESP8266WiFi.h>

int port = 22222;
WiFiServer server(port);

const char *ssid = "AAA";
const char *password = "1472580369";

typedef struct position_info{
  byte flag;//showing the status of ESP8266
           //1.Connecting to wifi.
           //2.Connected to wifi.
           //3.Connnected to PC
           //4.Disconnected.
           //5.Move the arm
  byte type;//Type of colour
  byte count;//counting the mission times.
  float pos[6];//The angle movement setting for each motor.
}pInfo;//bytes

pInfo msg;
char sendBuff[sizeof(msg)]={0};
char recvBuff[sizeof(msg)]={0};

void setup() {
  delay(6000);
  Serial.begin(115200);
  memset(&msg,0,sizeof(msg));
  
  //---------------
  msg.flag = 1;
  memcpy(sendBuff,&msg,sizeof(msg));
  Serial.write(sendBuff,sizeof(pInfo));
  //Serial.println("connecting\nflag status:");
  //Serial.println(msg.flag);
  //----------------------
  delay(500);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid,password);
  while(WiFi.status()!=WL_CONNECTED){
    delay(500);
  }

  //connected to wifi
  msg.flag = 2;
  memset(sendBuff,0,sizeof(msg));
  memcpy(sendBuff,&msg,sizeof(msg));
  //for(int i=0;i<sizeof(pInfo);i++){
  Serial.write(sendBuff,sizeof(pInfo));
  //}
  //Serial.println("Connected!");
  //Serial.print("flag status:");
  //Serial.println(msg.flag);
  server.begin();
}

void loop() {
  memset(&msg,0,sizeof(msg));
  memset(sendBuff,0,sizeof(msg));
  
  WiFiClient client = server.available();

  if(client){
    if(client.connected()){
      msg.flag = 3;
      memcpy(sendBuff,&msg,sizeof(msg));
      //for(int i=0;i<sizeof(msg);i++){
        Serial.write(sendBuff,sizeof(pInfo));
     //}
      //Serial.print("flag status:");
      //Serial.println(msg.flag);
    }
    while(client.connected()){
      while(client.available()>0){
        for(int i=0;i<sizeof(pInfo);i++){
          recvBuff[i]=client.read();
          //Serial.write(client.read());//write the data received from PC send to arduino
        }
        //for(int i=0;i<sizeof(pInfo);i++){
          Serial.write(recvBuff,sizeof(pInfo));
          //Serial.print("Sizeof struct:");
          //Serial.println(sizeof(pInfo));
          
        //}
        /*pInfo recvInfo;
        memset(&recvInfo,0,sizeof(recvInfo));
        memcpy(&recvInfo,recvBuff,sizeof(recvBuff));
        //test start
        Serial.print("\nflag status:");
        Serial.println(recvInfo.flag);
        Serial.print("Position:");
        for(int i=0;i<6;i++){
          Serial.print(recvInfo.pos[i]);
          Serial.print(",");
        }
        //test end*/
      }
      while(Serial.available()>0){
        client.write(Serial.read());
      }
    }
    client.stop();
    memset(&msg,0,sizeof(msg));
    memset(sendBuff,0,sizeof(msg));
    msg.flag = 4;
    memcpy(sendBuff,&msg,sizeof(msg));
    //for(int i=0;i<sizeof(msg);i++){
    Serial.write(sendBuff,sizeof(pInfo));
    //}
  }
  delay(500);
}
