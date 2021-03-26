/*
The communication and image processing for robotic arm project
Created by Zien Zheng
Athlone Institue of Technology
*/
#include "esp_camera.h"
#include <WiFi.h>


// CAMERA_MODEL_AI_THINKER
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

const char *ssid = "AAA";
const char *pw="1472580369";

int port = 21222;
WiFiServer server(port);

bool wifiConnected;
char recvBuff[1024]={0};
int flag;//1.Stream
         //2.Picture

bool initWIFI();
bool captureImg(WiFiClient client);

void setup() {
  // put your setup code here, to run once:
  delay(2000);
  Serial.begin(115200);
  if(initWIFI()){
    wifiConnected=true;
    Serial.print("\nConnected,IP:");
    Serial.println(WiFi.localIP());
  }
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_VGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("\nCamera init failed with error 0x%x\n", err);
    return;
  }
  else{
    Serial.println("\nCamera init done.");
  }
  server.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  WiFiClient client = server.available();
  
  flag=-1;//Stream or take a picture
  if(client){
    Serial.println("Connected to PC...");
    while(client.connected()){
      int index = 0;
      if(client.available()>0){//receive message from PC
        while(client.available()>0){
          recvBuff[index]=client.read();
          index++;
        }
        int rlt = strcmp(recvBuff,"stream");
        if(rlt==0){
          flag = 1;
        }
        else {
         rlt = strcmp(recvBuff,"picture");
            if(rlt==0){
              flag = 2;
            }
        }
      }
      else if(flag==-1)
      {
        continue;
      }
       
      switch(flag){
        case 1://Recording
        if(!captureImg(client)){
          Serial.println("failed to send image");
        }
        break;
        case 2://Taking picture
        if(!captureImg(client)){
          Serial.println("failed to send image");
        }
        flag = -1;
        client.stop();
        break;
      }
    }
  }
  client.stop();
}

bool initWIFI()
{
  int connAttempts = 0;
  Serial.println("\nConnecting to: " + String(ssid));
  WiFi.begin(ssid, pw);
  while (WiFi.status() != WL_CONNECTED ) {
    delay(500);
    Serial.print(".");
    if (connAttempts > 20) {
    return false;
    }
    connAttempts++;
  }
  return true;
}

bool captureImg(WiFiClient client){
  if(client.connected()){
    Serial.println("Taking picture...");
  }
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  fb = esp_camera_fb_get();
  if(!fb){
    Serial.println("Failed to take picture");
    return false;
  }
  else{
    int fileSize=fb->len;
    String str("size:");
    str.concat(fileSize);
    Serial.println(str);
    client.write(str.c_str(),str.length());
    Serial.println("Successfully take the picture.");
    Serial.printf("Width:%d\n",fb->width);
    Serial.printf("Height:%d\n",fb->height);
    Serial.printf("Size of buffer:%d\n",sizeof(fb->buf));
    client.write((const char *)fb->buf,fb->len);
   }
   if(fb!=NULL){
    esp_camera_fb_return(fb);
   }
   return true;
}
