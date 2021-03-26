#pragma once
#include <winsock2.h>
#include <windows.h>
#include <iostream>
#include <stdio.h>
#include <string>
#include <cstring>
#include "struInfo.h"
#include "ColourProc.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <vector>


#pragma comment(lib, "ws2_32.lib")

SOCKET esp8266Sock;         //Socket for TCP connection with esp8266
SOCKET esp32Sock;           //Socket for TCP connection with esp32-cam
char esp8266IpAddr[256] = { 0 };   //Store ESP8266 IP address
char esp32IpAddr[256] = { 0 };      //Store ESP32 IP address
bool recvImgFlag = false;			//The flag for judge whether need to receive image.
bool coObtained = false;			//Obtained the coordination

const int esp8266Port = 22222;      //Assign the port for communication with esp8266
const int esp32Port = 21222;        //Assign the port for communication with esp32
HANDLE hThreadE8;        //Thread handle for esp8266 communication.
HANDLE hThreadE3;        //Thread handle for esp32 communication.
struct sockaddr_in esp8266Addr;		//ESP8266 address info.
struct sockaddr_in esp32Addr;		//ESP32 address info.
int comStatus;						//Show the communication status.

vector<pInfo> vRed;		//store the processing result of red
vector<pInfo> vBlack;	//store the processing result of black
vector<pInfo> vYellow;	//store the processing result of yellow


bool initALL();				//Initialize WSADATA and Socket.
bool Connection(SOCKET* sock, struct sockaddr_in* addr, int port, char* ip);
//Connecting to other device.

bool initALL() {
	bool bRst = false;		//Return value of this function.
	do {
		WSADATA wVersion;
		int flag;
		flag = WSAStartup(MAKEWORD(2, 2), &wVersion);
		if (flag)
		{
			break;			//Initializing failed
		}
		esp8266Sock = socket(AF_INET, SOCK_STREAM, NULL);	//TCP 
		esp32Sock = socket(AF_INET, SOCK_STREAM, NULL);		//TCP
		if (esp8266Sock == INVALID_SOCKET || esp32Sock == INVALID_SOCKET)
		{
			break;			//Initializing failed
		}
		bRst = true;
	} while (false);
	return bRst;
}

bool Connection(SOCKET* sock, struct sockaddr_in* addr, int port, char* ip) {
	bool bRst = false;		//Return value of this function.
	do {
		(*addr).sin_family = AF_INET;
		(*addr).sin_port = htons(port);
		(*addr).sin_addr.S_un.S_addr = inet_addr(ip);
		int flag;
		flag = connect(*sock, (sockaddr*)addr, sizeof(*addr));
		if (flag == SOCKET_ERROR)
		{
			break;
		}
		bRst = true;
	} while (false);
	return bRst;
}
