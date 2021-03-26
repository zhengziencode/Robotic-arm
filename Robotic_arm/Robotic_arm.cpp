/*
The communication and image processing for robotic arm project
Created by Zien Zheng
Athlone Institue of Technology
*/

#include "robotic_arm.h"

using namespace std;
using namespace cv;

DWORD WINAPI ProcessingESP8266(LPVOID lpPara);  //Create a thread for communicating with ESP8266
DWORD WINAPI ProcessingESP32(LPVOID lpPara);    //Create a thread for communicating with ESP32

int main()
{	
	cout <<"size of structure of the packet: " <<sizeof(position_info) << endl;
	//cout << sizeof(byte) << endl;
	//cout << sizeof(float) << endl;
	comStatus = COM_NORMAL;		//The commnunication is set to normal
	
	if (initALL()) {
		cout << "Initialize WSADATA, Socket done!" << endl;
	}
	else {
		cout << "Failed to initialize system!" << endl;
		return -1;
	}

	string ipStr;
    //Initializing config for communicating with ESP8266
	cout << "Please input the IP address of ESP8266:";
	cin >> ipStr;
	strcpy_s(esp8266IpAddr, ipStr.c_str());
	printf("The ESP8266 IP:%s\n", esp8266IpAddr);
	if (Connection(&esp8266Sock,&esp8266Addr,esp8266Port,esp8266IpAddr)) {
		cout << "Connected to ESP8266."<<endl;
	}
	else {
		cout << "Failed to connect to ESP8266." << endl;
		//return -1;
	}
    //Initializing config for communicating with ESP32-CAM
	cout << "Please input the IP address of ESP32:";
	cin >> ipStr;
	strcpy_s(esp32IpAddr,ipStr.c_str());
	printf("The ESP32-CAM IP:%s\n", esp32IpAddr);
	if (Connection(&esp32Sock,&esp32Addr,esp32Port,esp32IpAddr)) {
		cout << "Connected to ESP32."<<endl;
	}
	else {
		cout << "Failed to connect to ESP32." << endl;
		//return -1;
	}

    //Receiving the image/video stream from ESP32-CAM,
	//Image processing.(In this function, it will return a value with type of pInfo)
	hThreadE3 = ::CreateThread(nullptr, 0, ProcessingESP32, nullptr, 0, nullptr);

    //Configurate the data and send the position information to ESP8266
	hThreadE8 = ::CreateThread(nullptr, 0, ProcessingESP8266, nullptr, 0, nullptr);

    //Receiving the robotic arm operation status to judge what to do next.
	while (true)
	{
		if (comStatus != COM_NORMAL) {
			closesocket(esp8266Sock);
			closesocket(esp32Sock);
			WSACleanup();
		}
		else {
			continue;
		}
	}
    return 0;
}

DWORD WINAPI ProcessingESP8266(LPVOID lpPara) {
	/*
	There are 3 colours now can be recognized(red black yellow), and for each of colour, there are 3 vectors 
	being used:
	vector<pInfo> red
	vector<pInfo> black
	vector<pInfo> yellow
	The operation logic of this thread are:
	1.check if the image processing result is exist.(coObatined == true)
	2.combine 3 vectors and save into vector<pInfo> tasks.
	3.arrange the task sequence
	4.send the information to esp8266
	5.received the work done for one single travel and then send the next information until all work done.
	*/
	int count = 1;	//count the number of time of robotic arm movement.
	//test
	bool bProc = false;
	if (bProc = procImage("test.png", &vRed, RED)) {
		cout << "red object detection done" << endl;
	}
	waitKey(0);
	if (bProc = procImage("test.png", &vBlack, BLACK)) {
		cout << "black object detection done" << endl;
	}
	waitKey(0);
	if (bProc = procImage("test.png", &vYellow, YELLOW)) {
		cout << "yellow object detection done" << endl;
	}
	waitKey(0);
	if (bProc) {
		coObtained = true;
	}
	//test

	while (true)
	{
		Sleep(100);
		while (coObtained)
		{
			int len = -1;		//the size of data sucessful sent.	

			vector<pInfo> vTasks;
			//combine the vectors
			vTasks.insert(vTasks.end(), vRed.begin(), vRed.end());
			vTasks.insert(vTasks.end(), vYellow.begin(), vYellow.end());
			vTasks.insert(vTasks.end(), vBlack.begin(), vBlack.end());
			for (auto i = vTasks.begin();i != vTasks.end();i++) {
				(*i).flag = 5;		//set the status to 5 which corresponding to "move robotic"
				(*i).count= count;	//save the working times
				count++;
			}
			cout << "There are " << vTasks.size() << " tasks in total." << endl;
			//start to send the position to esp8266
			for (auto i = vTasks.begin();i != vTasks.end();i++) {
				len = send(esp8266Sock, (char*)&(*i), sizeof(pInfo), 0);
				cout << "data sent: "<<len << endl;
				if (len > 0) {
					cout << "Send the command to ESP8266 successfully!" << endl;
				}
				else {
					cout << "Send the command to ESP8266 failed!" << endl;
					comStatus = COM_ERROR;
					break;
				}
				//waiting for response for next movement
				char recvBuf[126] = { 0 };
				String strRecv;
				while(true) {
					len = recv(esp8266Sock, recvBuf, sizeof(recvBuf), 0);
					strRecv += recvBuf;
					if (strRecv.length() == 4) {
						break;
					}
				}
				
				int rlt = strcmp(strRecv.c_str(), "NEXT");		//if the echo is NEXT, 
																//then means the robotic is able for next movement.
				if (len > 0 && rlt==0) {
					cout << "The comfirmation for next movement is checked!" << endl;
				}
				else {
					cout << "Send the command to ESP8266 failed!" << endl;
					comStatus = COM_ERROR;
					break;
				}
			}
			//end of the works
			coObtained = false;		//set the status to false to skip the loops.
		}
	}
}

DWORD WINAPI ProcessingESP32(LPVOID lpPara) {
	recvImgFlag = true;				//The trigger for receiving image.
	char sendBuf[] = "picture";		//The mode is set to "picture" which the camera will take the photo only.
	send(esp32Sock, sendBuf, sizeof(sendBuf), 0);		//Send the mode.

	while (true)
	{
		char recvBuf[1024] = { 0 };		//The buffer for receiving image stream
		char str[6] = "size:";			//Header of image before sent from esp32-cam
		int fileLength = 0;				//From header of package, the file size will be obatined.
		FILE* fp = NULL;
		bool fileOpened = false;		
		bool sizeChecked = false;
		char* loc = NULL;				//A temp value for locating file size in the header
		int len = 0;					//received file length.
		int totalLen = 0;				//total received file length
		int t = 0;						//size of data wrote into local.
		while (recvImgFlag) {
			if (!fileOpened) {//Open the file
				//Delete the exist image
				fopen_s(&fp,"captured_img.jpg", "w");
				if (fp == NULL) {
					cout << "failed to open the file" << endl;
					break;
				}
				fclose(fp);

				//Open the new file for writing the new image into it.
				fopen_s(&fp,"captured_img.jpg", "wb+");
				if (fp == NULL) {
					cout << "failed to open the file" << endl;
					break;
				}
				fileOpened = true;
			}
			len = recv(esp32Sock, recvBuf, sizeof(recvBuf), 0);

			//Get the size of image and drop the header
			if (!sizeChecked) {
				loc = strstr(recvBuf, str);
				sizeChecked = true;
			}
			if (loc != NULL) {
				char* temp = recvBuf + 5;
				fileLength = atoi(temp);
				cout << "The size of image:" << fileLength << endl;
				len = 0;				//Avoid the header being written into storage.
				loc = NULL;				
			}
			//count the total data received
			totalLen += len;
			//Write the image binary information to the local file.
			if (len != 0) {
				t += fwrite(recvBuf, 1, len, fp);
			}
			//finished reading the image
			if (totalLen == fileLength) {
				cout << "write " << t << " into the local" << endl;
				cout << "Image stored in local" << endl;
				fclose(fp);
				recvImgFlag = false;	//end of loop
				Mat img = imread("captured_img.jpg");
				imshow("captured", img);
				waitKey(0);
			}
		}//End of receiving image

		//Image processing.
		bool bProc = false;
		if (bProc = procImage("captured_img.jpg",&vRed,RED)) {
			cout << "red object detection done" << endl;
		}
		if (bProc = procImage("captured_img.jpg", &vYellow, YELLOW)) {
			cout << "yellow object detection done" << endl;
		}
		if (bProc = procImage("captured_img.jpg", &vBlack, BLACK)) {
			cout << "black object detectio ndone" << endl;
		}
		
		if (bProc) {
			coObtained = true;
		}
		//end of image processing
		
		//for selection
		while (true) {
			cout << "New objects on the table:(1.Yes 2.No):";
			int choice = 0;
			cin >> choice;
			if (choice == 1) {
				break;
			}
			else {
				continue;
			}
		}
	}
	cout << "Disconnected with ESP32!" << endl;
	comStatus = COM_ERROR;
	return -1;
}