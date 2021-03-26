#pragma once
#include "ColourConfig.h"
#include "struInfo.h"
#include "AngleSolution.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <vector>
#include <math.h>
#include <iostream>

const char str_red[] = "red_outImage";
const char str_black[] = "black_outImage";
const char str_yellow[] = "yellow_outImage";


using namespace cv;
using namespace std;


//calculate white area.
int bSums(Mat src)
{

	int counter = 0;

	Mat_<uchar>::iterator it = src.begin<uchar>();
	Mat_<uchar>::iterator itend = src.end<uchar>();
	for (; it != itend; ++it)
	{
		if ((*it) > 0) counter += 1;
	}
	return counter;
}
//calculate center
int circleCenter(Mat in, vector<float>* x, vector<float>* y, int type)
{
	Mat matSrc = in;

	GaussianBlur(matSrc, matSrc, Size(5, 5), 0);

	vector<vector<Point> > contours;

	vector<Vec4i> hierarchy;



	threshold(matSrc, matSrc, 100, 255, THRESH_BINARY);//Binary of image

	
	findContours(matSrc.clone(), contours, hierarchy, CV_RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point(0, 0));

	vector<Moments> mu(contours.size());

	for (int i = 0; i < contours.size(); i++)

	{

		mu[i] = moments(contours[i], false);

	}

	vector<Point2f> mc(contours.size());

	for (int i = 0; i < contours.size(); i++)

	{

		mc[i] = Point2f(mu[i].m10 / mu[i].m00, mu[i].m01 / mu[i].m00);

	}



	Mat drawing = Mat::zeros(matSrc.size(), CV_8UC1);

	for (int i = 0; i < contours.size(); i++)

	{

		Scalar color = Scalar(255);



		circle(drawing, mc[i], 4, color, -1, 8, 0);
		x->push_back(SCALE * mc[i].x);
		y->push_back(SCALE * mc[i].y);

	}
	//red
	if (type == 1) {
		imshow(str_red, drawing);
	}
	//black
	if (type == 2) {
		imshow(str_black, drawing);
	}
	//yellow
	if (type == 3) {
		imshow(str_yellow, drawing);
	}
	return contours.size();
}

bool procImage(const char *fileName,vector<pInfo> *vInfo,int type) {
	bool bRlt = false;
	do {
		Mat srcImg = imread(fileName);
		if (srcImg.empty()) {
			break;
		}
		else {
			imshow("original", srcImg);
		}

		//Start to process the image.
		Mat hsvImg = srcImg.clone();
		cvtColor(srcImg, hsvImg, COLOR_BGR2HSV);
		imshow("hsv", hsvImg);
		Mat frame_threshold = srcImg.clone();
		//waitKey(0);//test
		Red red;
		Black black;
		Yellow yellow;
		if (type == RED) {
			inRange(hsvImg, Scalar(red.low_H, red.low_S, red.low_V), Scalar(red.high_H, red.high_S, red.high_V), frame_threshold);
			imshow("red", frame_threshold);
		}
		else if (type == BLACK) {
			inRange(hsvImg, Scalar(black.low_H, black.low_S, black.low_V), Scalar(black.high_H, black.high_S, black.high_V), frame_threshold);
			imshow("black", frame_threshold);
		}
		else if (type == YELLOW) {
			inRange(hsvImg, Scalar(yellow.low_H, yellow.low_S, yellow.low_V), Scalar(yellow.high_H, yellow.high_S, yellow.high_V), frame_threshold);
			imshow("yellow", frame_threshold);
		}

		int num = bSums(frame_threshold);	//get the white area.

		cout << "Pixels:" << num << endl;

		int onePiece = (int)oneObjArea;	//for one object, it is pixels occupied in the image.
		float t1 = num / onePiece;
		int count = t1;
		t1 -= count;
		if (t1 > 0.8) {
			count++;
		}
		cout << "Amount:" << count << endl;

		pixelDimension = sqrt(4 * oneObjArea / PI);
		SCALE = realDimension / pixelDimension;
		cout << "Scale ration: " << SCALE << endl;
		vector<float> x;
		vector<float> y;

		int No = circleCenter(frame_threshold, &x, &y,type);
		for (int i = 0;i < x.size();i++) {
			cout << "The coordination for No. " << i + 1 << " is : x=" << x[i] << " y=" << y[i] << " z=" << 49.00 << endl;
		}

		svAng solver;
		//Solving the angle via coordination
		for (int i = 0;i < x.size();i++) {
			solver.coordination(x[i], y[i]);
			solver.linkLength(100, 111, 51.5);//Assign the length for each link
			solver.solveAngle();
			solver.solveAngle4Joint();
			if (!solver.isSolved()) {
				cout << "Failed to calculate the angle" << endl;
				break;
			}
			else {
				pInfo temp;
				temp.type = type;
				float* fRlt = solver.getAngle();
				if (fRlt != nullptr) {
					for (int j = 0;j < 6;j++) {
						temp.pos[j] = fRlt[j];
					}
					cout << "The angle for the No. " << i + 1 << " object, its rotation angle for each joint are:" << endl;
					for (int index = 0;index < 6;index++) {
						cout << temp.pos[index] << endl;
					}
					bRlt = true;
				}

				fRlt = solver.getAngle4Rot();
				if (fRlt != nullptr) {
					for (int j = 0;j < 6;j++) {
						temp.pos[j] = fRlt[j];
					}
					(*vInfo).push_back(temp);		//Save the result to vector
					cout << "The real rotation angle for the No. " << i + 1 << " object, its rotation angle for each joint are:" << endl;
					for (int index = 0;index < 6;index++) {
						cout << (*vInfo)[i].pos[index] << endl;
					}
					bRlt = true;
				}
				else {
					cout << "error occurs when acquiring result" << endl;
					break;
				}
			}
			solver.~svAng();
		}
	} while (false);
	return bRlt;
	
}

