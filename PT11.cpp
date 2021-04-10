#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

using namespace std;

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>

#include <Windows.h>

#include <conio.h>
#include <windows.h>
#include "serial_com.h"
#include "image_transfer.h"

#include "vision.h"
#include "robot.h"
#include "vision_simulation.h"
#include "timer.h"

#include "PT11.h"


PT11::PT11()
{
	pw_l = 1500;
	pw_r = 1500;
}

void PT11::manual_set(int& pw_l, int& pw_r, int& pw_laser, int& laser)
{
	int u[2];

	u[0] = 0;
	u[1] = 0;

	if (KEY(VK_UP)) u[0] = 200;
	if (KEY(VK_DOWN)) u[0] = -200;
	if (KEY(VK_RIGHT)) u[1] = -150;
	if (KEY(VK_LEFT)) u[1] = 150;

	this->pw_l = 1500 + u[1] - u[0];
	this->pw_r = 1500 + u[1] + u[0];

	pw_r = this->pw_r;
	pw_l = this->pw_l;
	laser = 0;

	if (KEY('A')) pw_laser += 100;
	if (KEY('D')) pw_laser -= 100;
	if (KEY('W')) laser = 1;

}

void PT11::set_coord(double x1, double y1, double x2, double y2)
{
	this->x1 = x1;
	this->y1 = y1;
	this->x2 = x2;
	this->y2 = y2;

	if (abs((y1 - y2) / (x1 - x2)) > 0.001)
	{
		theta = atan((y1 - y2) / (x1 - x2));
		if (y1 - y2 > 0 && x1 - x2 > 0) theta = theta;
		if (y1 - y2 > 0 && x1 - x2 < 0) theta = M_PI + theta;
		if (y1 - y2 < 0 && x1 - x2 < 0) theta = M_PI + theta;
		if (y1 - y2 < 0 && x1 - x2 > 0) theta = 2 * M_PI + theta;
	}

	cout << x1 << "\t" << y1 << "\t" << theta << endl;
}

void PT11::collision_points(image& rgb)
{
	Lx[0] = 40;		Ly[0] = 0;		LL[0] = 20;		Ln[0] = 4;
	Lx[1] = -40;	Ly[1] = -50;	LL[1] = 30;		Ln[1] = 4;
	Lx[2] = -120;	Ly[2] = 0;		LL[2] = 20;		Ln[2] = 4;
	Lx[3] = -40;	Ly[3] = 50;		LL[3] = 30;		Ln[3] = 4;

	for (int i = 0; i < 4; i++)
	{
		double* arrx = new double[Ln[i]];
		double* arry = new double[Ln[i]];

		switch (i)
		{
		case 0:
			for (int j = 0; j < Ln[0]; j++)
			{
				arrx[j] = x1 + Lx[i] * cos(theta) - (Ly[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * sin(theta);
				arry[j] = y1 + Lx[i] * sin(theta) + (Ly[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * cos(theta);
				draw_point_rgb(rgb, arrx[j], arry[j], 0, 0, 255);
			}
			break;
		case 1:
			for (int j = 0; j < Ln[0]; j++)
			{
				arrx[j] = x1 + (Lx[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * cos(theta) - (Ly[i]) * sin(theta);
				arry[j] = y1 + (Lx[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * sin(theta) + (Ly[i]) * cos(theta);
				draw_point_rgb(rgb, arrx[j], arry[j], 0, 0, 255);
			}
			break;
		case 2:
			for (int j = 0; j < Ln[0]; j++)
			{
				arrx[j] = x1 + Lx[i] * cos(theta) - (Ly[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * sin(theta);
				arry[j] = y1 + Lx[i] * sin(theta) + (Ly[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * cos(theta);
				draw_point_rgb(rgb, arrx[j], arry[j], 0, 0, 255);
			}
			break;
		case 3:
			for (int j = 0; j < Ln[0]; j++)
			{
				arrx[j] = x1 + (Lx[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * cos(theta) - (Ly[i]) * sin(theta);
				arry[j] = y1 + (Lx[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * sin(theta) + (Ly[i]) * cos(theta);
				draw_point_rgb(rgb, arrx[j], arry[j], 0, 0, 255);
			}
			break;
		
		}

		delete[]arrx;
		delete[]arry;
	}

	
	
	/*
	for (int i = 0; i < 4; i++)
	{
		xg[i] = x1 + Lx[i] * cos(theta) - Ly[i] * sin(theta);
		yg[i] = y1 + Lx[i] * sin(theta) + Ly[i] * cos(theta);
		draw_point_rgb(rgb, xg[i], yg[i], 0, 0, 255);
	}
	*/
}

PT11::~PT11()
{

}
