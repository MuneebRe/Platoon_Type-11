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

#include "Camera.h"
#include "PT11.h"
#include "NeuralNet.h"


PT11::PT11()
{
	pw_l = 1500;
	pw_r = 1500;
	for (int i = 0; i < 4; i++)
	{
		collision_state[i] = 0;
	}
	target_state = 0;
	state_dir[0] = 0;
	state_dir[1] = 0;
	state_laser = 0;

	for (int i = 0; i < 4; i++)
	{
		collision_t_flag[i] = 0;
		collision_dt[i] = 0;
		collision_t1[i] = 0;
		collision_t2[i] = 0;
		collision_state[i] = 0;
	}
	collision_dt_target[0] = 1.00;
	collision_dt_target[1] = 0.90;
	collision_dt_target[2] = 1.00;
	collision_dt_target[3] = 0.90;

}

void PT11::manual_set(int& pw_l, int& pw_r, int& pw_laser, int& laser)
{
	int u[2];

	u[0] = 0;
	u[1] = 0;

	if (KEY(VK_UP)) u[0] = 500;
	if (KEY(VK_DOWN)) u[0] = -500;
	if (KEY(VK_RIGHT)) u[1] = -450;
	if (KEY(VK_LEFT)) u[1] = 450;
	
	this->pw_l = 1500 + u[1] - u[0];
	this->pw_r = 1500 + u[1] + u[0];

	cout << this->pw_l << "\t" << this->pw_r << endl;

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

	calculate_theta(x1, y1, x2, y2, theta);
}

void PT11::collision_points(Camera &view)
{
	//Look at robot.cpp example, how Lx and Ly are used to connect laser on the bot at a distance
	//LL is the lenght between the dots (invisible)
	//Ln is the number of dots the user is interested in using per side
	//[0] Front - [1] right - [2] back - [3] left
	//Basically, there are dots on all sides of the car to detect collision.
	//I had to comment out draw_point_rgb because it messes up with check_collision( )

	Lx[0] = 60;		Ly[0] = 0;		LL[0] = 20;		Ln[0] = 6;
	Lx[1] = -40;	Ly[1] = -50;	LL[1] = 30;		Ln[1] = 6;
	Lx[2] = -120;	Ly[2] = 0;		LL[2] = 20;		Ln[2] = 6;
	Lx[3] = -40;	Ly[3] = 50;		LL[3] = 30;		Ln[3] = 6;

	for (int i = 0; i < 4; i++)
	{
		int* arrx = new int[Ln[i]];		//Dynamic memory, can change number of points interested in using
		int* arry = new int[Ln[i]];		//Kinda like resolution. More points can make it a line

		bool flag =0;
		
		switch (i)						//Different sides, different line rotation. Same for [0] & [2] - [1] & [3]
		{
		case 0:
			for (int j = 0; j < Ln[0]; j++)
			{
				arrx[j] = x1 + Lx[i] * cos(theta) - (Ly[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * sin(theta);
				arry[j] = y1 + Lx[i] * sin(theta) + (Ly[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * cos(theta);
				if(flag == 1) draw_point_rgb(view.return_image(), arrx[j], arry[j], 255, 255, 255);
			}
			break;
		case 1:
			for (int j = 0; j < Ln[1]; j++)
			{
				arrx[j] = x1 + (Lx[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * cos(theta) - (Ly[i]) * sin(theta);
				arry[j] = y1 + (Lx[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * sin(theta) + (Ly[i]) * cos(theta);
				if (flag == 1) draw_point_rgb(view.return_image(), arrx[j], arry[j], 255, 255, 255);
			}
			break;
		case 2:
			for (int j = 0; j < Ln[2]; j++)
			{
				arrx[j] = x1 + Lx[i] * cos(theta) - (Ly[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * sin(theta);
				arry[j] = y1 + Lx[i] * sin(theta) + (Ly[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * cos(theta);
				if (flag == 1) draw_point_rgb(view.return_image(), arrx[j], arry[j], 255, 255, 255);
			}
			break;
		case 3:
			for (int j = 0; j < Ln[3]; j++)
			{
				arrx[j] = x1 + (Lx[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * cos(theta) - (Ly[i]) * sin(theta);
				arry[j] = y1 + (Lx[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * sin(theta) + (Ly[i]) * cos(theta);
				if (flag == 1) draw_point_rgb(view.return_image(), arrx[j], arry[j], 255, 255, 255);
			}
			
			break;
		}
		check_collision(arrx, arry, view, i);

		delete[]arrx;
		delete[]arry;
	}

}

void PT11::check_collision(int arrx[], int arry[], Camera &view, int i)
{
	//Since we're using stuff from the threshold, it's better to use the grey image type instead of rgb image,
	//So we're using "a" image from view[0]

	copy(view.return_image(), view.return_a());
	
	ibyte* pa;

	pa = view.return_a().pdata;

	int* k = new int[Ln[i]];	//Can vary length of point series for collision accuracy
	

	for (int i2 = 0; i2 < Ln[i]; i2++)	//For each dot on the line of points, find position based on 1D image reference
	{
		if (arrx[i2] > 0 && arrx[i2] < view.return_a().width && arry[i2] > 0 && arry[i2] < view.return_a().height)
		{
			k[i2] = arrx[i2] + view.return_a().width * arry[i2];
		}
		else {
			k[i2] = 1 + view.return_a().width;
		}
	}

	for(int i2 =0; i2< Ln[i] ; i2++)	//If either point has 255 at pointer, turn on collision state
	{
		if (pa[k[i2]] == 255 && collision_t_flag[i] == 0)
		{
			collision_t_flag[i] = 1;
			collision_state[i] = true;
			collision_t1[i] = high_resolution_time();
			break;
		}
	}

	int sum = 0;

	for (int i2 = 0; i2 < Ln[i]; i2++)	//if all points of edge have 0 at pointer, than turn off collision state
	{
		if (pa[k[i2]] == 0)
		{
			sum++;
		}
	}
	//cout << "sum plus" << sum << endl;
	//if (sum >= Ln[i]) collision_state[i] = 0;

	delete []k;
	
	for (int i = 0; i < 4; i++)
	{

		if (collision_state[i] == 1 && collision_t_flag[i] == 1)
		{
			collision_t2[i] = high_resolution_time();
			collision_dt[i] = collision_t2[i] - collision_t1[i];
		}

		if (collision_dt[i] > collision_dt_target[i] && collision_t_flag[i] == 1)
		{
			collision_t_flag[i] = 0;
			collision_state[i] = 0;
		}
	}
	
	//cout << "\nAAAAAAAAAAAAAAAAAAAAAAAAA" << endl;
	//cout << collision_state[0] << "\t" << collision_t_flag[0] << "\t" << collision_t1[0] << "\t" << collision_t2[0] << "\t" << collision_dt[0] << endl;

	//cout << collision_state[0] << "\t" << collision_state[1] << "\t" << collision_state[2] << "\t" << collision_state[3] << endl;
	
}

void PT11::calculate_theta(double x1, double y1, double x2, double y2, double &theta)
{
	double* temp = new double;
	*temp = ((y1 - y2) / (x1 - x2));

	if (abs(*temp) > 0.001)				//Makes theta angle continous 0 - 2 pi
	{
		theta = atan(*temp);
		if (y1 - y2 >= 0 && x1 - x2 >= 0) theta = theta;
		if (y1 - y2 >= 0 && x1 - x2 < 0) theta = M_PI + theta;
		if (y1 - y2 < 0 && x1 - x2 < 0) theta = M_PI + theta;
		if (y1 - y2 < 0 && x1 - x2 >= 0) theta = 2 * M_PI + theta;
	}
	delete temp;

	//cout << x1 << "\t" << y1 << "\t" << theta << endl;
}

void PT11::find_target(PT11 enemy)
{
	calculate_theta(enemy.get_x1(), enemy.get_y1(), x1, y1, theta_target1);
	calculate_theta(enemy.get_x2(), enemy.get_y2(), x1, y1, theta_target2);
	//cout << theta_target << endl;

	target_delta1 = theta_target1 - theta;
	target_delta2 = theta_target2 - theta;
	//cout << target_delta1 << "\t" << target_delta2 << endl;

	trigger_range = 0.05;
	
	if (abs(target_delta1) < trigger_range || abs(target_delta2) < trigger_range){
		target_state = 1;
	}
	else {
		target_state = 0;
	}

	if (target_delta1 >= 0) { state_dir[0] = 1; }
	else if (target_delta1 < 0) {state_dir[1] = 1; }

	if (state_dir[0] == 1 && state_dir[1] == 1)
	{
		state_dir[0] = 1;
		state_dir[1] = 0;
	}

	if (target_state == 1) {
		state_dir[0] = 0;
		state_dir[1] = 0;
	}

	//cout << target_state << "\t" << state_dir[0] << "\t" << state_dir[1] << endl;
}

void PT11::m_runNet(int& pw_l, int& pw_r, int& laser)
{
	
	net_mem[0] = collision_state[0];
	net_mem[1] = collision_state[1];
	net_mem[2] = collision_state[2];
	net_mem[3] = collision_state[3];
	net_mem[4] = state_dir[0];
	net_mem[5] = state_dir[1];
	net_mem[6] = target_state;
	
	toTerminal2(net_mem, net_out);
	
	/*
	for (int i = 0; i < 7; i++)
	{
		cout << net_mem[i] << "\t";
	}
	cout << endl;
	*/

	pw_l = net_out[0]*1000+1000;
	pw_r = net_out[1]*1000+1000;
	laser = net_out[2];
	
}

PT11::~PT11()
{

}
