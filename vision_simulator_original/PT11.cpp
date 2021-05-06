#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

using namespace std;

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>
#include <iomanip>

#include <Windows.h>

#include <conio.h>
#include <windows.h>
#include <string>
#include "serial_com.h"
#include "image_transfer.h"

#include "vision.h"
#include "robot.h"
#include "vision_simulation.h"
#include "timer.h"

#include "Camera.h"
#include "PT11.h"

#include "Neural_Network/NeuroNet.h"
#include "Neural_Network/Input.h"
#include "Neural_Network/Hidden.h"
#include "Neural_Network/Output.h"


PT11::PT11(Camera& view)
{
	pw_l = 1500;		//Setup motors at 0
	pw_r = 1500;		//Setup motors at 0

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

	for (int i = 0; i < 8; i++)
	{
		distance_log[i] = 0;
	}

	trial_timer1 = high_resolution_time();

	is_there_obstacle = 0;

	label_nb_1 = 0;
	label_nb_2 = 0;

	radar_rgb.width = 640;
	radar_rgb.height = 480;
	radar_rgb.type = RGB_IMAGE;

	radar_greyscale.type = GREY_IMAGE;
	radar_greyscale.width = 640;
	radar_greyscale.height = 480;

	safezone_greyscale.type = GREY_IMAGE;
	safezone_greyscale.width = 640;
	safezone_greyscale.height = 480;

	safezone_label.type = LABEL_IMAGE;
	safezone_label.width = 640;
	safezone_label.height = 480;

	radar_a.type = GREY_IMAGE;
	radar_a.width = 640;
	radar_a.height = 480;

	radar_b.type = GREY_IMAGE;
	radar_b.width = 640;
	radar_b.height = 480;

	allocate_image(radar_rgb);	        //Houses RGB image with safezones drawn on
	allocate_image(radar_greyscale);	//Houses greyscale image copied from object 'a' from Camera class, used to process the safe zone
	allocate_image(safezone_greyscale);	//Houses the threshold processed image of the safezone, processed with the help of radar_rgb masking the safe zone
	allocate_image(safezone_label);		//Houses the labelled image of safezone_greyscale, labelling the safe zones
	allocate_image(radar_a);			//Used to erode/dilate binary image safezone_greyscale
	allocate_image(radar_b);

	attack_trigger = 0;
	evade_trigger = 0;

	disable_system = 0;

}

void PT11::init_neural()	//REF1-4 Initialize everytime the simulations starts over
{
	topology = new Neural_Net(11, 11, 2);	//Build neural network topology of input, hidden and output nodes. Include Bias for input & hidden
	topology->set_bias(false);
	topology->set_trial_nb_limit(30);

	flag_reset = 0;

	topology->load_best();	//Load the best weight configuration with the highest fitness recorded as of yet

	topology->randomize_weights();		//Perform either relative or fully randomize weightings for each trial
	
	
	for (int i = 0; i < (rand()%(16)); i++)
	{
		//topology->randomize_just_one_weight();	//Change the weight of only one neuron
	}
	

	cout << "Topology for pt11 Initialized!" << endl;
}

void PT11::manual_set(int& pw_l, int& pw_r, int& pw_laser, int& laser)
//Controls both motors based on the keywboard arrow inputs
{
	int u[2];

	u[0] = 0;
	u[1] = 0;

	//Change state variables based on key inputs
	if (KEY(VK_UP)) u[0] = 500;
	if (KEY(VK_DOWN)) u[0] = -500;
	if (KEY(VK_RIGHT)) u[1] = -450;
	if (KEY(VK_LEFT)) u[1] = 450;
	
	//Calculate motor outputs and save value inside robot motor info
	this->pw_l = 1500 + u[1] - u[0];
	this->pw_r = 1500 + u[1] + u[0];

	//Change motor value of actual program using motor info that was previously saved
	pw_r = this->pw_r;
	pw_l = this->pw_l;
	laser = 0;

	if (KEY('W')) laser = 1;

}

void PT11::set_coord(double x1, double y1, double x2, double y2)
{
	this->x1 = x1;	//Front of robot
	this->y1 = y1;	//Front of robot
	this->x2 = x2;	//Back of robot
	this->y2 = y2;	//Back of robot

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

	Lx[0] = 31;		Ly[0] = 0;		LL[0] = 20;		Ln[0] = 4;
	Lx[1] = -42;	Ly[1] = -37;	LL[1] = 40;		Ln[1] = 4;
	Lx[2] = -112;	Ly[2] = 0;		LL[2] = 20;		Ln[2] = 4;
	Lx[3] = Lx[1];	Ly[3] = -Ly[1];	LL[3] = LL[1];	Ln[3] = Ln[1];

	for (int i = 0; i < 4; i++)
	{
		int* arrx = new int[Ln[i]];		//Dynamic memory, can change number of points interested in using
		int* arry = new int[Ln[i]];		//Kinda like resolution. More points can make it a line

		bool flag = 0;
		
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
		
		//Compare if the pixel value of each point in the line is 255 or 0
		//If any of the values are 255, then a collision has occured
		check_collision(arrx, arry, view, i);

		delete[]arrx;
		delete[]arry;
	}

}

void PT11::fill_wheel_void(Camera& view)
{
	//This is used to merge the labels on the wheel with the front side of the robot
	//Else the VFF would conside the wheels as obstacles

	int x_draw;
	int y_draw;

	int Lx = 0;
	int Ly = -13;
	int length = 55;

	for (int i = -length/2; i < length; i++)
	{
		x_draw = x1 + Lx * cos(theta) - (Ly + i) * sin(theta);
		y_draw = y1 + Lx * sin(theta) + (Ly + i) * cos(theta);
		draw_point(view.return_a(), x_draw, y_draw, 255);
	}

}


void PT11::distance_sensor(Camera& view, PT11 &enemy)
{	
	Lx[0] = 30;		Ly[0] = 0;		LL[0] = 6;		Ln[0] = 100;			//Front
	Lx[1] = -20;	Ly[1] = -55;	LL[1] = 6;		Ln[1] = 100;			//Front Right
	Lx[2] = -40;	Ly[2] = 0;		LL[2] = 6;		Ln[2] = 100;			//Right
	Lx[3] = -100;	Ly[3] = -10;	LL[3] = 6;		Ln[3] = 100;			//Back Right
	Lx[4] = 40;		Ly[4] = 0;		LL[4] = 6;		Ln[4] = 100;			//Back
	Lx[5] = Lx[3];	Ly[5] = -Ly[3];	LL[5] = LL[3];	Ln[5] = Ln[3];		//Back Left
	Lx[6] = Lx[2];	Ly[6] = Ly[2];	LL[6] = LL[2];	Ln[6] = Ln[2];		//Left
	Lx[7] = Lx[1];	Ly[7] = -Ly[1];	LL[7] = LL[1];	Ln[7] = Ln[1];		//Top Left

	Ax[0] = 20;		Ay[0] = 0;		AF[0] = 0.0;
	Ax[1] = 30;		Ay[1] = 0;		AF[1] = -1.0;
	Ax[2] = 0;		Ay[2] = -30;	AF[2] = 0.0;
	Ax[3] = 0;		Ay[3] = -30;	AF[3] = -1.0;
	Ax[4] = -165;	Ay[4] = Ay[0];	AF[4] = 0.0;
	Ax[5] = 0;		Ay[5] = 30;		AF[5] = 1.0;
	Ax[6] = 0;		Ay[6] = -Ay[2];	AF[6] = 0.0;
	Ax[7] = 30;		Ay[7] = -Ay[1];	AF[7] = 1.0;

	enemy.collision_points(view);					//Enable collision sensor for Robot A

	for (int i = 0; i < 8; i++)
	{
		int* arrx = new int[Ln[i]];		//Dynamic memory, can change number of points interested in using
		int* arry = new int[Ln[i]];		//Kinda like resolution. More points can make it a line

		bool flag = 0;

		switch (i)
		{
		case 0:
			for (int j = 0; j < Ln[i]; j++)
			{
				arrx[j] = x1 + Lx[i] * cos(theta) - Ly[i] * sin(theta);
				arry[j] = y1 + Lx[i] * sin(theta) + Ly[i] * cos(theta);

				arrx[j] += (Ax[i] + LL[i] * j) * cos(theta) - (Ay[i] + LL[i] * j* AF[i]) * sin(theta);
				arry[j] += (Ax[i] + LL[i] * j) * sin(theta) + (Ay[i] + LL[i] * j* AF[i]) * cos(theta);
				if (flag == 1) draw_point_rgb(view.return_image(), arrx[j], arry[j], 255, 255, 255);
				
			}
			break;
		case 1:
			for (int j = 0; j < Ln[i]; j++)
			{
				arrx[j] = x1 + Lx[i] * cos(theta) - Ly[i] * sin(theta);
				arry[j] = y1 + Lx[i] * sin(theta) + Ly[i] * cos(theta);

				arrx[j] += (Ax[i] + LL[i] * j) * cos(theta) - (Ay[i] + LL[i] * j * AF[i]) * sin(theta);
				arry[j] += (Ax[i] + LL[i] * j) * sin(theta) + (Ay[i] + LL[i] * j * AF[i]) * cos(theta);
				if (flag == 1) draw_point_rgb(view.return_image(), arrx[j], arry[j], 255, 255, 255);
			}
			break;

		case 2:
			for (int j = 0; j < Ln[i]; j++)
			{
				arrx[j] = x1 + Lx[i] * cos(theta) - Ly[i] * sin(theta);
				arry[j] = y1 + Lx[i] * sin(theta) + Ly[i] * cos(theta);

				arrx[j] += (Ax[i]) * cos(theta) - (Ay[i] - LL[i] * j) * sin(theta);
				arry[j] += (Ax[i]) * sin(theta) + (Ay[i] - LL[i] * j) * cos(theta);
				if (flag == 1) draw_point_rgb(view.return_image(), arrx[j], arry[j], 255, 255, 255);
			}
			break;
		case 3:
			for (int j = 0; j < Ln[i]; j++)
			{
				arrx[j] = x1 + Lx[i] * cos(theta) - Ly[i] * sin(theta);
				arry[j] = y1 + Lx[i] * sin(theta) + Ly[i] * cos(theta);

				arrx[j] += (Ax[i] + LL[i] * j * AF[i]) * cos(theta) - (Ay[i] - LL[i] * j) * sin(theta);
				arry[j] += (Ax[i] + LL[i] * j * AF[i]) * sin(theta) + (Ay[i] - LL[i] * j) * cos(theta);
				if (flag == 1) draw_point_rgb(view.return_image(), arrx[j], arry[j], 255, 255, 255);
			}
			break;
		case 4:
			for (int j = 0; j < Ln[i]; j++)
			{
				arrx[j] = x1 + Lx[i] * cos(theta) - Ly[i] * sin(theta);
				arry[j] = y1 + Lx[i] * sin(theta) + Ly[i] * cos(theta);

				arrx[j] += (Ax[i] - LL[i] * j) * cos(theta) - (Ay[i]) * sin(theta);
				arry[j] += (Ax[i] - LL[i] * j) * sin(theta) + (Ay[i]) * cos(theta);
				if (flag == 1) draw_point_rgb(view.return_image(), arrx[j], arry[j], 255, 255, 255);
			}
			break;
		case 5:
			for (int j = 0; j < Ln[i]; j++)
			{
				arrx[j] = x1 + Lx[i] * cos(theta) - Ly[i] * sin(theta);
				arry[j] = y1 + Lx[i] * sin(theta) + Ly[i] * cos(theta);

				arrx[j] += (Ax[i] - LL[i] * j) * cos(theta) - (Ay[i] + LL[i] * j * AF[i]) * sin(theta);
				arry[j] += (Ax[i] - LL[i] * j) * sin(theta) + (Ay[i] + LL[i] * j * AF[i]) * cos(theta);
				if (flag == 1) draw_point_rgb(view.return_image(), arrx[j], arry[j], 255, 255, 255);
			}
			break;
		case 6:
			for (int j = 0; j < Ln[i]; j++)
			{
				arrx[j] = x1 + Lx[i] * cos(theta) - Ly[i] * sin(theta);
				arry[j] = y1 + Lx[i] * sin(theta) + Ly[i] * cos(theta);

				arrx[j] += (Ax[i]) * cos(theta) - (Ay[i] + LL[i] * j) * sin(theta);
				arry[j] += (Ax[i]) * sin(theta) + (Ay[i] + LL[i] * j) * cos(theta);
				if (flag == 1) draw_point_rgb(view.return_image(), arrx[j], arry[j], 255, 255, 255);
			}
			break;
		case 7:
			for (int j = 0; j < Ln[i]; j++)
			{
				arrx[j] = x1 + Lx[i] * cos(theta) - Ly[i] * sin(theta);
				arry[j] = y1 + Lx[i] * sin(theta) + Ly[i] * cos(theta);

				arrx[j] += (Ax[i] + LL[i] * j * AF[i] ) * cos(theta) - (Ay[i] + LL[i] * j) * sin(theta);
				arry[j] += (Ax[i] + LL[i] * j * AF[i] ) * sin(theta) + (Ay[i] + LL[i] * j) * cos(theta);
				if (flag == 1) draw_point_rgb(view.return_image(), arrx[j], arry[j], 255, 255, 255);
			}
			break;
		}

		//Draw a series of points that change depending on the robot theta
		//If a value of 255 is detected at any of those points, then
		//the point number at which it was detected will be the distance sensor value
		distance_input(arrx, arry, view, i);

		if (i ==0) is_obstacle_before_enemy(arrx, arry, enemy, view);
		
		delete[]arrx;
		delete[]arry;

		//cout << distance_log[i] << "\t";
		//if (i == 7) cout << endl;
	}
	
}

void PT11::distance_input(int arrx[], int arry[], Camera& view, int i)
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

	for (int i2 = 0; i2 < Ln[i]; i2++)	//If either point has 255 at pointer, turn on collision state
	{
		
		if (arrx[i2] < 0 || arrx[i2] > view.return_a().width || arry[i2] < 0 || arry[i2] > view.return_a().height)
		{
			//pa[k[i2]] = 255;
			distance_log[i] = i2;
			break;
		}
		
		//cout << (int)pa[k[i2]] << "\t" << i2 << endl;
		if (pa[k[i2]] == 255)
		{
			//cout << i2 << endl;
			distance_log[i] = i2;
			break;
		}
		else
		{
			distance_log[i] = Ln[i];
		}
	}


	delete[]k;


}

void PT11::is_obstacle_before_enemy(int arrx[], int arry[], PT11 enemy, Camera& view)
{
	//To make sure the robot doesn't fire at the enemy despite having
	//an obstacle in the middle, this function prevents that kind of enemy detection

	int what_label = 0;

	ibyte* pa;

	pa = view.return_a().pdata;

	int* k = new int[Ln[0]];	//Can vary length of point series for collision accuracy

	int distance_log_enemy = 1000;

	for (int i2 = 0; i2 < Ln[0]; i2++)	//For each dot on the line of points, find position based on 1D image reference
	{

		if (arrx[i2] > 0 && arrx[i2] < view.return_a().width && arry[i2] > 0 && arry[i2] < view.return_a().height)
		{
			what_label = view.label_at_coordinate(arrx[i2], arry[i2]);
		}
		else
		{
			//distance_log_enemy = 1000;
			break;
		}

		//draw_point_rgb(view.return_image(), arrx[i2], arry[i2], 255, 255, 255);
		
		if (what_label == enemy.label_nb_1 || what_label == enemy.label_nb_2)
		{
			distance_log_enemy = i2;
			break;
		}
	}

	//cout << distance_log[0] << "\t" << distance_log_enemy - 1<< endl;

	if (distance_log[0] > distance_log_enemy-1)
	{

		target_state = 1;
		//cout << "ATTACK!!!!!!" << endl;
	}
	else
	{
		target_state = 0;
	}

	delete[]k;
	
	distance_enemy1 = sqrt(pow(enemy.get_x1() - x1, 2) + pow(enemy.get_y1() - y1, 2));
	distance_enemy2 = sqrt(pow(enemy.get_x2() - x1, 2) + pow(enemy.get_y2() - y1, 2));
	distance_enemy_avg = (distance_enemy1 + distance_enemy2) / 2;

	
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

void PT11::theta_target_delta_fix(double theta_target, double& target_delta)
{
	int aim_dir;

	target_delta = theta_target - theta;

	if (target_delta < 0) target_delta = theta_target - theta + 2 * M_PI;

	double value1, value2;
	value1 = target_delta;
	value2 = 2 * M_PI - target_delta;

	if (value1 < value2)
	{
		target_delta = value1;
		aim_dir = -1;
	}
	if (value1 > value2)
	{
		target_delta = value2;
		aim_dir = 1;
	}

	if (aim_dir == 1)
	{
		target_delta = -target_delta;
	}
	if (aim_dir == -1)
	{
		target_delta = target_delta;
	}
}

void PT11::find_target(PT11 enemy)
{
	calculate_theta(enemy.get_x1(), enemy.get_y1(), x1, y1, theta_target1);
	calculate_theta(enemy.get_x2(), enemy.get_y2(), x1, y1, theta_target2);
	//cout << theta_target1 << endl;

	int aim_dir;

	//cout << theta_target1 << "   " << theta_target2 << endl;

	theta_target_delta_fix(theta_target1, target_delta1);
	theta_target_delta_fix(theta_target2, target_delta2);

	//cout << target_delta1 << "     " << target_delta2 << "    " << endl;
	
	
	//cout << state_dir[0] << "    " << state_dir[1] << endl;
}

void PT11::NeuroLearn(int& pw_l, int& pw_r, int& laser, int &trial_number) 
{

	//REF1-5 Learning process
	//Everytime the robot aims at the enemy and there is no obstacle, increase fitness value
	//If trial is running for more than trial_dt, assume collision and reset to new trial run
	//After each trial, record the trial number, fitness value and store the weights in the trial#.txt under Fitness_Logs
	//If trial 9 reached, or accidently higher, then find the trial with the best fitness withing those 10 trials.
	//Also compare with the fitness of the best.txt one, 
	//And store it in Fitness_Logs/best.txt. Each new trial will take the weights from best.txt. Depending on how REF1-6 is setup

	static int fitness = 0;

	
	bool init = 0;

	if (init == 0)
	{
		
		init = 1;
	}

	trial_timer2 = high_resolution_time();
	double timer_dt = trial_timer2 - trial_timer1;
	trial_dt = 20.00;

	
	for (int i = 0; i < 4; i++)
	{
		cout << collision_state[i] << "   ";
	}
	cout << endl;

	if (collision_state[0] == 1 || collision_state[1] == 1 || collision_state[2] == 1 || collision_state[3] == 1 || KEY('O') || timer_dt > trial_dt)
	{
		flag_reset = 1;

		topology->set_trial_number(trial_number);
		topology->set_finess_number(fitness);


		topology->save_weights();
		

		if (trial_number >= topology->get_trial_nb_limit())
		{
			topology->find_best();
			trial_number = 0;
		}
		
		fitness = 0;
		Sleep(800);
	}
	
	//topology->find_best();

	//cout << flag_reset << endl;
	//if (distance_enemy1 < 100) distance_enemy1 = 100;
	//fitness = 800 -  distance_enemy1;
	if (target_state == 1) fitness = fitness + 100;
	if (target_state == 1) fitness++;
	//fitness++;
	//int distance_enemy1 = sqrt(pow(enemy.get_x1() - x1, 2) + pow(enemy.get_y1() - y1, 2));

	//fitness = 

	cout << "trial: " << trial_number << " ";
	cout << "fit: " << fitness << " ";
	cout << endl;

	cout << fixed;
	cout << setprecision(2);

	for (int i = 0; i < 8; i++)
	{
		topology->input[i].set_value(distance_log[i] / Ln[i]);
		//cout << "  dis-" << i << ": " << distance_log[i] / Ln[i];
		//if (i == 7) cout << endl;
	}

	
	topology->input[8].set_value(state_dir[0]);
	topology->input[9].set_value(state_dir[1]);
	topology->input[10].set_value(target_state);
	//topology->input[8].set_value(target_state);
	
	/*
	cout << "Aim L - " << state_dir[0] << " ";
	cout << "Aim R - " << state_dir[1] << " ";
	
	*/

	/*
	topology->input[0].set_value(collision_state[0]);
	topology->input[1].set_value(collision_state[1]);
	topology->input[2].set_value(collision_state[2]);
	topology->input[3].set_value(collision_state[3]);
	*/


	topology->calculate_hidden();
	topology->calculate_output();
	//topology->print_inputs();
	//topology->print_hidden();
	//topology->print_output();

	/*
	cout << "pw_l output: " << topology->output[0].get_value() << " ";
	cout << "pw_r output: " << topology->output[1].get_value() << " ";
	cout << "laser: " << topology->output[2].get_value() << endl;
	*/

	//Outputs are between 0 and 1, the functions below convert it to 1000 to 2000 for the "servo" control
	//My guess is that the because there's so much going on, that getting 11 inputs into 2 outputs is risky.
	//f it, this is still the better option, since sending one discret command interferes with othoers.
	//In this one, however, I'm making so if both outputs are 0, then the car will go straight and crash,
	//instead of rotating counter clockwise endlessly...
	pw_l = topology->output[0].get_value() * 1000 + 1000;  //1000-2000
	//pw_r = topology->output[1].get_value() * 1000 + 1000; 
	pw_r = topology->output[1].get_value() * -1000 + 2000;  //2000-1000
	//laser = topology->output[2].get_value();

}

void PT11::scout(int& pw_l, int& pw_r, int& pw_laser, int& laser)
{
	//Control the robot so it avoids obstacles and attacks the enemy
	//Using only those variables:

	//Distance sensor:
	//distance_log[0]	Front distance sensor
	//distance_log[1]	Front Right distance sensor
	//distance_log[2]	Right side distance sensor
	//distance_log[3]	Back right distance sensor
	//distance_log[4]	Back distance sensor
	//distance_log[5]	Back left distance sensor
	//distance_log[6]	Left distance sensor
	//distance_log[7]	Forward left distance sensor
	//Note: The values stored in those is under Ln[...], check void PT11::distance_sensor.
	//Basically, if Ln[0] were 100, my robot is draw 100 points on a straight line
	//If the white pixel of an obstacle was found in point 60, then ditance_log[0] would return 60.
	
	//Collision sensor:
	//collision_state[0]	Front collision
	//collision_state[1]	Right collision
	//collision_state[2]	Back collision
	//collision_state[3]	Left collision
	//Value is either 0 or 1.

	//target_state		Will return 1 if the front distance sensor is directly looking at the enemy
	//					Will return 0 if enemy hiding behind obstacle, or just looking at obstacle
	//state_dir[0]		Will return 1 if the enemy is counter clockwise from your theta
	//state_dir[1]		Will return 0 if enemy is clockwise from your theta

	//In order to know if those values make sense, use cout << [thing of interest] << "  " << [other thing] << ...  << endl;
	//To use this whole function, make sure that AI_player = 0 in program.cpp, and that pt11.scout(pw_l, pw_r, pw_laser, laser);
	//is commented out, while commenting pt11.manual_set(pw_l, pw_r, pw_laser, laser);

	int action = 0;
	cout << is_there_obstacle << endl;

	if (state_dir[0] == 1) action = 0;		
	if (state_dir[1] == 1) action = 1;
	if (target_state == 1) action = 2;

	switch (action)
	{
	case 0:
		pw_l = 2000;
		pw_r = 2000;
		break;
	case 1:
		pw_l = 1000;
		pw_r = 1000;
		break;
	case 2:
		pw_l = 1000;
		pw_r = 2000;
		break;
	}
	
}

void PT11::attack(int& pw_l, int& pw_r, int& pw_laser, int& laser)
{
	attack_trigger = 1;
	evade_trigger = 0;

	int u[2] = { 0,0 };

	static double time1 = 0;
	static double time2 = 0.1;
	double time_delta;
	time2 = high_resolution_time();
	time_delta = time2 - time1;
	time1 = time2;

	double kp_PID = 10000;
	double kd_PID = 30;
	double ki_PID = 1;

	static double error = 0;
	static double old_error = 0;
	static double error_dot = 0;
	static double int_error = 0;

	static double theta_delta;
	int aim_dir;

	theta_target_delta_fix(VFF_theta, theta_delta);

	if (distance_enemy_avg < 150)
	{
		if (target_delta1 > target_delta2)
		{
			error = target_delta2;
		}
		else if (target_delta1 < target_delta2)
		{
			error = target_delta1;
		}
	}
	else
	{
		error = theta_delta;
	}
	error = theta_delta;
	//fout << VFF_mag << endl;
	error_dot = (error - old_error) / time_delta;
	int_error = int_error + error * time_delta;
	u[1] = kp_PID * error + ki_PID * int_error + kd_PID * error_dot;
	
	if (u[1] > 400) u[1] = 400;
	if (u[1] < -400) u[1] = -400;

	old_error = error;

	double error_limit = 1.00;

	if (abs(error) <= error_limit)
	{
		u[0] = 500;
	}
	if ( abs(error) > error_limit) {
		u[0] = 0;
	}

	
	if (target_state == 1)
	{
		u[0] = 500;
		u[1] = 0;

		if (collision_state[0] == 1) u[0] = -100;
		if (collision_state[2] == 1) u[0] = 100;
	}

	//cout << VFF_mag << endl;

	if (distance_log[0] < 10 || distance_log[1] < 8 || distance_log[7] < 8) u[0] = -500;
	if (distance_log[4] < 10 || distance_log[3] < 8 || distance_log[5] < 8) u[0] = 500;
	
	if (collision_state[0] == 1) u[0] = -100;
	//if (collision_state[1] == 1) u[1] = 100;
	//if (collision_state[3] == 1) u[1] = -100;
	if (collision_state[2] == 1) u[0] = 300;

	//if (distance_log[6] < 4) u[1] = 100;
	//if (distance_log[2] < 4) u[1] = -100;
	
	if (VFF_mag < 300) u[0] = 500;
	if (VFF_mag > 3000) u[0] = 0;
	
	//cout << abs(error) << "   " << VFF_mag << endl;

	if (distance_enemy_avg < 150)
	{
		u[0] = 0;
	}
	
	if (distance_log[0] < 30 && target_state == 1 && abs(error) < 0.05)
	{
		u[0] = 0;
	}


	
	if (target_state == 1 && (target_delta1 < 0.4 || target_delta2 < 0.4))
	{
		//cout << "FIRE! FIRE!" << endl;
		//This is where you could add the laser thing later
	}
	
	//cout << distance_enemy1 << endl;

	if (disable_system == 1) { u[0] = 0; }

	if (disable_laser == 1) { u[0] = 0; u[1] = 0;}

	this->pw_l = 1500 + u[1] - u[0];
	this->pw_r = 1500 + u[1] + u[0];

	//cout << this->pw_l << "\t" << this->pw_r << endl;
	//cout << distance_log[0] << endl;
	//cout << distance_log[0] << "   " << target_state << "   " <<  abs(error) << endl;

	static int laser_timer;

	if (target_state == 1)
	{
		laser_timer++;
	}
	else laser_timer = 0;

	bool laser_trigger = 0;

	if (laser_timer > 1 && disable_system == 0 && disable_laser == 0)
	{
		laser_trigger = 1;
		laser_timer = 0;
	}

	pw_r = this->pw_r;
	pw_l = this->pw_l;
	laser = laser_trigger;
	//laser = 0; //Temporary, for tuning purposes

}

void PT11::evade(int& pw_l, int& pw_r, int& pw_laser, int& laser)
{
	evade_trigger = 1;
	attack_trigger = 0;

	int u[2] = { 0,0 };

	static double time1 = 0;
	static double time2 = 0.1;
	double time_delta;
	time2 = high_resolution_time();
	time_delta = time2 - time1;
	time1 = time2;

	double kp_PID = 10000;
	double kd_PID = 30;
	double ki_PID = 1;

	static double error = 0;
	static double old_error = 0;
	static double error_dot = 0;
	static double int_error = 0;

	static double theta_delta;
	int aim_dir;

	theta_target_delta_fix(VFF_theta, theta_delta);

	if (distance_enemy_avg < 150)
	{
		if (target_delta1 > target_delta2)
		{
			error = target_delta2;
		}
		else if (target_delta1 < target_delta2)
		{
			error = target_delta1;
		}
	}
	else
	{
		error = theta_delta;
	}
	error = theta_delta;
	//fout << VFF_mag << endl;
	error_dot = (error - old_error) / time_delta;
	int_error = int_error + error * time_delta;
	u[1] = kp_PID * error + ki_PID * int_error + kd_PID * error_dot;

	if (u[1] > 400) u[1] = 400;
	if (u[1] < -400) u[1] = -400;

	old_error = error;

	double error_limit = 1.00;

	if (abs(error) <= error_limit)
	{
		u[0] = 500;
	}
	if (abs(error) > error_limit) {
		u[0] = 0;
	}


	if (collision_state[0] == 1) u[0] = -100;
	//if (collision_state[1] == 1) u[1] = 100;
	//if (collision_state[3] == 1) u[1] = -100;
	if (collision_state[2] == 1) u[0] = 300;

	//cout << VFF_mag << endl;
	
	/*
	if (VFF_mag < 1500)
	{
		u[0] = 500;
	}
	else
	{
		u[0] = 0;
	}

	if (VFF_mag > 20000)
	{
		u[0] = 500;
	}
	*/

	//if (VFF_mag < 300) u[0] = 500;
	if (VFF_mag > 400000) { u[0] = 0; u[1] = 0; }

	//cout << VFF_mag << endl;

	/*
	if (VFF_mag < 40000)
	{
		u[0] = 500;
	}
	else
	{
		u[0] = 0;
	}
	*/

	//cout << VFF_mag << endl;

	if (disable_system == 1) { u[0] = 0; }
	

	this->pw_l = 1500 + u[1] - u[0];
	this->pw_r = 1500 + u[1] + u[0];

	//cout << this->pw_l << "\t" << this->pw_r << endl;
	//cout << distance_log[0] << endl;
	//cout << distance_log[0] << "   " << target_state << "   " <<  abs(error) << endl;


	pw_r = this->pw_r;
	pw_l = this->pw_l;
	laser = 0;


}


void PT11::highlight_view(Camera& view, PT11 enemy)
{
	bool flag_draw = 1;
	int radar_radius = 0;
	double theta_index = 0;
	double theta_jump = 0.1;
	int radar_minimum = 50;
	int radius_jump = 2;
	int radius_limit;
	double vector_x = 0;
	double vector_y = 0;
	double counter_vector_x = 0;
	double counter_vector_y = 0;
	double multiplier;
	double enemy_multiplier;
	bool enemy_trigger;
	double shadow_multiplier;
	bool shadow_zone_trigger;

	/*
	for (int i = 0; i < 4; i++)
	{
		cout << enemy.collision_state[i] << "   ";
	}
	cout << endl;
	*/

	if (enemy.collision_state[0] == 1 || enemy.collision_state[1] == 1 || enemy.collision_state[2] == 1 || enemy.collision_state[3] == 1)
	{
		disable_system = 1;
		disable_laser = 1;
	}
	else
	{
		disable_system = 0;
		disable_laser = 0;
	}
	
	while (theta_index < (2 * M_PI) && disable_system != 1)
	{
		radius_limit = 200;
		multiplier = 1;

		if (attack_trigger == 1)
		{
			enemy_multiplier = -20.00;
		}
		else
		{
			enemy_multiplier = 1.00;
		}

		shadow_multiplier = -200.0;

		enemy_trigger = 0;
		shadow_zone_trigger = 0;

		theta_index = theta_index + theta_jump;

		double sym_offset;
		double sym_size;
		double sym_radius;
		double sym_multi;

		/*
		
		double temp = 0.5;
		sym_offset = M_PI/2 + temp /2.0;
		sym_size = temp;
		sym_radius = 70;
		sym_multi = 1.0;
		VFF_section_modifier(theta_index, sym_offset, sym_size, radius_limit, sym_radius, multiplier, sym_multi);
		VFF_section_modifier(theta_index, 2*M_PI - sym_offset, sym_size, radius_limit, sym_radius, multiplier, sym_multi);
		
		
		sym_offset = M_PI;
		sym_size = 0.5;
		sym_radius = 130;
		sym_multi = 1.0;
		VFF_section_modifier(theta_index, sym_offset, sym_size, radius_limit, sym_radius, multiplier, sym_multi);
		//VFF_section_modifier(theta_index, 2 * M_PI - sym_offset, sym_size, radius_limit, sym_radius, multiplier, sym_multi);
		*/

		double theta_bracket_1 = theta + 0 + M_PI + 1.0;
		double theta_bracket_2 = theta + 0 + M_PI - 1.0;

		radar_radius = 0;
		int* arrx = new int[radius_limit];		//Dynamic memory, can change number of points interested in using
		int* arry = new int[radius_limit];		//Kinda like resolution. More points can make it a line

		for (int radius = 0; radius < radius_limit; radius += radius_jump)
		{

			arrx[radius] = x1 + radius * cos(theta_index);
			arry[radius] = y1 + radius * sin(theta_index);
			
			//draw_point_rgb(view.return_image(), arrx[radius], arry[radius], 255, 0, 0);
		}

		hide_shadows(arrx, arry, view, theta_index, radar_radius, radius_limit, enemy_trigger, enemy, radius_jump, shadow_zone_trigger);
		//cout << radar_radius << endl;


		for (int radius = radar_minimum; radius < radar_radius; radius+=radius_jump)
		{

			if (arrx[radius] < 0 || arrx[radius] > view.return_a().width || arry[radius] < 0 || arry[radius] > view.return_a().height)
			{
				break;
			}
			if (flag_draw == 1) draw_point_rgb(view.return_image(), arrx[radius], arry[radius], 255, 0, 0);

			if (enemy_trigger == 1)
			{
				multiplier = enemy_multiplier;
				if (flag_draw == 1) draw_point_rgb(view.return_image(), arrx[radius], arry[radius], 0, 255, 255);
			}

			if (shadow_zone_trigger == 1)
			{
				multiplier = shadow_multiplier;
				if (flag_draw == 1) draw_point_rgb(view.return_image(), arrx[radius], arry[radius], 0, 255, 255);
			}
		}

		vector_x += radar_radius * cos(theta_index) * multiplier;
		vector_y += radar_radius * sin(theta_index) * multiplier;

		counter_vector_x += radius_limit * cos(theta_index) * multiplier;
		counter_vector_y += radius_limit * sin(theta_index) * multiplier;

		delete[]arrx;
		delete[]arry;
	}

	//cout << vector_x << "  " << vector_y << endl;

	counter_vector_x = -counter_vector_x;
	counter_vector_y = -counter_vector_y;

	vector_x += counter_vector_x;
	vector_y += counter_vector_y;

	double resultant_theta;
	double resultant_mag = sqrt(pow((vector_x), 2) + pow((vector_y), 2));

	calculate_theta(vector_x, vector_y, 0, 0, resultant_theta);
	//cout << resultant_theta << endl;

	VFF_theta = resultant_theta;
	VFF_mag = resultant_mag;

	int x_draw, y_draw;
	for (int radius = radar_minimum; radius < resultant_mag; radius += radius_jump)
	{
		x_draw = x1 + radius * cos(resultant_theta);
		y_draw = y1 + radius * sin(resultant_theta);

		if (x_draw < 0 || x_draw > view.return_a().width || y_draw < 0 || y_draw > view.return_a().height)
		{
			break;
		}

		if (flag_draw == 1) draw_point_rgb(view.return_image(), x_draw, y_draw, 0, 255, 0);
	}
}

void PT11::highlight_view_evade(Camera& view, PT11 enemy)
{
	bool flag_draw = 0;
	bool flag_draw_shadow = 0;
	int radar_radius = 0;
	double theta_index = 0;
	double theta_jump = 0.06;
	int radar_minimum;
	int radius_jump = 5;
	int radius_limit;
	double vector_x = 0;
	double vector_y = 0;
	double counter_vector_x = 0;
	double counter_vector_y = 0;
	bool enemy_trigger = 0;
	double multiplier;
	double enemy_multiplier;
	bool shadow_zone_trigger = 0;
	double shadow_multiplier;

	int Area = 0;
	int Ax = 0;
	int Ay = 0;


	const int shadow_max = 50;
	int shadow_x[shadow_max];
	int shadow_y[shadow_max];
	int shadow_A[shadow_max];

	int shadow_index = 0;

	//int full_shadow_centroid_index = 0;


	while (theta_index < (2 * M_PI))
	{
		if (evade_trigger == 1)
		{
			flag_draw_shadow = 1;
		}
		else
		{
			flag_draw_shadow = 0;
		}

		radar_minimum = 0;
		radius_limit = 500;
		multiplier = 1;
		enemy_multiplier = 1.0;
		shadow_multiplier = -300.0;
		enemy_trigger = 0;
		shadow_zone_trigger = 0;
		theta_index = theta_index + theta_jump;

		double sym_offset;
		double sym_size;
		double sym_radius;
		double sym_multi;

		/*

		double temp = 0.5;
		sym_offset = M_PI/2 + temp /2.0;
		sym_size = temp;
		sym_radius = 70;
		sym_multi = 1.0;
		VFF_section_modifier(theta_index, sym_offset, sym_size, radius_limit, sym_radius, multiplier, sym_multi);
		VFF_section_modifier(theta_index, 2*M_PI - sym_offset, sym_size, radius_limit, sym_radius, multiplier, sym_multi);


		sym_offset = M_PI;
		sym_size = 0.5;
		sym_radius = 130;
		sym_multi = 1.0;
		VFF_section_modifier(theta_index, sym_offset, sym_size, radius_limit, sym_radius, multiplier, sym_multi);
		//VFF_section_modifier(theta_index, 2 * M_PI - sym_offset, sym_size, radius_limit, sym_radius, multiplier, sym_multi);
		*/

		double theta_bracket_1 = theta + 0 + M_PI + 1.0;
		double theta_bracket_2 = theta + 0 + M_PI - 1.0;

		radar_radius = 0;
		int* arrx = new int[radius_limit];		//Dynamic memory, can change number of points interested in using
		int* arry = new int[radius_limit];		//Kinda like resolution. More points can make it a line

		for (int radius = 0; radius < radius_limit; radius += radius_jump)
		{

			arrx[radius] = enemy.x1 + radius * cos(theta_index);
			arry[radius] = enemy.y1 + radius * sin(theta_index);

			//draw_point_rgb(view.return_image(), arrx[radius], arry[radius], 255, 0, 0);
		}

		hide_shadows_evade(arrx, arry, view, theta_index, radar_minimum, radar_radius, radius_limit, enemy_trigger, enemy, radius_jump);
		//cout << radar_radius << endl;
		//cout << shadow_zone_trigger << endl;

		//if (radar_minimum > radar_radius) radar_minimum = radar_radius;

		for (int radius = radar_minimum; radius < radar_radius; radius += radius_jump)
		{
			
			if (arrx[radius] < 0 || arrx[radius] > view.return_a().width || arry[radius] < 0 || arry[radius] > view.return_a().height)
			{
				continue;
			}

			if (flag_draw == 1) draw_point_rgb(view.return_image(), arrx[radius], arry[radius], 255, 0, 0);
			Area++;

			Ax += 1 * arrx[radius];
			Ay += 1 * arry[radius];


		}


		//if (Area == 0) Area = 1;

		int centroid_x;
		int centroid_y;

		if (Area != 0 && Area < 10000)
		{

			centroid_x = Ax / Area;
			centroid_y = Ay / Area;

			//cout << Area << endl;

			int box_length = 40;
			int tolerance = 30;

			for (int i = -box_length / 2; i < box_length; i++)
			{
				for (int j = -box_length / 2; j < box_length; j++)
				{
					if (centroid_x + i < 0 + tolerance || centroid_x + i > view.return_a().width - tolerance || centroid_y + j < 0 + tolerance || centroid_y + j > view.return_a().height - tolerance)
					{
						continue;
					}
					else
					{
						if (flag_draw_shadow == 1) draw_point_rgb(view.return_image(), centroid_x + i, centroid_y + j, 255, 0, 255);
					}
					
				}
			}

			//if (flag_draw == 1) draw_point_rgb(view.return_image(), centroid_x, centroid_y, 0, 255, 0);

			shadow_x[shadow_index] = centroid_x;
			shadow_y[shadow_index] = centroid_y;
			//shadow_A[shadow_index] = Area;
			shadow_A[shadow_index] = 1;
			//cout << Area << endl;
			shadow_index++;
			//if (shadow_index > shadow_max) shadow_index = 0;
		}

		Area = 0;
		Ax = 0;
		Ay = 0;

		//int mid_point = (radar_minimum + radar_radius) / 2;
		//if (flag_draw == 1) draw_point_rgb(view.return_image(), arrx[mid_point], arry[mid_point], 0, 255, 0);

		vector_x += radar_radius * cos(theta_index) * multiplier;
		vector_y += radar_radius * sin(theta_index) * multiplier;

		counter_vector_x += radius_limit * cos(theta_index) * multiplier;
		counter_vector_y += radius_limit * sin(theta_index) * multiplier;

		delete[]arrx;
		delete[]arry;

	}


}


void PT11::VFF_section_modifier(double theta_index, double offset, double range, int& radius_limit, int limit_val, double& multiplier, double multiplier_val)
{

	double theta_bracket_1 = theta + offset + range;
	double theta_bracket_2 = theta + offset - range;

	while (theta_bracket_1 > 2 * M_PI)
	{
		theta_bracket_1 -= 2 * M_PI;
	}

	while (theta_bracket_2 > 2 * M_PI)
	{
		theta_bracket_2 -= 2 * M_PI;
	}

	if (theta_bracket_1 < theta_bracket_2)
	{
		theta_bracket_1 = theta_bracket_1 + 2 * M_PI;

	}

	//cout << theta << "    " << theta_bracket_1 << "   " << theta_bracket_2 << endl;
	if (theta_index + 2 * M_PI < theta_bracket_1 && theta_index + 2 * M_PI > theta_bracket_2 ||
		theta_index < theta_bracket_1 && theta_index > theta_bracket_2)
	{
		radius_limit = limit_val;
		multiplier = multiplier_val;
	}
}


void PT11::hide_shadows(int arrx[], int arry[], Camera& view, double theta_index, int& radar_radius, int radius_limit, bool& enemy_trigger, PT11 enemy, int radius_jump, bool& shadow_zone_trigger)
{
	int what_label;

	copy(view.return_image(), view.return_a());

	ibyte* pa;

	pa = view.return_a().pdata;

	int* k = new int[radius_limit];	//Can vary length of point series for collision accuracy


	for (int i2 = 0; i2 < radius_limit; i2 += radius_jump)	//For each dot on the line of points, find position based on 1D image reference
	{
		if (arrx[i2] > 0 && arrx[i2] < view.return_a().width && arry[i2] > 0 && arry[i2] < view.return_a().height)
		{
			k[i2] = arrx[i2] + view.return_a().width * arry[i2];
		}
		else {
			k[i2] = 1 + view.return_a().width;
		}
	}

	for (int i2 = 0; i2 < radius_limit; i2 += radius_jump)	//If either point has 255 at pointer, turn on collision state
	{

		if (arrx[i2] < 0 || arrx[i2] > view.return_a().width || arry[i2] < 0 || arry[i2] > view.return_a().height)
		{
			pa[k[i2]] = 255;
			//radar_radius = i2;
			break;
		}

		what_label = view.label_at_coordinate(arrx[i2], arry[i2]);

		if (what_label == label_nb_1 || what_label == label_nb_2)
		{
			continue;
		}


		if ((what_label == enemy.label_nb_1 || what_label == enemy.label_nb_2))
		{
			enemy_trigger = 1;
		}

		//cout << (int)pa[k[i2]] << "\t" << i2 << endl;
		if (pa[k[i2]] == 255)
		{
			//cout << i2 << endl;
			//distance_log[i] = i2;
			radar_radius = i2;
			break;
		}
		else
		{
			radar_radius = radius_limit;
			//distance_log[i] = Ln[i];

		}
	}


	int k2;
	ibyte R, G, B;
	ibyte* p, * pc;

	p = view.return_image().pdata;

	for (int i2 = 0; i2 < radius_limit; i2 += radius_jump)
	{
		k2 = arrx[i2] + view.return_image().width * arry[i2];
		pc = p + 3 * k2; // pointer to the kth pixel (3 bytes/pixel)

		B = *pc;
		G = *(pc + 1);
		R = *(pc + 2);

		if (arrx[i2] < 640 && arrx[i2] > 0 && arry[i2] < 480 && arry[i2] > 0)
		{
			if ((B == 255) && (R == 255) && (G == 0))
			{
				radar_radius = i2;
				//cout << i2 << endl;
				shadow_zone_trigger = 1;
				//cout << shadow_zone_trigger << endl;
				//cout << "AAAAAAAAAAAAAAAA HIDE!" << endl;
			}
		}
		
	}
	delete[]k;

}


void PT11::hide_shadows_evade(int arrx[], int arry[], Camera& view, double theta_index, int& radar_minimum, int& radar_radius, int radius_limit, bool& enemy_trigger, PT11 enemy, int radius_jump)
{
	int what_label;

	copy(view.return_image(), view.return_a());

	ibyte* pa;

	pa = view.return_a().pdata;

	int* k = new int[radius_limit];	//Can vary length of point series for collision accuracy


	for (int i2 = 0; i2 < radius_limit; i2 += radius_jump)	//For each dot on the line of points, find position based on 1D image reference
	{
		if (arrx[i2] > 0 && arrx[i2] < view.return_a().width && arry[i2] > 0 && arry[i2] < view.return_a().height)
		{
			k[i2] = arrx[i2] + view.return_a().width * arry[i2];
		}
		else {
			k[i2] = 1 + view.return_a().width;
		}
	}

	bool transition_trigger = 0;

	for (int i2 = 0; i2 < radius_limit; i2 += radius_jump)	//If either point has 255 at pointer, turn on collision state
	{

		double tol = 5;

		if (arrx[i2] < 0 + tol || arrx[i2] > view.return_a().width - tol || arry[i2] < 0 + tol || arry[i2] > view.return_a().height - tol)
		{
			//radar_radius = i2;
			break;
		}

		/*

		if (arrx[i2] < 0 || arrx[i2] > view.return_a().width || arry[i2] < 0 || arry[i2] > view.return_a().height)
		{
			pa[k[i2]] = 255;
			//radar_radius = i2;
			break;
		}
		*/
		

		what_label = view.label_at_coordinate(arrx[i2], arry[i2]);

		if (what_label == enemy.label_nb_1 || what_label == enemy.label_nb_2)
		{
			continue;
		}

		if (what_label == label_nb_1 || what_label == label_nb_2)
		{
			continue;
		}

		/*

		if ((what_label == enemy.label_nb_1 || what_label == enemy.label_nb_2))
		{

			enemy_trigger = 1;
		}
		*/

		//cout << (int)pa[k[i2]] << "\t" << i2 << endl;

		

		if (pa[k[i2]] == 255)
		{
			transition_trigger = 1;
			//cout << i2 << endl;
			//distance_log[i] = i2;
			//radar_radius = i2;
			//break;
		}

		radar_minimum = 500;

		if (pa[k[i2]] == 0 && transition_trigger == 1)
		{
			//radar_radius = i2;
			radar_minimum = i2;
			break;
		}

		if (pa[k[i2]] == 0)
		{
			//radar_radius = i2;
			//radar_radius = radius_limit;
			radar_radius = radius_limit;
		}

		/*
		else
		{
			radar_radius = radius_limit;
			//radar_minimum = radius_limit - 1;
			//distance_log[i] = Ln[i];
		}
		*/
	}


	delete[]k;

}



void PT11::label_enemy(Camera& view, PT11 enemy)
{
	
}


void PT11::acquire_camera_image(Camera& view) {
	//Initializes all PT11 image objects. These image objects are used for processing

	copy(view.return_image(), radar_rgb);		//This will house an RGB image with the mask that will be used for processing of safe zones

												//***NOTE2: CURRENTLY THIS FUNCTION IS ONLY GRABBING THRESHOLDED IMAGE after calling case 1 in main, placement of this function in main is important

	copy(view.return_a(), radar_greyscale);		//Storing image "a" which is a binary thresholded image (I placed it in main like that)


}


void PT11::draw_safe_zone(int* line_array_i, int* line_array_j, int size, Camera& view, PT11& enemy) {
	//This function performs the radar processing of determining the safe zones. It creates an RGB mask of the safe zone which can be used in other processing functions. 
	//Uses Camera object's "label" label image, PT11 object's "radar_rgb" rgb image, and PT11 object's "radar_greyscale" greyscale object.  

	ibyte* p_greyscale, * p_rgb, R, G, B;	//This will iterate through the binary image, perform logic to understand which pixels should be considered green. RGB will be used in view->return_image()
	i2byte* p_label;
	int i, j, x, y, k, size1;
	int flag1 = 0;		//Triggers when white pixel is seen, this means following black pixels are safe zones
	//int flag2 = 0;		//Triggers when safe zone starts, this indicates new white pixel has been observed along line, which is not a safe zone because we can't enter white pixel areas (obstacle)
	size1 = size;

	p_greyscale = radar_greyscale.pdata;	//Points to binary image of the radar
	//p_rgb = view.return_image().pdata;		//Points to LIVE rgb image UPDATE: Make it point to radar rgb image to create mask
	p_label = (i2byte*)view.return_label().pdata;	//Points to label image from camera class, since label_nb_1 and label_nb_2 are labels from this image
	p_rgb = radar_rgb.pdata;					//Mask is created on radar_rgb, holding C will copy the mask to the viewable "rgb" image in Camera class (written in at the end of get_safe_zone())


	for (i = 70; i < size1; i++) {
		//This for-loop is in place to increment through the pixels stored in the line aray from get_safe_zone. Starting at 70 means mask is starting at a location 70 pixels further from centroid
		x = line_array_i[i];		//x-coordinate of pixel 
		y = line_array_j[i];		//y-coordinate of pixel

		k = x + (y * 640);		//k-coordinate of pixel NOTE: I would use view.width instead of 640 BUT its sussy... just gonna put 640 LOL

		if (p_greyscale[k] < 50) { //detecting a black pixel in the line equation indicating an object
			if (flag1 == 1) {
				//Processing will only trigger for the first black pixel observed after white pixels ie: white pixels have been detected earlier in the line, triggering flag1, safe zone!
				p_rgb[(k * 3)] = 50; //B
				p_rgb[(k * 3) + 1] = 205; //G
				p_rgb[(k * 3) + 2] = 50; //R
				//flag2 = 1;		//black pixels have been observed, so next time white pixels appear = stop safe zone (not necessary, safe zone is only written for black pixels, and white object is covering those black pixels anyways)
			}
		}

		if (p_greyscale[k] > 240) {
			if (p_label[k] != label_nb_1 && p_label[k] != label_nb_2 && p_label[k] != enemy.label_nb_1 && p_label[k] != enemy.label_nb_2) {
				flag1 = 1;			//white pixels have been detected (ie: an object is observed), next time black pixels appear = safe zone (ie: the location after the object)
			}
		}
	}
}

void PT11::get_safe_zone(Camera& view, PT11& enemy, int pt_i[4], int pt_j[4]) {
	//This function creates lines that increment from the centroid of robot to the right/left wall, then top/bottom wall. Once the pixels are stored in the array, 
	//we now have access to i,j value of *most* of every pixel in the image *organized from the centroid of the robot* to the walls. 
	//This will let us process the safe zones and other process functions that are *robot-location sensitive*.

	//IMPORTANT: Right now I set it so the radar stops 20/30 pixels away from the borders of the walls. Also, the mask only starts 70 pixels away from the centroid of robot.
	//This can be adjusted easily.

	int i, j;		//Store the i and j pixel values of the radar
	double x0, y0, x1, y1;		//Store centroid of robot and the final point of the radar sweep
	double delta_x, delta_y;
	double slope, b;
	int height, width;		//Unnecessary if this becomes a class function, just use height and width class variable
	int* line_array_i, * line_array_j;		//Store pixel coordinates in these dynamic arrays, delete at the end
	int size;		//Stores size of dynamic array
	int increment;		//Used to increment through dynamic arrays to print rgb points
	double border_x, border_y;		//Used in greater for-loop controlling radar around borders
								//These values are the final points of the radar line segment, they will change according to location of robot and required sweep

	width = 640;
	height = 480;
	line_array_i = new int[1000];
	line_array_j = new int[1000];
	size = 0;

	x0 = enemy.get_x1();// pt_i[1];		//This is the centroids of the "enemy" bot, the one that we are trying to evade
	y0 = enemy.get_y1();// pt_j[1];


	for (border_y = 0; border_y < 480; border_y++) {
		//Scanning through RIGHT-BORDER, all lines from centroid to wall

		border_x = 640;		//Initialize border_x for this sweep NOTE:(should be in a loop or something eventually, since border will change to 0 at some point)

		if (border_y == y0) {
			for (i = x0; i < border_x - 30; i++) {
				//Iterate through all x-values for each line, LATER: account for vertical leaning lines
				j = y0;

				line_array_i[size] = i;
				line_array_j[size] = j;

				size++;
			}
		}
		else {
			delta_x = border_x - x0;
			delta_y = border_y - y0;

			slope = delta_y / delta_x;
			b = y0 - (slope * x0);

			for (i = x0; i < border_x - 30; i++) {
				//Iterate through all x-values for each line, LATER: account for vertical leaning lines
				j = int((slope * i) + b);

				line_array_i[size] = i;
				line_array_j[size] = j;

				size++;
			}
		}

		draw_safe_zone(line_array_i, line_array_j, size, view, enemy);		//SHOULD DRAW LIME GREEN SAFE ZONES :O

		size = 0;		//Very important, reset array back to 0 element for next line

		//***I am adding line-drawing of left border in this for-loop because it uses the same y-range***

		border_x = 0;	//Left of screen

		//Scanning through LEFT-BORDER, all lines from centroid to wall

		if (border_y == y0) {
			for (i = x0; i > border_x+30; i -= 1) {
				//Iterate through all x-values for each line (Lines start from centroid as if laser is shooting out)
				j = y0;

				line_array_i[size] = i;
				line_array_j[size] = j;

				size++;
			}
		}
		else {

			delta_x = border_x - x0;
			delta_y = border_y - y0;

			slope = delta_y / delta_x;
			b = y0 - (slope * x0);

			for (i = x0; i > border_x+30; i -= 1) {
				//Iterate through all x-values for each line (Lines start from centroid as if laser is shooting out)
				j = int((slope * i) + b);

				line_array_i[size] = i;
				line_array_j[size] = j;

				size++;
			}
		}

		draw_safe_zone(line_array_i, line_array_j, size, view, enemy);		//SHOULD DRAW LIME GREEN SAFE ZONES :O

		size = 0;		//Very important, reset array back to 0 element for next line

	}

	for (border_x = 0; border_x < 640; border_x++) {
		//Scanning through TOP-BORDER, all lines from centroid to wall

		border_y = 480;		//Initialize border_x for this sweep NOTE:(should be in a loop or something eventually, since border will change to 0 at some point)

		if (border_x == x0) {
			for (j = y0; j < border_y - 20; j++) {
				//Iterate through all x-values for each line, LATER: account for vertical leaning lines
				i = x0;

				line_array_i[size] = i;
				line_array_j[size] = j;

				size++;
			}
		}
		else {
			delta_x = border_x - x0;
			delta_y = border_y - y0;

			slope = delta_x / delta_y;
			b = x0 - (slope * y0);

			for (j = y0; j < border_y - 20; j++) {
				//Iterate through all x-values for each line, LATER: account for vertical leaning lines
				i = int((slope * j) + b);

				line_array_i[size] = i;
				line_array_j[size] = j;

				size++;
			}
		}

		draw_safe_zone(line_array_i, line_array_j, size, view, enemy);		//SHOULD DRAW LIME GREEN SAFE ZONES :O

		size = 0;		//Very important, reset array back to 0 element for next line

		//I am adding line-drawing of bottom border in this for-loop because it uses the same y-range

		border_y = 0;		//Initialize border_x for this sweep NOTE:(should be in a loop or something eventually, since border will change to 0 at some point)


		//Scanning through BOTTOM-BORDER, all lines from centroid to wall

		if (border_x == x0) {
			for (j = y0; j > border_y + 20; j -= 1) {
				//Iterate through all x-values for each line, LATER: account for vertical leaning lines
				i = x0;

				line_array_i[size] = i;
				line_array_j[size] = j;

				size++;
			}
		}
		else {

			delta_x = border_x - x0;
			delta_y = border_y - y0;

			slope = delta_x / delta_y;
			b = x0 - (slope * y0);

			for (j = y0; j > border_y+20; j -= 1) {
				//Iterate through all x-values for each line, accounts for vertical leaning lines
				i = int((slope * j) + b);

				line_array_i[size] = i;
				line_array_j[size] = j;

				size++;
			}
		}

		draw_safe_zone(line_array_i, line_array_j, size, view, enemy);		//SHOULD DRAW LIME GREEN SAFE ZONES :O

		size = 0;		//Very important, reset array back to 0 element for next line

	}


	//THUS: RGB image mask, radar_rgb, of safe zone is initialized by draw_safe_zone() by this point.
	
	threshold_radar(view, enemy, pt_i, pt_j);
	//THUS: safezone_greyscale binary image is initialized by threshold_radar(), safezones are now white and everything else should be black in "safezone_greyscale"

	//assess_safe_zone() This function will label safezone_greyscale, placing it into safezone_label label image. It will then find the centroids of each labelled safe zone!
	assess_safe_zone(view);

	//radar_evasion(pt_i, pt_j, enemy); Not needed, using VFF solutions

	if (KEY('V')) {
		copy(safezone_greyscale, radar_rgb);
		copy(radar_rgb, view.return_image());
	}

	if (KEY('C')) {
		copy(radar_rgb, view.return_image());
	}



	delete[] line_array_i;
	delete[] line_array_j;
}

void PT11::threshold_radar(Camera& view, PT11& enemy, int pt_i[4], int pt_j[4]) {
//Taking all pixel values of radar_rgb RGB image and converting lime green safezone pixels to black/white in safezone_greyscale
//All safezone pixel colors will be black, all other pixels will be white
	ibyte* p_rgb, * p_greyscale;
	i2byte* p_label;
	int i, j, k1, k2;
	p_rgb = radar_rgb.pdata; //Increment this pointer through the rgb image "radar_rgb" which contains the lime green safezone pixels, and access the pixel values
	p_greyscale = safezone_greyscale.pdata;	//Once safezone pixels are detected, draw them as 255 value white pixels. (SHOULD BE 0 BLACK BUT NOT INVERTING so we put it white)
	p_label = (i2byte*)view.return_label().pdata;
	int size1;
	size1 = 640 * 480;

	k1 = pt_i[1] + 640 * pt_j[1];	//calculating the k pixel value of the centroid of the robot, will check if it's white which indicates its in a safe zone
	k2 = pt_i[3] + 640 * pt_j[3];

	/* This works too
	for (j = 0; j < 480; j++) {
		for (i = 0; i < 640; i++) {
			k = i + j * 640;
			if (p_rgb[k * 3] == 50 && p_rgb[(k * 3) + 1] == 205 && p_rgb[(k * 3) + 2] == 50) {
				p_greyscale[k] = 255;	//Safezone pixel will be white (no inversion or clean up processing like erosion/dialate happening yet)
			}
			else p_greyscale[k] = 0;	//every non safezone pixel
		}
	} */

	for (i = 0; i < size1; i++) {
		if (p_rgb[i * 3] == 50 && p_rgb[(i * 3) + 1] == 205 && p_rgb[(i * 3) + 2] == 50) {
			p_greyscale[i] = 255;	//Safezone pixel will be white (no inversion or clean up processing like erosion/dialate happening)
		}
		else p_greyscale[i] = 0;
	}

	
	
	for (i = 0; i < size1; i++) {
		//Solution 1 to preventing robot from appearing black in safe zone: if centroid is in safe zone, make robot white like safe zone

		 if (p_greyscale[i] == 0) {
			//Checking if pixel is black or white in safezone mask (ie: the binary image that only depicts the safe zone), we only care about black pixels as the robot will appear black in safe zone
			 if (p_label[i] == enemy.label_nb_1) {
				 //Checking if the black pixel is apart of the friendly robot (in this case we're using the friendly robot as the danger, thats why we are usign enemy label)
				 if (p_greyscale[k1] == 255 || p_greyscale[k1 - 10] == 255 || p_greyscale[k1 + 10] == 255 || p_greyscale[k1 + (640 * 10)] == 255 || p_greyscale[k1 - (640 * 10)] == 255) {
					 //If the centroid of the robot is in the safe zone, all pixels of that robot should be white
					 p_greyscale[i] = 255;
				 }
			 }
			 if (p_label[i] == enemy.label_nb_2) {
				 if (p_greyscale[k2] == 255 || p_greyscale[k2 - 10] == 255 || p_greyscale[k2 + 10] == 255 || p_greyscale[k2 + (640 * 10)] == 255 || p_greyscale[k2 - (640 * 10)] == 255) {
					 //If the centroid of the robot is in the safe zone, all pixels of that robot should be white
					 p_greyscale[i] = 255;
				 }
			}

		} 


		/* Not a good idea: too much noise, which results in more objects
		//Solutiont 2 to preventing robo from appearing in black safe zone: if robot pixel is near safe zone, make it white
		 if (p_greyscale[i] == 0) {
			 if (p_label[i] == enemy.label_nb_1 || p_label[i] == enemy.label_nb_2) {
				 //Checking if the black pixel is apart of the friendly robot (in this case we're using the friendly robot as the danger, thats why we are usign enemy label)
				 if (p_rgb[(i * 3)+30] == 50 && p_rgb[((i * 3) + 1)+30] == 205 && p_rgb[((i * 3) + 2)+30] == 50 || p_rgb[(i * 3) - 30] == 50 && p_rgb[((i * 3) + 1) - 30] == 205 && p_rgb[((i * 3) + 2) - 30] == 50 || p_rgb[(i * 3) + (1920 * 10)] == 50 && p_rgb[((i * 3) + 1) + (1920 * 10)] == 205 && p_rgb[((i * 3) + 2) + (1920 * 10)] == 50 || p_rgb[(i * 3) - (1920 * 10)] == 50 && p_rgb[((i * 3) + 1) - (1920 * 10)] == 205 && p_rgb[((i * 3) + 2) - (1920 * 10)] == 50) {
					 p_greyscale[i] = 255;	//Safezone pixel will be white (no inversion or clean up processing like erosion/dialate happening yet)
				 }
			 }
		 } */
	}

	//perform post processing

	copy(safezone_greyscale, radar_a);
	erode(radar_a, radar_b);			//Erode
	copy(radar_b, radar_a);
	dialate(radar_a, radar_b);			//Dilate
	copy(radar_b, radar_a);
	erode(radar_a, radar_b);			//Erode
	copy(radar_b, radar_a);
	dialate(radar_a, radar_b);			//Dilate
	copy(radar_b, radar_a);

	copy(radar_a, safezone_greyscale);

	//Results in  safezone_greyscale image being completed: This image makes the safe zone white, everything else black, begin processing to reach these safe zones

}

void PT11::assess_safe_zone(Camera& view) {
	//This function will label the newly obtained binary image "safezone_greyscale", effectively labelling all the safe zones.
	//It will determine the centroids of the safe zones, and store them in an array.
	//There is a filter in radar_centroid() called "number_of_pixels", which manipulates variable "flag", which determines if the centroid of an object should be stored.
	//The centroid of an object will be stored in the array only if the object involved more than 500 pixels (can adjust this), ensures the only certain safe zones matter.
	//We will use these array of centroids for further processing ie: which centroids are closer?? etc.
	double i, j;	//These variables will house the actual centroids of the safe zone objects in the labelled image, safezone_label
	int k;			//This variable is used for incrementing through the loop, through each label number to assess each labelled object
	
	int box_length = 80;
	int tolerance = 30;
	int box_i, box_j;

	int flag = 0;	//This flag will be set to 0 or 1 in radar_centroid(), it controls whether a labelled object in safezone_label is big enough to be considered a safezone
					//If it is, flag = 1, and centroids will be stored and array will increment. If not, then it was probably a stray pixel or noise, ignore.
					//Current minimum size of safe zone is 500 pixels (number_of_pixels variable in radar_centroid() ), random number I picked!
	safezone_array_index = 0;
	label_image(safezone_greyscale, safezone_label, radar_nlabels);	//radar_nlabels is an int variable which says number of safe zones that are labelled 
																	//I made a safezone_label...
	k = 1;
	//cout << "\n number of safe zones detected: " << radar_nlabels;	//Checking if number of safe zones is being recorded properly after labelling binary image
	for (k; k <= radar_nlabels; k++) {
		//This for loop is calculating the centroids of safezones that meet a minimum requirement of pixels, then adds those centroids to the array. Flag is used for that purpose.
		//Flag is reset to 0 whenever radar_centroid() is called, and triggered to 1 if the labelled safe zone is big enough
		radar_centroid(safezone_greyscale, safezone_label, k, i, j, flag);
		if (flag == 1) {
			safezone_centroid_x[safezone_array_index] = (int)i;
			safezone_centroid_y[safezone_array_index] = (int)j;
			draw_point_rgb(radar_rgb, i, j, 255, 0, 0);		//Visualizes the point

			for (box_i = -box_length / 2; box_i < box_length/2; box_i++) {
				for (box_j = -box_length / 2; box_j < box_length/2; box_j++) {
					if (safezone_centroid_x[safezone_array_index] + box_i < 0 + tolerance || safezone_centroid_x[safezone_array_index] + box_i > 640 - tolerance || safezone_centroid_y[safezone_array_index] + box_j < 0 + tolerance || safezone_centroid_y[safezone_array_index] + box_j > 480 - tolerance) {
						continue;
					}
					else {
						draw_point_rgb(view.return_image(), safezone_centroid_x[safezone_array_index] + box_i, safezone_centroid_y[safezone_array_index] + box_j, 255, 0, 255);
					}
				}
			}

			safezone_array_index++;
		}
	}

	/* Testing, tracks centroid location in console
	* //cout << endl << safezone_array_index;
	for (int l = 0; l < safezone_array_index; l++) {
		cout << "\nCentroid " << l << "\tx: " << safezone_centroid_x[l] << "\t y: " << safezone_centroid_y[l];
	}
	*/
}

int PT11::radar_centroid(image& a, image& label, int nlabel, double& ic, double& jc, int &flag) {
	// calculate the greyscale centroid of a labelled object
// a - GREY_IMAGE type
// label - LABEL_IMAGE type (pixel values 0-65535)
// nlabel - label number
// ic - i centroid coordinate (x direction / right)
// jc - j centroid coordinate (y direction / up)

	//ADJUSTED -> Count number of pixels used per centroid, we only care for the labelled objects with lots of pixels
	{
		ibyte* pa;
		i2byte* pl;
		i4byte i, j, width, height;
		double mi, mj, m, rho;
		int number_of_pixels;	//This counter tracks the number of pixels per labelled object, this will filter out only the centroids that matter

		number_of_pixels = 0;	//This value will judge whether a safezone labelled object is big enough or not!
		flag = 0;	//Flag indicating if the centroid should be recorded or not
		// check for compatibility of a, label
		if (a.height != label.height || a.width != label.width) {
			cout << "\nerror in centroid: sizes of a, label are not the same!";
			return 1;
		}

		if (a.type != GREY_IMAGE || label.type != LABEL_IMAGE) {
			cout << "\nerror in centroid: input types are not valid!";
			return 1;
		}

		pa = a.pdata;
		pl = (i2byte*)label.pdata;

		// number of pixels
		width = a.width;
		height = a.height;

		mi = mj = m = 0.0;

		for (j = 0; j < height; j++) { // y-dir
			for (i = 0; i < width; i++) { // x-dir
				if (pl[j * width + i] == nlabel) {
					rho = pa[j * width + i];
					m += rho;
					// assume pixel has area of 1 so m = rho * A = rho
					mi += rho * i;
					mj += rho * j;
					number_of_pixels++; //Increment, counts number of pixels being added for each labelled safe zone object
				}
			}
		}

		if (number_of_pixels > 1000) {	//Right now the minimum size of an eligible safe zone is 400 pixels
			ic = mi / m;
			jc = mj / m;
			flag = 1; //Trigger that it is allowed to be stored;
		}

		return 0;
	}
}

void PT11::radar_evasion(int pt_i[4], int pt_j[4], PT11& enemy) {
	//This function processes each centroid, determines which one is the appropriate centroid to follow, and initiates evasion. Will use VFF method instead of this function.
	int i;
	double x11, y11; //This variable stores the location of the robot centroid
	double x22, y22; //This variable stores the location of the safe zone centroid
	int best_index; //This variable will be used to store the array element index of the best centroid (ie: closest and most appropriate) in safezone_centroid_x[]/safezone_centroid_y[]
	int best_magnitude; //This variable will help with logic to determine the best centroid
	double magnitude; //This value will calculate and store the distance between the centroid of the robot and centroid of safe zones

	best_index = 0;
	best_magnitude = 800; //Start at a high value, replace whenever a shorter distance is input
	x11 = pt_i[1]; //Front wheels centroid
	y11 = pt_j[1];
	x22 = safezone_centroid_x[0];
	y22 = safezone_centroid_y[0];

	for (i = 0; i < safezone_array_index; i++) {
		//Increment through each safe zone centroid to find the closest one.
		x22 = safezone_centroid_x[i];
		y22 = safezone_centroid_y[i];

		magnitude = sqrt((pow((x22 - x11), 2) + pow((y22 - y11), 2)));
		if (magnitude < best_magnitude) {
			best_magnitude = magnitude;
			best_index = i; //Store the index of the closest centroid
			//Thus, we will use best_index in safezone_centroid_x/y to do processing
		}
	}
	//cout << "\nThe distance to the closest centroid is: " << best_magnitude;  //Check if closest centroid is being recorded into best_magnitude, subsequent best_index too
	cout << "\n The best index is: " << best_index;
	//***MUST ADD SAFE CHECK TO SEE IF OBJECT IS BETWEEN ROBOT AND SAFEZONE CENTROID***
	/*
	double target_angle; //angle of line between centroids
	double delta_angle;	//difference between angle of robot and line to centroid
	x2 = safezone_centroid_x[best_index];
	y2 = safezone_centroid_y[best_index];
	calculate_theta(x11, y11, x22, y22, target_angle);
	delta_angle = enemy.theta - target_angle;

	if (delta_angle > 10) {
		enemy.pw_l = 1050;
		enemy.pw_r = 1050;
	}
	else if (delta_angle < -10) {
		enemy.pw_l = 1950;
		enemy.pw_r = 1950;
	}
	else if (-10<=delta_angle <=10) {
		enemy.pw_l = 1000;
		enemy.pw_r = 2000;
	} */
}

void PT11::enemy_out_of_map(PT11 enemy)
{
	double robot_elongation = sqrt(pow(enemy.get_x1() - enemy.get_x2(), 2) + pow(enemy.get_y1() - enemy.get_y2(), 2));

	if (robot_elongation > 85 || robot_elongation < 75){ disable_system = 1; }
	else { disable_system = 0; }

	//cout << disable_system << endl;
}

PT11::~PT11()
{

}
