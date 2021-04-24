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
#include "NeuralNet.h"

#include "Neural_Network/NeuroNet.h"
#include "Neural_Network/Input.h"
#include "Neural_Network/Hidden.h"
#include "Neural_Network/Output.h"


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

	for (int i = 0; i < 8; i++)
	{
		distance_log[i] = 0;
	}

	trial_timer1 = high_resolution_time();

	is_there_obstacle = 0;

	label_nb_1 = 0;
	label_nb_2 = 0;

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

	//cout << this->pw_l << "\t" << this->pw_r << endl;

	pw_r = this->pw_r;
	pw_l = this->pw_l;
	laser = 0;

	/*
	if (KEY('A')) pw_laser += 100;
	if (KEY('D')) pw_laser -= 100;
	*/
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

	Lx[0] = 31;		Ly[0] = 0;		LL[0] = 20;		Ln[0] = 4;
	Lx[1] = -42;	Ly[1] = -34;	LL[1] = 40;		Ln[1] = 4;
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
		check_collision(arrx, arry, view, i);

		delete[]arrx;
		delete[]arry;
	}

}

void PT11::fill_wheel_void(Camera& view)
{
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


void PT11::distance_sensor(Camera& view, PT11 enemy)
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
				//distance_input(arrx, arry, view, i);
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
	//cout << "label 1 is at " << enemy.label_nb_1 << endl;
	//cout << "label 2 is at " << enemy.label_nb_2 << endl;


	int what_label = 0;

	//copy(view.return_image(), view.return_a());

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
	/*
	//int distance_enemy2 = sqrt(pow(enemy.get_x2() - x1, 2) + pow(enemy.get_y2() - y1, 2));
	int enemy_no_obs1;
	int enemy_no_obs2;

	//cout << distance_enemy1 << "  |  " << distance_enemy2 << "  |  " << distance_log[0]*LL[0]<< endl;

	int box_length = 100;

	for (int i = 0; i < Ln[0]; i++)
	{
		if (arrx[i] > enemy.get_x1() - box_length && arrx[i] < enemy.get_x1() + box_length &&
			arry[i] > enemy.get_y1() - box_length && arry[i] < enemy.get_y1() + box_length)
		{
			enemy_no_obs1 = i * LL[0];
			break;
		}
		else {
			enemy_no_obs1 = 0;
		}
	}

	for (int i = 0; i < Ln[0]; i++)
	{
		if (arrx[i] > enemy.get_x2() - box_length && arrx[i] < enemy.get_x2() + box_length &&
			arry[i] > enemy.get_y2() - box_length && arry[i] < enemy.get_y2() + box_length)
		{
			enemy_no_obs2 = i * LL[0];
			break;
		}
		else {
			enemy_no_obs2 = 0;
		}
	}
	
	if (target_state == 1 &&
		(distance_log[0] * LL[0] < enemy_no_obs1 || distance_log[0] * LL[0] < enemy_no_obs2))
	{
		is_there_obstacle = 1;
	}
	else
	{
		is_there_obstacle = 0;
	}
	//cout << is_there_obstacle << endl;
	//cout << target_state << endl;
	/*
	cout << distance_log[0] << " ! " << enemy_no_obs1 << endl;

	if (distance_log[0] * LL[0] < enemy_no_obs2) cout << "Obstacle behind enemy front target" << endl;
	cout << distance_log[0] << " ! " << enemy_no_obs2 << endl;
	*/
	
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

void PT11::theta_target_delta_fix(double theta_target, double& target_delta, int& aim_dir)
{
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
}

void PT11::find_target(PT11 enemy)
{
	calculate_theta(enemy.get_x1(), enemy.get_y1(), x1, y1, theta_target1);
	//calculate_theta(enemy.get_x2(), enemy.get_y2(), x1, y1, theta_target2);
	//cout << theta_target1 << endl;

	int aim_dir;

	//cout << theta_target1 << "   ";

	theta_target_delta_fix(theta_target1, target_delta1, aim_dir);

	//cout << target_delta1 << "    " << aim_dir << endl;
	
	if (aim_dir == 1)
	{
		state_dir[0] = 0;
		state_dir[1] = 1;
	}
	else if (aim_dir == -1)
	{
		state_dir[0] = 1;
		state_dir[1] = 0;
	}

	if (target_state == 1)
	{
		state_dir[0] = 0;
		state_dir[1] = 0;
	}

	//cout << state_dir[0] << "    " << state_dir[1] << endl;
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
	
	cout << "Aim L - " << state_dir[0] << " ";
	cout << "Aim R - " << state_dir[1] << " ";
	
	
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

	/*
	double activation[4];
	for (int i = 0; i < 4; i++)
	{
		activation[i] = topology->output[i].get_value();
		
		if (activation[i] >= 0.5) activation[i] = 1;
		if (activation[i] < 0.5) activation[i] = 0;

		cout << activation[i] << "      ";
	}
	cout << endl;

	int action = 0;
	if (activation[0] == 1) action = 1;
	if (activation[1] == 1) action = 2;
	if (activation[2] == 1) action = 3;
	if (activation[3] == 1) action = 4;

	switch (action)
	{
	case 1:
		pw_l = 1000;
		pw_r = 2000;
		break;
	case 2:
		pw_l = 2000;
		pw_r = 1000;
		break;
	case 3:
		pw_l = 1000;
		pw_r = 1000;
		break;
	case 4:
		pw_l = 2000;
		pw_r = 2000;
		break;
	}
	*/



	/*

	int u[2];

	u[0] = 0;
	u[1] = 0;

	double activation[2];
	for (int i = 0; i < 2; i++)
	{
		activation[i] = topology->output[i].get_value();
		cout << activation[i] << "      ";
	}

	if (activation[0] >= 0.5) u[0] = 500;
	if (activation[0] < 0.5) u[0] = -500;
	if (activation[1] >= 0.5) u[1] = -450;
	if (activation[1] < 0.5) u[1] = 450;

	this->pw_l = 1500 + u[1] - u[0];
	this->pw_r = 1500 + u[1] + u[0];

	//cout << this->pw_l << "\t" << this->pw_r << endl;

	pw_r = this->pw_r;
	pw_l = this->pw_l;
	*/
}

void PT11::scout(int& pw_l, int& pw_r, int& pw_laser, int& laser)
{
	//Task for Afroza:
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
	int u[2];

	u[0] = 0;
	u[1] = 0;

	int action = 5;
	double theta_delta;
	int aim_dir;

	theta_target_delta_fix(VFF_theta, theta_delta, aim_dir);

	
	if (aim_dir == -1)
	{
		u[1] = 450;
	}
	else if (aim_dir == 1)
	{
		u[1] = -450;
	}
	
	if (theta_delta < 0.15) u[0] = 500;

	if (VFF_mag < 2000) u[0] = 500;

	//if (collision_state[2] == 1) action = 2;

	cout << VFF_theta << "   " << VFF_mag << endl;

	//cout << theta_delta << endl;


	this->pw_l = 1500 + u[1] - u[0];
	this->pw_r = 1500 + u[1] + u[0];

	//cout << this->pw_l << "\t" << this->pw_r << endl;

	pw_r = this->pw_r;
	pw_l = this->pw_l;
	laser = 0;

	/*
	switch (action)
	{
	case 0:					//Turn left
		pw_l = 2000;
		pw_r = 2000;
		break;
	case 1:					//Turn right
		pw_l = 1000;
		pw_r = 1000;
		break;
	case 2:					//Go Straight
		pw_l = 1000;
		pw_r = 2000;
		break;
	}
	*/
}

void PT11::highlight_view(Camera& view, PT11 enemy)
{
	bool flag_draw = 1;
	int radar_radius = 0;
	double theta_index = 0;
	double theta_jump = 0.1;
	int radar_minimum = 50;
	int radius_jump = 5;
	int radius_limit;
	double vector_x = 0;
	double vector_y = 0;
	double counter_vector_x = 0;
	double counter_vector_y = 0;
	bool enemy_trigger = 0;
	double multiplier;
	double enemy_multiplier;

	while (theta_index < (2 * M_PI))
	{
		radius_limit = 250;
		multiplier = 1;
		enemy_multiplier = 1.0;
		bool enemy_trigger = 0;
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

		hide_shadows(arrx, arry, view, theta_index, radar_radius, radius_limit, enemy_trigger, enemy, radius_jump);
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


void PT11::hide_shadows(int arrx[], int arry[], Camera& view, double theta_index, int& radar_radius, int radius_limit, bool& enemy_trigger, PT11 enemy, int radius_jump)
{
	int what_label;

	copy(view.return_image(), view.return_a());

	ibyte* pa;

	pa = view.return_a().pdata;

	int* k = new int[radius_limit];	//Can vary length of point series for collision accuracy
	

	for (int i2 = 0; i2 < radius_limit; i2+= radius_jump)	//For each dot on the line of points, find position based on 1D image reference
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
		
		
		if (what_label == enemy.label_nb_1 || what_label == enemy.label_nb_2)
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


	delete[]k;

}

void PT11::label_enemy(Camera& view, PT11 enemy)
{
	
}

PT11::~PT11()
{

}
