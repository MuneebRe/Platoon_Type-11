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



PT11::PT11(Camera &view)
{//We are using view[0], only this single Camera object, so no need for the whole view[3] pointer array of objects
	pw_l = 1500;
	pw_r = 1500;
	for (int i = 0; i < 4; i++)
	{
		collision_state[i] = 0;
	}

	radar_rgb.width = 640;
	radar_rgb.height = 480;
	radar_rgb.type = RGB_IMAGE;

	radar_greyscale.type = GREY_IMAGE;
	radar_greyscale.width = 640;
	radar_greyscale.height = 480;

	radar_label.type = LABEL_IMAGE;
	radar_label.width = 640;
	radar_label.height = 480;

	allocate_image(radar_rgb);
	allocate_image(radar_greyscale);
	allocate_image(radar_label);
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

	if (abs((y1 - y2) / (x1 - x2)) > 0.001)				//Makes theta angle continous 0 - 2 pi
	{
		theta = atan((y1 - y2) / (x1 - x2));
		if (y1 - y2 > 0 && x1 - x2 > 0) theta = theta;
		if (y1 - y2 > 0 && x1 - x2 < 0) theta = M_PI + theta;
		if (y1 - y2 < 0 && x1 - x2 < 0) theta = M_PI + theta;
		if (y1 - y2 < 0 && x1 - x2 > 0) theta = 2 * M_PI + theta;
	}

	//cout << x1 << "\t" << y1 << "\t" << theta << endl;
}

void PT11::collision_points(Camera &view)
{
	//Look at robot.cpp example, how Lx and Ly are used to connect laser on the bot at a distance
	//LL is the lenght between the dots (invisible)
	//Ln is the number of dots the user is interested in using per side
	//[0] Front - [1] right - [2] back - [3] left
	//Basically, there are dots on all sides of the car to detect collision.
	//I had to comment out draw_point_rgb because it messes up with check_collision( )

	Lx[0] = 40;		Ly[0] = 0;		LL[0] = 20;		Ln[0] = 4;
	Lx[1] = -40;	Ly[1] = -50;	LL[1] = 30;		Ln[1] = 4;
	Lx[2] = -120;	Ly[2] = 0;		LL[2] = 20;		Ln[2] = 4;
	Lx[3] = -40;	Ly[3] = 50;		LL[3] = 30;		Ln[3] = 4;

	for (int i = 0; i < 4; i++)
	{
		int* arrx = new int[Ln[i]];		//Dynamic memory, can change number of points interested in using
		int* arry = new int[Ln[i]];		//Kinda like resolution. More points can make it a line
		
		switch (i)						//Different sides, different line rotation. Same for [0] & [2] - [1] & [3]
		{
		case 0:
			for (int j = 0; j < Ln[0]; j++)
			{
				arrx[j] = x1 + Lx[i] * cos(theta) - (Ly[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * sin(theta);
				arry[j] = y1 + Lx[i] * sin(theta) + (Ly[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * cos(theta);
				//draw_point_rgb(view.return_image(), arrx[j], arry[j], 0, 0, 255);
			}
			break;
		case 1:
			for (int j = 0; j < Ln[1]; j++)
			{
				arrx[j] = x1 + (Lx[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * cos(theta) - (Ly[i]) * sin(theta);
				arry[j] = y1 + (Lx[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * sin(theta) + (Ly[i]) * cos(theta);
				//draw_point_rgb(view.return_image(), arrx[j], arry[j], 0, 0, 255);
			}
			break;
		case 2:
			for (int j = 0; j < Ln[2]; j++)
			{
				arrx[j] = x1 + Lx[i] * cos(theta) - (Ly[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * sin(theta);
				arry[j] = y1 + Lx[i] * sin(theta) + (Ly[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * cos(theta);
				//draw_point_rgb(view.return_image(), arrx[j], arry[j], 0, 0, 255);
			}
			break;
		case 3:
			for (int j = 0; j < Ln[3]; j++)
			{
				arrx[j] = x1 + (Lx[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * cos(theta) - (Ly[i]) * sin(theta);
				arry[j] = y1 + (Lx[i] + (LL[i] * Ln[i]) / 2.0 - LL[i] / 2.0 - j * LL[i]) * sin(theta) + (Ly[i]) * cos(theta);
				//draw_point_rgb(view.return_image(), arrx[j], arry[j], 0, 0, 255);
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
		k[i2] = arrx[i2] + view.return_a().width * arry[i2];
	}

	for(int i2 =0; i2< Ln[i] ; i2++)	//If either point has 255 at pointer, turn on collision state
	{
		if (pa[k[i2]] == 255)
		{
			collision_state[i] = true;
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
	if (sum >= Ln[i]) collision_state[i] = 0;

	delete []k;

	
	cout << collision_state[0] << "\t" << collision_state[1] << "\t" << collision_state[2] << "\t" << collision_state[3] << endl;
	
}

void PT11::acquire_camera_image(Camera &view) {
	//copy(view.return_image(), radar_rgb);		//Call this function after acquiring image in main so PT11 has its own copy, this is the equivalent to "original" in Camera class
												//***NOTE: THIS MIGHT BE USELESS, might just directly alter view[0]->rgb image, and view it directly on the actual "rgb" variable
												//***NOTE2: CURRENTLY THIS FUNCTION IS ONLY GRABBING THRESHOLDED IMAGE after calling case 1 in main, placement of this function in main is important
	
	copy(view.return_a(), radar_greyscale); //Storing image "a" which is a binary thresholded image (I placed it in main like that)


	//label_image(radar_greyscale, );		//Do I even need to label? Not yet, just need to set up the code to work around black and white pixels, then I will add
											//the logic of distinguishing objects to robots by using label image I think
}

void PT11::draw_safe_zone(int* line_array_i, int* line_array_j, int size, Camera &view) {
	ibyte *p_greyscale, *p_rgb, R, G, B;	//This will iterate through the binary image, perform logic to understand which pixels should be considered green. RGB will be used in view->return_image()
	int i, j, x, y, k, size1;
	int flag1 = 0;		//Triggers when white pixel is seen, this means following black pixels are safe zones
	//int flag2 = 0;		//Triggers when safe zone starts, this indicates new white pixel has been observed along line, which is not a safe zone because we can't enter white pixel areas (obstacle)
	size1 = size;

	p_greyscale = radar_greyscale.pdata;	//Points to binary image
	p_rgb = view.return_image().pdata;		//Points to LIVE rgb image

	for (i = 40; i < size1; i++) {
		x = line_array_i[i];		//x-coordinate of pixel 
		y = line_array_j[i];		//y-coordinate of pixel

		k = x + (y * 640);		//k-coordinate of pixel NOTE: I would use view.width instead of 640 BUT its sussy... just gonna put 640 LOL

		if (p_greyscale[k] < 50) { //detecting a black pixel
			if (flag1 == 1) {
				//This will only trigger for the first black pixel observed after white pixels ie: white pixels have been detected earlier in the line, safe zone!
				p_rgb[(k*3)] = 50; //B
				p_rgb[(k*3) + 1] = 205; //G
				p_rgb[(k*3) + 2] = 50; //R
				//flag2 = 1;		//black pixels have been observed, so next time white pixels appear = stop safe zone (not necessary, safe zone is only written for black pixels, and white object is covering those black pixels anyways)
			}
		}

		if (p_greyscale[k] > 240) {
			flag1 = 1;			//white pixels have been detected, so next time black pixels appear = safe zone
		}

	}

}

void PT11::get_safe_zone(Camera &view, int pt_i[4], int pt_j[4]) {

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

	x0 = pt_i[2];		//centroids of enemy_robot (orange circle)
	y0 = pt_j[2];

	//x1 = 640;		Replaced by border_x, border_y
	//y1 = y0 + 1;

	//delta_x = x1 - x0;		These should adjust as lines change
	//delta_y = y1 - y0;



	for (border_y = 0; border_y < 480; border_y++) {
		//Scanning through right-border, all lines from centroid to wall

		if (border_y == y0) {
			border_y++;		//safety net to avoid horizontal line which will have slope = 0 (Is this even necessary??)
		}

		border_x = 640;		//Initialize border_x for this sweep NOTE:(should be in a loop or something eventually, since border will change to 0 at some point)

		delta_x = border_x - x0;
		delta_y = border_y - y0;

		slope = delta_y / delta_x;
		b = y0 - (slope * x0);

		for (i = x0; i < border_x - 3; i++) {
			//Iterate through all x-values for each line, LATER: account for vertical leaning lines
			j = int((slope * i) + b);

			line_array_i[size] = i;
			line_array_j[size] = j;

			size++;
		}
		/* Commented out constant line drawing to start next step of programming
		for (increment = 50; increment < size; increment++) {
			//Draw result so we can see what's happening
			int x, y;

			x = line_array_i[increment];
			y = line_array_j[increment];

			draw_point_rgb(view.return_image(), x, y, 255, 0, 0);
		}
		*/

		draw_safe_zone(line_array_i, line_array_j, size, view);		//SHOULD DRAW LIME GREEN SAFE ZONES :O

		size = 0;		//Very important, reset array back to 0 element for next line

		//I am adding line-drawing of left border in this for-loop because it uses the same y-range

		border_x = 0;

		delta_x = border_x - x0;
		delta_y = border_y - y0;

		slope = delta_y / delta_x;
		b = y0 - (slope * x0);

		for (i = x0; i > border_x; i -= 1) {
			//Iterate through all x-values for each line (Lines start from centroid as if laser is shooting out)
			j = int((slope * i) + b);

			line_array_i[size] = i;
			line_array_j[size] = j;

			size++;
		}

		/* Commented out constant line drawing to start next step of programming
		for (increment = 50; increment < size; increment++) {
			//Draw result so we can see what's happening
			int x, y;

			x = line_array_i[increment];
			y = line_array_j[increment];

			draw_point_rgb(view.return_image(), x, y, 0, 255, 0);
		}
		*/

		draw_safe_zone(line_array_i, line_array_j, size, view);		//SHOULD DRAW LIME GREEN SAFE ZONES :O

		size = 0;		//Very important, reset array back to 0 element for next line

	}

	for (border_x = 0; border_x < 640; border_x++) {
		//Scanning through top-border, all lines from centroid to wall

		if (border_x == x0) {
			border_x++;		//safety net to avoid horizontal line which will have slope = 0 (Is this even necessary??)
		}

		border_y = 480;		//Initialize border_x for this sweep NOTE:(should be in a loop or something eventually, since border will change to 0 at some point)

		delta_x = border_x - x0;
		delta_y = border_y - y0;

		slope = delta_x / delta_y;
		b = x0 - (slope * y0);

		for (j = y0; j < border_y; j++) {
			//Iterate through all x-values for each line, LATER: account for vertical leaning lines
			i = int((slope * j) + b);

			line_array_i[size] = i;
			line_array_j[size] = j;

			size++;
		}

		/*
		for (increment = 50; increment < size; increment++) {
			//Draw result so we can see what's happening
			int x, y;

			x = line_array_i[increment];
			y = line_array_j[increment];

			draw_point_rgb(view.return_image(), x, y, 0, 0, 255);
		}
		*/ 

		draw_safe_zone(line_array_i, line_array_j, size, view);		//SHOULD DRAW LIME GREEN SAFE ZONES :O

		size = 0;		//Very important, reset array back to 0 element for next line

		//I am adding line-drawing of bottom border in this for-loop because it uses the same y-range

		border_y = 0;		//Initialize border_x for this sweep NOTE:(should be in a loop or something eventually, since border will change to 0 at some point)

		delta_x = border_x - x0;
		delta_y = border_y - y0;

		slope = delta_x / delta_y;
		b = x0 - (slope * y0);

		for (j = y0; j > border_y; j -= 1) {
			//Iterate through all x-values for each line, LATER: account for vertical leaning lines
			i = int((slope * j) + b);

			line_array_i[size] = i;
			line_array_j[size] = j;

			size++;
		}

		/*
		for (increment = 50; increment < size; increment++) {
			//Draw result so we can see what's happening
			int x, y;

			x = line_array_i[increment];
			y = line_array_j[increment];

			draw_point_rgb(view.return_image(), x, y, 255, 255, 0);
		}
		*/

		draw_safe_zone(line_array_i, line_array_j, size, view);		//SHOULD DRAW LIME GREEN SAFE ZONES :O

		size = 0;		//Very important, reset array back to 0 element for next line

	}

	delete[] line_array_i;
	delete[] line_array_j;
}



PT11::~PT11()
{

}
