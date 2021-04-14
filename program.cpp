using namespace std;
#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

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
#include "Serial.h"
#include "PT11.h"

extern robot_system S1;


#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

/*
1. Create line equation for radar (DONE)
2. Sweep through entire image regardeless of robot positioning 
3. Account for vertical leaning slopes (vertical leaning slopes will require the opposite: iterating through y to calculate x, watch linear equation youtube video btw)
4. Add processing functions to analyze the line arrays when they are created, to create a mask image
*/

/*
void get_safe_zone(Camera* view[3], int pt_i[4], int pt_j[4]);

void get_safe_zone(Camera* view[3], int pt_i[4], int pt_j[4]){

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

		for (increment = 30; increment < size; increment++) {
			//Draw result so we can see what's happening
			int x, y;

			x = line_array_i[increment];
			y = line_array_j[increment];

			draw_point_rgb(view[0]->return_image(), x, y, 255, 0, 0);
		}
		size = 0;		//Very important, reset array back to 0 element for next line

		//I am adding line-drawing of left border in this for-loop because it uses the same y-range

		border_x = 0;

		delta_x = border_x - x0;
		delta_y = border_y - y0;

		slope = delta_y / delta_x;
		b = y0 - (slope * x0);

		for (i = x0; i > border_x; i-= 1) {
			//Iterate through all x-values for each line (Lines start from centroid as if laser is shooting out)
			j = int((slope * i) + b);

			line_array_i[size] = i;
			line_array_j[size] = j;

			size++;
		}

		for (increment = 30; increment < size; increment++) {
			//Draw result so we can see what's happening
			int x, y;

			x = line_array_i[increment];
			y = line_array_j[increment];

			draw_point_rgb(view[0]->return_image(), x, y, 0, 255, 0);
		}
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

		for (increment = 30; increment < size; increment++) {
			//Draw result so we can see what's happening
			int x, y;

			x = line_array_i[increment];
			y = line_array_j[increment];

			draw_point_rgb(view[0]->return_image(), x, y, 0, 0, 255);
		}
		size = 0;		//Very important, reset array back to 0 element for next line

		//I am adding line-drawing of bottom border in this for-loop because it uses the same y-range

		border_y = 0;		//Initialize border_x for this sweep NOTE:(should be in a loop or something eventually, since border will change to 0 at some point)

		delta_x = border_x - x0;
		delta_y = border_y - y0;

		slope = delta_x / delta_y;
		b = x0 - (slope * y0);

		for (j = y0; j > border_y; j-=1) {
			//Iterate through all x-values for each line, LATER: account for vertical leaning lines
			i = int((slope * j) + b);

			line_array_i[size] = i;
			line_array_j[size] = j;

			size++;
		}

		for (increment = 30; increment < size; increment++) {
			//Draw result so we can see what's happening
			int x, y;

			x = line_array_i[increment];
			y = line_array_j[increment];

			draw_point_rgb(view[0]->return_image(), x, y, 255, 255, 0);
		}
		size = 0;		//Very important, reset array back to 0 element for next line

	}

	delete[] line_array_i;
	delete[] line_array_j;
}
*/


int main()
{
	double x0, y0, theta0, max_speed, opponent_max_speed;
	int pw_l, pw_r, pw_laser, laser;
	double light, light_gradient, light_dir, image_noise;
	double width1, height1;
	int N_obs, n_robot;
	double x_obs[50], y_obs[50], size_obs[50];
	double D, Lx, Ly, Ax, Ay, alpha_max;
	double tc, tc0; // clock time
	int mode, level;
	int pw_l_o, pw_r_o, pw_laser_o, laser_o;
	
	// note that the vision simulation library currently
	// assumes an image size of 640x480
	width1  = 640;
	height1 = 480;
	
	// number of obstacles
	N_obs  = 2;

	x_obs[1] = 270; // pixels
	y_obs[1] = 270; // pixels
	size_obs[1] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	x_obs[2] = 135; // pixels
	y_obs[2] = 135; // pixels
	size_obs[2] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	// set robot model parameters ////////
	
	D = 121.0; // distance between front wheels (pixels)
	
	// position of laser in local robot coordinates (pixels)
	// note for Lx, Ly we assume in local coord the robot
	// is pointing in the x direction		
	Lx = 31.0;
	Ly = 0.0;
	
	// position of robot axis of rotation halfway between wheels (pixels)
	// relative to the robot image center in local coordinates
	Ax = 37.0;
	Ay = 0.0;
	
	alpha_max = 3.14159/2; // max range of laser / gripper (rad)
	
	// number of robot (1 - no opponent, 2 - with opponent, 3 - not implemented yet)
	n_robot = 2;
	
	cout << "\npress space key to begin program.";
	pause();

	activate_vision();

	activate_simulation(width1,height1,x_obs,y_obs,size_obs,N_obs,
		"robot_A.bmp","robot_B.bmp","background.bmp","obstacle.bmp",D,Lx,Ly,
		Ax,Ay,alpha_max,n_robot);	

	// open an output file if needed for testing or plotting
	//	ofstream fout("sim1.txt");
	//	fout << scientific;
	
	// set simulation mode (level is currently not implemented)
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 0;
	level = 1;
	set_simulation_mode(mode,level);	
	
	// set robot initial position (pixels) and angle (rad)
	x0 = 470;
	y0 = 170;
	theta0 = 0;
	set_robot_position(x0,y0,theta0);
	
	// set opponent initial position (pixels) and angle (rad)
	x0 = 150;
	y0 = 375;
	theta0 = 3.14159/4;
	set_opponent_position(x0,y0,theta0);

	// set initial inputs / on-line adjustable parameters /////////	

	// inputs
	pw_l = 1500; // pulse width for left wheel servo (us)
	pw_r = 1500; // pulse width for right wheel servo (us)

	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)
	
	// paramaters
	max_speed = 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // max wheel speed of opponent (pixels/s)
	
	// lighting parameters (not currently implemented in the library)
	light = 1.0;
	light_gradient = 1.0;
	light_dir = 1.0;
	image_noise = 1.0;

	// set initial inputs
	set_inputs(pw_l,pw_r,pw_laser,laser,
		light,light_gradient,light_dir,image_noise,
		max_speed,opponent_max_speed);

	// opponent inputs
	pw_l_o = 1500; // pulse width for left wheel servo (us)
	pw_r_o = 1500; // pulse width for right wheel servo (us)
	pw_laser_o = 1500; // pulse width for laser servo (us)
	laser_o = 0; // laser input (0 - off, 1 - fire)

	// manually set opponent inputs for the simulation
	// -- good for testing your program
	set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, 
				opponent_max_speed);

	// regular vision program ////////////////////////////////
	
	// note that at this point you can write your vision program
	// exactly as before.
	
	// in addition, you can set the robot inputs to move it around
	// the image and fire the laser

	int index = 0;	//Used for switching cameras

	int pt_i[4];	//x Point locations of the "colored" parts of both robots
	int pt_j[4];	//y Point locations of the "colored" parts of both robots

	int view_state[3] = { true, false, false };	//Defines if enabled/disabled Simulator, Top View Cam, First Person Cam

	Camera* view[3];
	view[0] = new Camera(view_state[0], 0, 640, 480, RGB_IMAGE, true, 1);	 //Simulator		   (sim)
	view[1] = new Camera(view_state[1], 0, 640, 480, RGB_IMAGE, false, 0);	 //Top View Camera	   (real)
	view[2] = new Camera(view_state[2], 0, 640, 480, RGB_IMAGE, false, 0);   //First Person Camera (real)

	Serial port(false, "COM12", 1);		//Establish bluetooth communication with robot (real)

	PT11 pt11(*view[0]);		//Make instance of robot (sim)

	// measure initial clock time
	tc0 = high_resolution_time(); 

	
	//view[0]->find_object();    //Testing for "tracking" lecture, pick a white spot. Press 'c' to select.

	while(1) {

		port.send(0, 0, 0, 0);		//Send control order to robot (real)

		if (index > view[0]->get_count() - 1) index = 0;	//Roll back view number
		if (KEY('V') || view_state[index] == false)			//*Only used if using real camera
		{													//*Changes views
			index++;
			Sleep(350);
			continue;
		}

		view[index]->acquire();					//Get RGB image
		pt11.acquire_camera_image(*view[0]);	//Make a copy of the RGB image to pt11

		view[0]->set_processing(0);			//Set and Prep for original copy
		view[0]->processing();				//Make a copy of the rgb image

		view[0]->set_processing(1);			//Threshold image 'a' so pt11 can copy it
		view[0]->processing();				

		pt11.acquire_camera_image(*view[0]);	//Make a copy of the thresholded 'a' image to greyscale so I can do processing on the binary image

		view[0]->set_processing(3);			//Set and Prep for original copy
		view[0]->processing();				//Make a copy of the rgb image

		/*
		view[0]->set_processing(1);			//Prepare rgb image for process used to keep tracking
		view[0]->track_object();			//Track object selected from view[0]->find_object();

		view[0]->set_processing(3);			//Copy original image to current rgb image
		view[0]->processing();				//Also, add a point to the "tracked" object

		//view[0]->set_processing(4);
		//view[0]->processing();

		//view[0]->set_processing(5);
		//view[0]->processing();
		*/
		view[0]->set_processing(0);			//Set and Prep for original copy
		view[0]->processing();				//Make a copy of the rgb image

		for (int i = 6; i < 10; i++)
		{
			view[0]->set_processing(i);		//Run filter for blue, orange, green and red
			view[0]->processing();			//to find centroid location for each
			pt_i[i-6] = view[0]->get_ic();	//And put them in this array
			pt_j[i-6] = view[0]->get_jc();	//For each color
		}

		/*
		for (int i = 0; i < 4; i++) {
			cout << "\n Centroid (x,y), " << i << " = " << pt_i[i] << " , " << pt_j[i] << endl;
		}
		cout << "gurv: paused, press any key to continue";
		*/

		for (int i = 0; i < 4; i++)
		{
			draw_point_rgb(view[0]->return_image(), pt_i[i], pt_j[i], 0, 0, 255); //Call back array and draw point at those locations
		}
		 
		/*
		view[0]->set_processing(1);			//Enable threshold processing and everything
		view[0]->processing();				//Run process
		pt11.collision_points(*view[0]);	//Move view[0] object into pt11 function
		*/
		//pt11.check_collision(view[0]);

		/*
		view[0]->set_processing(0);			//Set and Prep for original copy
		view[0]->processing();				//Make a copy of the rgb image

		view[0]->set_processing(10);		//Prep for sobel imagery
		view[0]->processing();				//Do sobel imagery
		*/

		pt11.get_safe_zone(*view[0], pt_i, pt_j);		//This function draws directly onto the latest RGB before its viewed on screen

		view[index]->view();	//View the the processed image
		
		pt11.set_coord(pt_i[3], pt_j[3], pt_i[0], pt_j[0]);

		pt11.manual_set(pw_l, pw_r, pw_laser, laser);		//Control the bot. A W D for laser, arrows for bot

		tc = high_resolution_time() - tc0;

		set_inputs(pw_l,pw_r,pw_laser,laser,
			light,light_gradient,light_dir,image_noise,
			max_speed,opponent_max_speed);

		set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, 
					opponent_max_speed);

		// don't need to simulate too fast
		Sleep(10); // 100 fps max
	}


	// free the image memory before the program completes

	deactivate_vision();
	
	deactivate_simulation();	
	
	cout << "\ndone.\n";

	return 0;
}
