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
#include "NeuralNet.h"

extern robot_system S1;


#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

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
	/*
	x_obs[1] = 300; // pixels
	y_obs[1] = 200; // pixels
	size_obs[1] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	x_obs[2] = 300;// 135; // pixels
	y_obs[2] = 70;// 135; // pixels
	size_obs[2] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	
	*/
	/*
	N_obs = 5;
	for (int i = 1; i <= N_obs; i++)
	{
		x_obs[i] = 300;
		y_obs[i] = 70 * i - 70 ;
		//y_obs[i] = 0;
		size_obs[i] = 1.0;
	}
	*/
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
	//pause();

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

	int view_state[3] = { true, false, false };	//Defines if enabled/disabled Simulator, Top View Cam, First Person Cam

	Camera* view[3];

	view[0] = new Camera(view_state[0], 0, 640, 480, RGB_IMAGE, true, 1);	 //Simulator		   (sim)
	view[1] = new Camera(view_state[1], 0, 640, 480, RGB_IMAGE, false, 0);	 //Top View Camera	   (real)
	view[2] = new Camera(view_state[2], 1, 640, 480, RGB_IMAGE, false, 0);   //First Person Camera (real)
	

	while (1)
	{
	static int trial_number = 0;
	cout << "Trial Number " << trial_number << " begin!" << endl;
	// set robot initial position (pixels) and angle (rad)
	x0 = 470;
	y0 = 300;
	theta0 = 0;
	//theta0 = 3.14159/2;
	set_robot_position(x0,y0,theta0);
	
	// set opponent initial position (pixels) and angle (rad)
	x0 = 50;
	y0 = 400;
	//theta0 = 3.14159/4;
	theta0 = 3.14159 / 2;
	set_opponent_position(x0,y0,theta0);

	// set initial inputs / on-line adjustable parameters /////////	

	// inputs
	pw_l = 1500; // pulse width for left wheel servo (us)
	pw_r = 1500; // pulse width for right wheel servo (us)

	pw_laser = 1500; // pulse width for laser servo (us)
	laser = 0; // laser input (0 - off, 1 - fire)
	
	// paramaters
	max_speed = 100; // 100; // max wheel speed of robot (pixels/s)
	opponent_max_speed = 100; // 100; // max wheel speed of opponent (pixels/s)
	
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

	//Serial port(false, "COM12", 1);		//Establish bluetooth communication with robot (real)

	//Enable Neural Network for enemy?
	static bool AI_player = 0;	//for player
	static bool AI_enemy = 0;	//REF1-1 enemy will follow weight pattern as pt11

	PT11 pt11;		//Make instance of robot (sim)
	PT11 enemy;		//Make instance of enemy

	if (AI_player == 1) pt11.init_neural();		//REF1-2 Load latest weight data, then randomize it (either relative to best one or fully random)
	if(AI_enemy == 1) enemy.init_neural();

	//runNet();	//Neural Network copied off online and edited for my use,
				//But it requires specific training data so it's no good, deals with conflict in order.

	// measure initial clock time
	tc0 = high_resolution_time(); 
	
	//view[0]->find_object();    //Testing for "tracking" lecture, pick a white spot. Press 'c' to select.

		while(1) {
			
			//port.send(0, 0, 0, 0);		//Send control order to robot (real)

			if (index > view[0]->get_count() - 1) index = 0;	//Roll back view number
			if (KEY('V') || view_state[index] == false)			//*Only used if using real camera
			{													//*Changes views
				index++;
				Sleep(350);
				continue;
			}

			view[index]->acquire();					//Get RGB image
			//view[0]->acquire();					//Get RGB image
			view[index]->draw_border();
			//view[0]->draw_border();

			view[0]->set_processing(0);			//Set and Prep for original copy
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
			

			pt11.set_coord(pt_i[2], pt_j[2], pt_i[0], pt_j[0]);
			enemy.set_coord(pt_i[1], pt_j[1], pt_i[3], pt_j[3]);
		
			view[0]->set_processing(1);			//Enable threshold processing and everything
			view[0]->processing();				//Run process
		
			pt11.fill_wheel_void(*view[0]);
			enemy.fill_wheel_void(*view[0]);

			view[0]->set_processing(11);		//Enable labeling processing and everything
			view[0]->processing();				//Run process

			
			pt11.label_nb_1 = (int)view[0]->label_at_coordinate(pt_i[2] + 15, pt_j[2] + 15);
			pt11.label_nb_2 = (int)view[0]->label_at_coordinate(pt_i[0] + 15, pt_j[0] + 15);

			enemy.label_nb_1 = (int)view[0]->label_at_coordinate(pt_i[1] + 15, pt_j[1] + 15);
			enemy.label_nb_2 = (int)view[0]->label_at_coordinate(pt_i[3] + 15, pt_j[3] + 15);

			//pt11.label_enemy(view[0], enemy);

			pt11.collision_points(*view[0]);	//Move view[0] object into pt11 function
			pt11.distance_sensor(*view[0], enemy);
			pt11.find_target(enemy);
			pt11.highlight_view(*view[0], enemy);

			if (AI_player == 1)
			{
				pt11.NeuroLearn(pw_l, pw_r, laser, trial_number);
			}
			else if (AI_player == 0)
			{
				pt11.manual_set(pw_l, pw_r, pw_laser, laser);		//Control the bot. A W D for laser, arrows for bot
				//pt11.scout(pw_l, pw_r, pw_laser, laser);
				pt11.attack(pw_l, pw_r, pw_laser, laser);
			}
			/*
			enemy.collision_points(*view[0]);
			enemy.distance_sensor(*view[0], pt11);
			enemy.find_target(pt11);
			enemy.distance_sensor(*view[0], pt11);
			enemy.highlight_view(*view[0], pt11);
			*/
			if (AI_enemy == 1)		//REF1-3 Enable collision detection, target detection, and 8 sides distance sensor, run AI.
			{
				//enemy.NeuroLearn(pw_l_o, pw_r_o, laser, trial_number);
			}
			else
			{
				//enemy.manual_set(pw_l_o, pw_r_o, pw_laser_o, laser_o);
				//enemy.attack(pw_l_o, pw_r_o, pw_laser_o, laser_o);
			}


			for (int i = 0; i < 4; i++)
			{
				draw_point_rgb(view[0]->return_image(), pt_i[i], pt_j[i], 0, 255, 255); //Call back array and draw point at those locations
			}

			////pt11.m_runNet(pw_l, pw_r, laser);		//Also not used, results inconsistent
			//view[0]->set_processing(12);			//Set and Prep for original copy
			//view[0]->processing();				//Make a copy of the rgb image
			/*
			view[0]->set_processing(0);			//Set and Prep for original copy
			view[0]->processing();				//Make a copy of the rgb image

			view[0]->set_processing(10);		//Prep for sobel imagery
			view[0]->processing();				//Do sobel imagery
			*/
			//draw_point_rgb(view[0]->return_image(), pt_i[1], pt_j[1], 0, 0, 255);
			//draw_point_rgb(view[0]->return_image(), pt_i[3], pt_j[3], 0, 0, 255);

			view[index]->view();	//View the the processed image MUNEEB REF 
			//view[0]->view();


			tc = high_resolution_time() - tc0;

			set_inputs(pw_l,pw_r,pw_laser,laser,
				light,light_gradient,light_dir,image_noise,
				max_speed,opponent_max_speed);

			set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, 
						opponent_max_speed);

			// don't need to simulate too fast
			Sleep(10); // 100 fps max

			if (pt11.get_reset_state() == 1 || enemy.get_reset_state() == 1)
			{
				trial_number += 1;

				//Sleep(800);
				break;
				
			}

		}

	}
	// free the image memory before the program completes

	deactivate_vision();
	
	deactivate_simulation();	
	
	
	cout << "\ndone.\n";

	return 0;
}
