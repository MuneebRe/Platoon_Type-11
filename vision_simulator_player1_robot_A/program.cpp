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
#include "..\vision_simulator_original\serial_com.h"
#include "..\vision_simulator_original\image_transfer.h"

#include "..\vision_simulator_original\vision.h"
#include "..\vision_simulator_original\robot.h"
#include "..\vision_simulator_original\vision_simulation.h"
#include "..\vision_simulator_original\timer.h"

#include "..\vision_simulator_original\Camera.h"
#include "..\vision_simulator_original\Serial.h"
#include "..\vision_simulator_original\PT11.h"
#include "..\vision_simulator_original\NeuralNet.h"

extern robot_system S1;


#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

void processing_and_labelling(Camera& view, PT11& pt11, PT11& enemy, int pt_i[], int pt_j[]);

void command_execution(char which_robot, int Robot_Command, bool return_original, bool show_centroids,
	Camera& view, PT11& pt11, PT11& enemy,
	int& pw_l, int& pw_r, int& pw_laser, int& laser,
	int& pw_l_o, int& pw_r_o, int& pw_laser_o, int& laser_o,
	int pt_i[], int pt_j[]);


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
	
	x_obs[1] = 300; // pixels
	y_obs[1] = 200; // pixels
	size_obs[1] = 1.0; // scale factor 1.0 = 100% (not implemented yet)	

	x_obs[2] = 3000;// 135; // pixels
	y_obs[2] = 300;// 135; // pixels
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
	//pause();

	activate_vision();

	activate_simulation(width1,height1,x_obs,y_obs,size_obs,N_obs,
		"../vision_simulator_original/robot_A.bmp","../vision_simulator_original/robot_B.bmp","../vision_simulator_original/background.bmp","../vision_simulator_original/obstacle.bmp",D,Lx,Ly,
		Ax,Ay,alpha_max,n_robot);	
	
	// set simulation mode (level is currently not implemented)
	// mode = 0 - single player mode (manual opponent)
	// mode = 1 - two player mode, player #1
	// mode = 2 - two player mode, player #2	
	mode = 1;
	level = 1;
	set_simulation_mode(mode,level);

	//Defines if camera view enabled/disabled for
	//int view_state[3] = { true,			//Simulator		   (sim)
	//					  false,		//Top View Cam     (real)
	//	                  false };		//First Person Cam (real)

	//Camera* view[3];
	//view[0] = new Camera(view_state[0], 0, 640, 480, RGB_IMAGE, true, 1);	 //Simulator		   (sim)
	//view[1] = new Camera(view_state[1], 0, 640, 480, RGB_IMAGE, false, 0);	 //Top View Camera	   (real)
	//view[2] = new Camera(view_state[2], 1, 640, 480, RGB_IMAGE, false, 0);   //First Person Camera (real)


	int view_state[3] = { true,			//Simulator		   (sim)
							  false,		//Top View Cam     (real)
							  false };		//First Person Cam (real)

	//Camera* view[3];
	//view[0] = new Camera(view_state[0], 0, 640, 480, RGB_IMAGE, true, 1);	 //Simulator		   (sim)
	//view[1] = new Camera(view_state[1], 0, 640, 480, RGB_IMAGE, false, 0);	 //Top View Camera	   (real)
	//view[2] = new Camera(view_state[2], 1, 640, 480, RGB_IMAGE, false, 0);   //First Person Camera (real)

	Camera view_0(view_state[0], 0, 640, 480, RGB_IMAGE, true, 1);	 //Simulator
	Camera view_1(view_state[1], 0, 640, 480, RGB_IMAGE, false, 0);	 //Top View Camera	
	Camera view_2(view_state[2], 1, 640, 480, RGB_IMAGE, false, 0);	 //First person camera

	int pt_i[4];	//x Point locations of the "colored" parts of both robots
	int pt_j[4];	//y Point locations of the "colored" parts of both robots
	PT11 pt11(view_0);		//Make instance of robot (sim)
	PT11 enemy(view_0);		//Make instance of enemy



	while (1)
	{
	
	/*
	//AI Discontinued
	static int trial_number = 0;
	cout << "Trial Number " << trial_number << " begin!" << endl;
	*/

	// set robot initial position (pixels) and angle (rad)
	
	x0 = 400;
	y0 = 300;
	theta0 = 0;
	set_robot_position(x0,y0,theta0);
	
	/*
	// set opponent initial position (pixels) and angle (rad)
	x0 = 150;
	y0 = 200;
	//theta0 = 3.14159/4;
	theta0 = 3.14159 / 2;
	set_opponent_position(x0,y0,theta0);
	*/

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

	/*
	// opponent inputs
	pw_l_o = 1500; // pulse width for left wheel servo (us)
	pw_r_o = 1500; // pulse width for right wheel servo (us)
	pw_laser_o = 1500; // pulse width for laser servo (us)
	laser_o = 0; // laser input (0 - off, 1 - fire)

	// manually set opponent inputs for the simulation
	// -- good for testing your program
	set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, 
				opponent_max_speed);
	*/
	// regular vision program ////////////////////////////////
	
	// note that at this point you can write your vision program
	// exactly as before.
	
	// in addition, you can set the robot inputs to move it around
	// the image and fire the laser

	int index = 0;	//Used for switching cameras

	//int pt_i[4];	//x Point locations of the "colored" parts of both robots
	//int pt_j[4];	//y Point locations of the "colored" parts of both robots

	Serial port(false, "COM12", 1);		//Establish bluetooth communication with robot (real)

	//Enable Neural Network for enemy?
	static bool AI_player = 0;	//for player
	static bool AI_enemy = 0;	//REF1-1 enemy will follow weight pattern as pt11

	//PT11 pt11(*view[0]);		//Make instance of robot (sim)
	//PT11 enemy(*view[0]);		//Make instance of enemy

	if (AI_player == 1) pt11.init_neural();		//REF1-2 Load latest weight data, then randomize it (either relative to best one or fully random)
	if(AI_enemy == 1) enemy.init_neural();

	// measure initial clock time
	tc0 = high_resolution_time(); 
	
	wait_for_player();

		while(1) {
			
			port.send(0, 0, 0, 0);		//Send control order to robot (real)
			
			/*
			if (index > view[0]->get_count() - 1) index = 0;	//Roll back view number
			if (KEY('V') || view_state[index] == false)			//*Only used if using real camera
			{													//*Changes views
				index++;
				Sleep(350);
				//continue;
			}
			*/

			processing_and_labelling(view_0, pt11, enemy, pt_i, pt_j);

			//Write 'A' for Robot A
			//Write 'B' for Robot B
			//**Update: Both robots need to beleive that they are Robot A
			//			For testing pruposes, use vision_simulator_original
			//			to test and fine tune parameters.

			//Command 0: Manual Key Control
			//Command 1: Attack Function
			//Command 2: Evade Function
			//Command 3: AI Evolutionary Neural Network (Disabled)
			//Command 4: Scout function (Discontinued)
			//Command 5: Do Nothing

			command_execution('A', 1, 0, 1,	//Robot A or B , Command #0-5, show original 0-1, show centroids 0-1
							   view_0, pt11, enemy, 
							   pw_l, pw_r, pw_laser, laser,
							   pw_l_o, pw_r_o, pw_laser_o, laser_o,
							   pt_i, pt_j);

			view_0.view();

			tc = high_resolution_time() - tc0;

			set_inputs(pw_l,pw_r,pw_laser,laser,
				light,light_gradient,light_dir,image_noise,
				max_speed,opponent_max_speed);

			/*
			set_opponent_inputs(pw_l_o, pw_r_o, pw_laser_o, laser_o, 
						opponent_max_speed);
			*/
			// don't need to simulate too fast
			Sleep(10); // 100 fps max

			/*
			//AI Discontinued
			if (pt11.get_reset_state() == 1 || enemy.get_reset_state() == 1)
			{
				trial_number += 1;

				//Sleep(800);
				break;
				
			}
			*/
		}

	}
	// free the image memory before the program completes

	deactivate_vision();
	
	deactivate_simulation();
	
	cout << "\ndone.\n";

	return 0;
}

void processing_and_labelling(Camera& view, PT11& pt11, PT11& enemy, int pt_i[], int pt_j[])
{
	view.acquire();					// Get simulation image
	view.draw_border();				// Draw a black border around the map

	view.set_processing(0);			// Copy 'rgb' image to 'original' image
	view.processing();

	view.set_processing(1);			//Grayscale . Scale . Threshold <100> . ..
	view.processing();				//..Invert . Erode . Dilate

	pt11.acquire_camera_image(view);	// Copies image 'rgb' and greyscale (ie: binary) image 'a' into radar image objects, creates label image

	view.set_processing(11);		//Label 'rgb' objects to 'label' image 
	view.processing();

	view.coordinate_finder();	//Find labels of objects with 1000-3000 pixels
									//Then find the centroid of those of those labels

	for (int i = 13; i < 17; i++)
	{
		view.set_processing(i);			//Run filter for blue, orange, green and red
		view.processing();				//If the rgb pixel for each centroids
														//found from the coordinate_finder
														//is the same as the HSV range from the hue_filter function
														//Then save ic and jc
		pt_i[i - 13] = view.get_ic();	//And move them in this array
		pt_j[i - 13] = view.get_jc();	//For each color
	}

	pt11.set_coord(pt_i[2], pt_j[2], pt_i[0], pt_j[0]);		//Save x1, y1, x2, y2 for Robot A
	enemy.set_coord(pt_i[1], pt_j[1], pt_i[3], pt_j[3]);	//Save x1, y1, x2, y2 for Robot B

	pt11.fill_wheel_void(view);		//Both wheels will have the same label as the front side of Robot A
	enemy.fill_wheel_void(view);	//Both wheels will have the same label as the front side of Robot B

	view.overwrite_border_labels();	//Attempt to prevent label from propagating to borders

	view.set_processing(11);		//Label everything again to apply fill_wheel_void changes
	view.processing();

	pt11.label_nb_1 = (int)view.label_at_coordinate(pt_i[2] + 15, pt_j[2] + 15);	//Label the front side of Robot A
	pt11.label_nb_2 = (int)view.label_at_coordinate(pt_i[0] + 15, pt_j[0] + 15);	//Label the back side of Robot A

	enemy.label_nb_1 = (int)view.label_at_coordinate(pt_i[1] + 15, pt_j[1] + 15);	//Label the front side of Robot B
	enemy.label_nb_2 = (int)view.label_at_coordinate(pt_i[3] + 15, pt_j[3] + 15);	//Label the back side of Robot B



}

void command_execution(char which_robot, int Robot_Command, bool return_original, bool show_centroids,
	Camera& view, PT11& pt11, PT11& enemy,
	int& pw_l, int& pw_r, int& pw_laser, int& laser,
	int& pw_l_o, int& pw_r_o, int& pw_laser_o, int& laser_o,
	int pt_i[], int pt_j[])
{


	//Write 'A' for Robot A
	//Write 'B' for Robot B
	//char which_robot = 'A';

	//Command 0: Manual Key Control
	//Command 1: Attack Function
	//Command 2: Evade Function
	//Command 3: AI Evolutionary Neural Network (Disabled)
	//Command 4: Scout function (Discontinued)
	//Command 5: Do Nothing

	//int Robot_Command = 1;

	if (which_robot == 'A')
	{
		pt11.collision_points(view);					//Enable collision sensor for Robot A
		pt11.distance_sensor(view, enemy);				//Enable distance sensor for Robot A
		if (Robot_Command == 1) pt11.find_target(enemy);							//Find target and determine which side is it located on
		if (Robot_Command == 2) pt11.highlight_view_evade(view, enemy);		//Enable MR's Safe zone feature
		if (Robot_Command == 2) pt11.get_safe_zone(view, enemy, pt_i, pt_j);	//Enable GG's Safe zone feature
		pt11.highlight_view(view, enemy);				//Enable VFF highlighter
		pt11.enemy_out_of_map(enemy);						//Disable VFF if enemy outside of map
	}

	if (which_robot == 'B')
	{
		enemy.collision_points(view);					//Enable collision sensor for Robot A
		enemy.distance_sensor(view, pt11);				//Enable distance sensor for Robot A
		if (Robot_Command == 1) enemy.find_target(pt11);							//Find target and determine which side is it located on
		if (Robot_Command == 2) enemy.highlight_view_evade(view, pt11);		//Enable MR's Safe zone feature
		if (Robot_Command == 2) enemy.get_safe_zone(view, pt11, pt_i, pt_j);	//Enable GG's Safe zone feature
		enemy.highlight_view(view, pt11);				//Enable VFF highlighter
		enemy.enemy_out_of_map(pt11);						//Disable VFF if enemy outside of map
	}

	switch (Robot_Command)
	{
	case 0:
		if (which_robot == 'A') pt11.manual_set(pw_l, pw_r, pw_laser, laser);		//Control the bot. A W D for laser, arrows for bot
		if (which_robot == 'B') enemy.manual_set(pw_l_o, pw_r_o, pw_laser_o, laser_o);		//Control the bot. A W D for laser, arrows for bot
		break;
	case 1:
		if (which_robot == 'A') pt11.attack(pw_l, pw_r, pw_laser, laser);
		if (which_robot == 'B') enemy.attack(pw_l_o, pw_r_o, pw_laser_o, laser_o);
		break;
	case 2:
		if (which_robot == 'A') pt11.evade(pw_l, pw_r, pw_laser, laser);
		if (which_robot == 'B') enemy.evade(pw_l_o, pw_r_o, pw_laser_o, laser_o);
		break;
	case 3:
		//pt11.NeuroLearn(pw_l, pw_r, laser, trial_number);  //Discontinued! AI keeps resetting because collision_points for some reason...
		break;
	case 4:
		if (which_robot == 'A') enemy.scout(pw_l, pw_r, pw_laser, laser);
		if (which_robot == 'B') enemy.scout(pw_l_o, pw_r_o, pw_laser_o, laser_o);
		break;
	case 5:
		//Don't do anything
		break;
	}

	//if (which_robot == 'A') enemy.manual_set(pw_l_o, pw_r_o, pw_laser_o, laser_o);
	//if (which_robot == 'B') pt11.manual_set(pw_l, pw_r, pw_laser, laser);

	if (show_centroids == 1)
	{
		for (int i = 0; i < 4; i++)
		{
			draw_point_rgb(view.return_image(), pt_i[i], pt_j[i], 0, 255, 255); //Call back array and draw point at those locations
		}
	}

	if (return_original == 1)
	{
		view.set_processing(12);			//Copy 'original' image to 'rgb' image
		view.processing();
	}
}
