
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>

#include <Windows.h>

#include <conio.h>
#include <windows.h>

#include "serial_com.h"

using namespace std;

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

#include "image_transfer.h"

// include this header file for computer vision functions
#include "vision.h"

#include "robot.h"

#include "vision_simulation.h"

#include "timer.h"

extern robot_system S1;

class camera
{
private:
	image rgb, a, b;
	int cam_number, width, height, type;
	bool state, is_simulator;
	int processing_type;
public:
	camera(int cam_number, int width, int height, int type, bool is_simulator, int processing_type);
	int get_cam_number() { return cam_number; }
	void view();
	void view_sim();
	void processing();
	void free();
};

camera::camera(int cam_number, int width, int height, int type, bool is_simulator, int processing_type)
{
	this->cam_number = cam_number;
	this->width = width;
	this->height = height;
	this->is_simulator = is_simulator;
	this->processing_type = processing_type;

	if (is_simulator != true)
	{
		activate_camera(cam_number, height, width);	// activate camera
	}

	rgb.width = width;
	rgb.height = height;
	rgb.type = type;

	a.type = GREY_IMAGE;
	a.width = width;
	a.height = height;

	b.type = GREY_IMAGE;
	b.width = width;
	b.height = height;
	
	allocate_image(rgb);
	allocate_image(a);
	allocate_image(b);
}

void camera::processing()
{
	if (is_simulator == true)
	{
		acquire_image_sim(rgb);
	}
	else
	{
		acquire_image(rgb, cam_number);
	}

	switch (processing_type)
	{
	case 0:

		break;
	case 1:

		break;
	case 2:
		copy(rgb, a);
		copy(a, rgb);
		scale(a, b);
		copy(b, a);
		copy(a, rgb);
		break;
	}
}

void camera::view()
{
	view_rgb_image(rgb);
}

void camera::free()
{
	free_image(rgb);
}

#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

class Serial
{
private:
	HANDLE h1;
	char buffer_in[64];
	int speed;
	char u[2];
public:
	Serial(char COM_number[], int speed);
	void send(char servo_L, char servo_R, char confirm, char flag);
	~Serial();
};

Serial::Serial(char COM_number[], int speed)
{
	this->speed = speed;

	open_serial(COM_number, h1, speed);

	cout << "\npress c key to continue, x to quit\n";
	while (!KEY('C')) Sleep(1);
}

void Serial::send(char servo_L, char servo_R, char confirm, char flag)
{
	u[0] = 0;
	u[1] = 0;

	if (KEY(VK_UP)) u[0] = 20;

	if (KEY(VK_DOWN)) u[0] = -20;

	if (KEY(VK_RIGHT)) u[1] = -15;

	if (KEY(VK_LEFT)) u[1] = 15;


	buffer_in[0] = u[0] + u[1];

	buffer_in[1] = u[0] - u[1];

	if (abs(buffer_in[0]) > 0 || abs(buffer_in[1] > 0))
	{
		buffer_in[2] = 'S';
	}
	else {
		buffer_in[2] = 's';
	}

	cout << (int)buffer_in[0] << "  " << (int)buffer_in[1] << "  " << buffer_in[2] << endl;


	serial_send(buffer_in, 3, h1);
	Sleep(350);
}

Serial::~Serial()
{
	close_serial(h1);
}

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
	int cam_number;
	
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

	// you need to activate the regular vision library before 
	// activating the vision simulation library
	activate_vision();

	// note it's assumed that the robot points upware in its bmp file
	
	// however, Lx, Ly, Ax, Ay assume robot image has already been
	// rotated 90 deg so that the robot is pointing in the x-direction
	// -- ie when specifying these parameters assume the robot
	// is pointing in the x-direction.

	// note that the robot opponent is not currently implemented in 
	// the library, but it will be implemented soon.

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
	pw_l = 1250; // pulse width for left wheel servo (us)
	pw_r = 2000; // pulse width for right wheel servo (us)
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
	pw_l_o = 1300; // pulse width for left wheel servo (us)
	pw_r_o = 1600; // pulse width for right wheel servo (us)
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

	Serial port("COM12", 1);

	int index = 0;

	camera* view[3];
	view[0] = new camera(0, 640, 480, RGB_IMAGE, true, 1);	 //Simulator
	view[1] = new camera(1, 640, 480, RGB_IMAGE, false, 2);	 //Top View Camera
	view[2] = new camera(0, 640, 480, RGB_IMAGE, false, 0);  //Laptop Webcam *to become 1 first person view

	// measure initial clock time
	tc0 = high_resolution_time(); 


	while(1) {
		port.send(0, 0, 0, 0);

		if (KEY('V'))
		{
			index++;
			if (index > 2) index = 0;
			Sleep(350);
		}

		view[index]->processing();

		view[index]->view();


		tc = high_resolution_time() - tc0;

		// fire laser
		if(tc > 1) laser = 1;
		
		if(tc > 9) laser_o = 1;

		// turn off the lasers so we can fire it again later
		if(tc > 10) { 
			laser = 0;
			laser_o = 0;
		}
		
		// fire laser at tc = 14 s
		if(tc > 14) {
			laser = 1;
			
			// turn laser angle alpha at the same time
			pw_laser = 1000;
		}

		// change the inputs to move the robot around
		// or change some additional parameters (lighting, etc.)
		
		// only the following inputs work so far
		// pw_l -- pulse width of left servo (us) (from 1000 to 2000)
		// pw_r -- pulse width of right servo (us) (from 1000 to 2000)
		// pw_laser -- pulse width of laser servo (us) (from 1000 to 2000)
		// -- 1000 -> -90 deg
		// -- 1500 -> 0 deg
		// -- 2000 -> 90 deg
		// laser -- (0 - laser off, 1 - fire laser for 3 s)
		// max_speed -- pixels/s for right and left wheels
		set_inputs(pw_l,pw_r,pw_laser,laser,
			light,light_gradient,light_dir,image_noise,
			max_speed,opponent_max_speed);

		// manually set opponent inputs for the simulation
		// -- good for testing your program
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
