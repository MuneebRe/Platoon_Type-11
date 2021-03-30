#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>

#include <Windows.h>

#include <conio.h>
#include <windows.h>

#include "image_transfer.h"

#include "vision.h"
#include "robot.h"
#include "vision_simulation.h"
#include "timer.h"

#include "Camera.h"
#include "Serial.h"

Camera::Camera(bool state, int cam_number, int width, int height, int type, bool is_simulator, int processing_type)
{
	this->cam_number = cam_number;
	this->width = width;
	this->height = height;
	this->is_simulator = is_simulator;
	this->processing_type = processing_type;
	this->state = state;

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

void Camera::processing()
{
	if (state == false) return;

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

void Camera::view()
{
	if (state == false) return;

	view_rgb_image(rgb);
}

Camera::~Camera()
{
	free_image(rgb);
}
