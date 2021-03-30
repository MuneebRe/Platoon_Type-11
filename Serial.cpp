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

#include "Serial.h"



Serial::Serial(bool state, char COM_number[], int speed)
{
	this->speed = speed;
	this->state = state;

	if (state == false) return;

	open_serial(COM_number, h1, speed);

	cout << "\npress c key to continue, x to quit\n";
	while (!KEY('C')) Sleep(1);
}

void Serial::send(char servo_L, char servo_R, char confirm, char flag)
{

	if (state == false) return;

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
	//Sleep(350);
}

Serial::~Serial()
{
	close_serial(h1);
}