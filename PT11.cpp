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

#include "PT11.h"


PT11::PT11()
{
	pw_l = 1500;
	pw_r = 1500;
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

PT11::~PT11()
{

}
