#pragma once

class Serial
{
private:
	HANDLE h1;
	char buffer_in[64];
	int speed;
	bool state;
	char u[2];
public:
	Serial(bool state, char COM_number[], int speed);
	void send(char servo_L, char servo_R, char confirm, char flag);
	~Serial();
};
