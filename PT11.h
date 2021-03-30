#pragma once


class PT11
{
public:
	int pw_l, pw_r, pw_laser, laser;

private:
	double x, dx, ddx;
	double y, dy, ddy;
	double theta, dtheta, ddtheta;
	bool state_laser;

public:
	PT11();
	void manual_set(int& pw_l, int& pw_r, int& pw_laser, int& laser);
	~PT11();
};
