#pragma once
# define M_PI           3.14159265358979323846  /* pi */

class PT11
{
public:
	int pw_l, pw_r, pw_laser, laser;

private:
	double x1,x2, dx, ddx;
	double y1,y2, dy, ddy;
	double theta, dtheta, ddtheta;
	bool state_laser;

	double Lx[4];
	double Ly[4];
	double LL[4];
	double Ln[4];

	double xg[4], yg[4];

public:
	PT11();
	void manual_set(int& pw_l, int& pw_r, int& pw_laser, int& laser);
	void set_coord(double x1, double y1, double x2, double y2);
	void collision_points(image& rgb);
	~PT11();
};

