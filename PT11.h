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

	int Lx[4];
	int Ly[4];
	int LL[4];
	int Ln[4];

	bool collision_state[4];
	double timer1[4];
	double timer2[4];
	bool timer_flag;

	double xg[4], yg[4];

public:
	PT11();
	void manual_set(int& pw_l, int& pw_r, int& pw_laser, int& laser);
	void set_coord(double x1, double y1, double x2, double y2);
	void collision_points(Camera &view);
	void check_collision(int arrx[], int arry[], Camera &view, int i);
	~PT11();
};

