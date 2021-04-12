#pragma once
# define M_PI           3.14159265358979323846  /* pi */

class PT11
{
public:
	int pw_l, pw_r, pw_laser, laser;

	bool collision_state[4];
	bool collision_t_flag[4];
	double collision_t1[4];
	double collision_t2[4];
	double collision_dt[4];

	bool net_mem[7];
	double net_out[3];

private:
	double x1,x2, dx, ddx;
	double y1,y2, dy, ddy;
	double theta, dtheta, ddtheta;
	double theta_target1, theta_target2;
	double target_delta1, target_delta2;
	double trigger_range;
	bool target_state;
	bool state_laser;
	bool state_dir[2];	//Trigger to determine which side enemy located


	int Lx[4];
	int Ly[4];
	int LL[4];
	int Ln[4];

public:
	PT11();
	void manual_set(int& pw_l, int& pw_r, int& pw_laser, int& laser);
	void set_coord(double x1, double y1, double x2, double y2);
	void collision_points(Camera &view);
	void check_collision(int arrx[], int arry[], Camera &view, int i);
	double get_theta() { return theta; }
	void find_target(PT11 enemy);
	double get_x1() { return x1; }
	double get_y1() { return y1; }
	double get_x2() { return x2; }
	double get_y2() { return y2; }
	void calculate_theta(double x1, double y1, double x2, double y2, double &theta);

	void m_runNet(int& pw_l, int& pw_r, int& laser);
	//void NeuroNet(int pw_l, int pw_r);
	~PT11();
};