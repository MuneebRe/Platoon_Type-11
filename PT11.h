#pragma once
# define M_PI           3.14159265358979323846  /* pi */
#include "Neural_Network/NeuroNet.h"
#include "Neural_Network/Input.h"
#include "Neural_Network/Hidden.h"
#include "Neural_Network/Output.h"

class PT11
{
public:
	int pw_l, pw_r, pw_laser, laser;

	bool collision_state[4];
	bool collision_t_flag[4];
	double collision_t1[4];
	double collision_t2[4];
	double collision_dt[4];
	double collision_dt_target[4];
	bool collision_reset;

	bool net_mem[7];
	double net_out[3];

	int label_nb_1;
	int label_nb_2;


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
	bool flag_reset;
	bool is_there_obstacle;
	double distance_enemy1;

	int Lx[8];
	int Ly[8];
	int LL[8];
	int Ln[8];

	int Ax[8];
	int Ay[8];
	double AF[8];
	double distance_log[8];

	Neural_Net* topology;
	double trial_timer1, trial_timer2, trial_dt;

public:
	PT11();
	void init_neural();
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
	bool get_reset_state() { return flag_reset; }

	void distance_sensor(Camera& view, PT11 enemy);
	void distance_input(int arrx[], int arry[], Camera& view, int i);
	void is_obstacle_before_enemy(int arrx[], int arry[], PT11 enemy, Camera& view);
	void label_enemy(Camera& view, PT11 enemy);
	void fill_wheel_void(Camera& view);

	void m_runNet(int& pw_l, int& pw_r, int& laser);
	//void NeuroNet(int pw_l, int pw_r);
	void NeuroLearn(int& pw_l, int& pw_r, int& laser, int &trial_number);

	void scout(int& pw_l, int& pw_r, int& pw_laser, int& laser);
	void flee(int& pw_l, int& pw_r, int& pw_laser, int& laser, int tc0);
	void attack(int& pw_l, int& pw_r, int& pw_laser, int& laser);

	void highlight_view(Camera& view, PT11 enemy);
	void hide_shadows(int arrx[], int arry[], Camera& view, double theta_index, int& radar_radius, int radius_limit, bool& enemy_trigger, PT11 enemy, int radius_jump);

	~PT11();
};