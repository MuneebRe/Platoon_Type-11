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
	image radar_label, radar_rgb, radar_greyscale;	//Radar-Evasion
	i2byte radar_nlabel;		//Radar-Evasion
	int radar_nlabels, radar_robot_objects[5]; //Radar-Evasion: radar_robot_objects array will hold object labels for radar detection filtering
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

	double VFF_theta;
	double VFF_mag;

public:
	PT11(Camera& view);
	void init_neural();
	void manual_set(int& pw_l, int& pw_r, int& pw_laser, int& laser);
	void set_coord(double x1, double y1, double x2, double y2);
	void collision_points(Camera &view);
	void check_collision(int arrx[], int arry[], Camera &view, int i);
	double get_theta() { return theta; }
	void find_target(PT11 enemy);
	double get_x1() { return x1; }	//Centroid of front robot circle
	double get_y1() { return y1; }	//Centroid of front robot circle
	double get_x2() { return x2; }	//Centroid of back robot circle
	double get_y2() { return y2; }	//Centroid of back robot circle
	void calculate_theta(double x1, double y1, double x2, double y2, double &theta);
	void theta_target_delta_fix(double target_theta,double& target_delta, int& aim_dir);
	bool get_reset_state() { return flag_reset; }

	void distance_sensor(Camera& view, PT11 enemy);
	void distance_input(int arrx[], int arry[], Camera& view, int i);
	void is_obstacle_before_enemy(int arrx[], int arry[], PT11 enemy, Camera& view);
	void label_enemy(Camera& view, PT11 enemy);
	void fill_wheel_void(Camera& view);	//Combines front wheel objects with front circle object into one object

	void m_runNet(int& pw_l, int& pw_r, int& laser);
	//void NeuroNet(int pw_l, int pw_r);
	void NeuroLearn(int& pw_l, int& pw_r, int& laser, int &trial_number);

	void scout(int& pw_l, int& pw_r, int& pw_laser, int& laser);
	void flee(int& pw_l, int& pw_r, int& pw_laser, int& laser, int tc0);
	void attack(int& pw_l, int& pw_r, int& pw_laser, int& laser);

	void highlight_view(Camera& view, PT11 enemy);
	void hide_shadows(int arrx[], int arry[], Camera& view, double theta_index, int& radar_radius, int radius_limit, bool& enemy_trigger, PT11 enemy, int radius_jump);
	void VFF_section_modifier(double theta_index, double offset, double range, int& radius_limit, int limit_val, double& multiplier, double multiplier_val);

	void acquire_camera_image(Camera& view);   //Radar-Evasion: Assigns rgb, greyscale and label image to their respect PT11 objects for processing.
	void get_safe_zone(Camera& view, PT11 enemy, int pt_i[4], int pt_j[4]);			//Radar-Evasion: Compiles all pixels into "vision" lines expanding from robot centroid for Radar processing
	void draw_safe_zone(int* line_array_i, int* line_array_j, int size, Camera& view, PT11 enemy);		//Radar-Evasion: Radar processing function to determine safe zones and store inro rgb image for masking purposes
	//Might remove identify_radar_objects() function! 
	void identify_radar_objects(int pt_i[4], int pt_j[4], Camera& view); //Radar-Evasion: Radar processing function to distinguish between obstacles and robots - necessary for draw_safe_zone

	~PT11();
};