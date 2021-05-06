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

	bool collision_state[4];		//0 Front- 1 Right- 2 Back- 3 Left
	bool collision_t_flag[4];		//Timer function on collision
	double collision_t1[4];
	double collision_t2[4];
	double collision_dt[4];
	double collision_dt_target[4];

	int label_nb_1;		//Label of front side of robot
	int label_nb_2;		//Label of back side of robot


private:
	image radar_rgb, radar_greyscale, safezone_greyscale, safezone_label, radar_a, radar_b;	//Radar-Evasion
	i2byte radar_nlabel;		//Radar-Evasion
	int radar_nlabels; //Radar-Evasion: number of objects detected when thresholding radar greyscale
	int safezone_centroid_x[50], safezone_centroid_y[50], safezone_array_index; //This will house the i and j value of the safe zones, which will be used to assess which is closer and for processing (50 assumes only 50 safezones max)
	double x1,x2, dx, ddx;		//x1 front, x2 back
	double y1,y2, dy, ddy;		//y1 front, y2 back
	double theta, dtheta, ddtheta;	//Theta 0 - 2 PI
	double theta_target1, theta_target2;	//Relative theta against front side enemy
	double target_delta1, target_delta2;	//Relative theta against back side enemy
	double trigger_range;
	bool target_state;
	bool state_laser;
	bool state_dir[2];	//Trigger to determine which side enemy located
	bool flag_reset;
	bool is_there_obstacle;
	double distance_enemy1, distance_enemy2, distance_enemy_avg;

	bool attack_trigger;
	bool evade_trigger;

	//Variables for distance sensors, 8 sides
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

	//Virtual Force Field system
	double VFF_theta;
	double VFF_mag;

	bool disable_system;
	bool disable_laser;

public:
	//Muneeb functions:
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
	void theta_target_delta_fix(double target_theta,double& target_delta);
	bool get_reset_state() { return flag_reset; }

	void distance_sensor(Camera& view, PT11 &enemy);
	void distance_input(int arrx[], int arry[], Camera& view, int i);
	void is_obstacle_before_enemy(int arrx[], int arry[], PT11 enemy, Camera& view);
	void label_enemy(Camera& view, PT11 enemy);
	void fill_wheel_void(Camera& view);	//Combines front wheel objects with front circle object into one object
	
	void NeuroLearn(int& pw_l, int& pw_r, int& laser, int &trial_number);

	void scout(int& pw_l, int& pw_r, int& pw_laser, int& laser);
	void attack(int& pw_l, int& pw_r, int& pw_laser, int& laser);
	void evade(int& pw_l, int& pw_r, int& pw_laser, int& laser);

	void highlight_view(Camera& view, PT11 enemy);
	void highlight_view_evade(Camera& view, PT11 enemy);

	void hide_shadows(int arrx[], int arry[], Camera& view, double theta_index, int& radar_radius, int radius_limit, bool& enemy_trigger, PT11 enemy, int radius_jump, bool& shadow_zone_trigger);
	void hide_shadows_evade(int arrx[], int arry[], Camera& view, double theta_index,int& radar_minimum, int& radar_radius, int radius_limit, bool& enemy_trigger, PT11 enemy, int radius_jump);

	void VFF_section_modifier(double theta_index, double offset, double range, int& radius_limit, int limit_val, double& multiplier, double multiplier_val);

	void enemy_out_of_map(PT11 enemy);

	~PT11();

	//Gurv functions:
	void acquire_camera_image(Camera& view);   //Radar-Evasion: Assigns rgb, greyscale and label image to their respect PT11 objects for processing.
	void get_safe_zone(Camera& view, PT11& enemy, int pt_i[4], int pt_j[4]);			//Radar-Evasion: Compiles all pixels into "vision" lines expanding from robot centroid for Radar processing
	void draw_safe_zone(int* line_array_i, int* line_array_j, int size, Camera& view, PT11& enemy);		//Radar-Evasion: Radar processing function to determine safe zones and store inro rgb image for masking purposes
	void threshold_radar(Camera& view, PT11& enemy, int pt_i[4], int pt_j[4]);
	void assess_safe_zone(Camera& view);
	int radar_centroid(image& a, image& label, int nlabel, double& ic, double& jc, int& flag);
	void radar_evasion(int pt_i[4], int pt_j[4], PT11& enemy); //Using VFF
};