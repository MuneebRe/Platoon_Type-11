#pragma once

class Camera
{
private:
	image rgb, a, b, original, label, mag, theta;
	i2byte nlabel;
	int nlabels;
	int cam_number, width, height, type;
	bool state, is_simulator;
	int processing_type;
	static int count;
	int t_value;
	
	int first_finder_x[4];
	int first_finder_y[4];

	int is, js;
	double ic, jc;

public:
	Camera(bool state, int cam_number, int width, int height, int type, bool is_simulator, int processing_type);
	int get_cam_number() { return cam_number; }
	bool get_state() { return state; }
	void view();
	void processing();
	image& return_image() { return rgb; }
	image& return_a() { return a; }
	image& return_label() { return label; }
	int get_count() { return count; }
	void set_processing(int processing_type) { this->processing_type = processing_type; }
	void acquire();

	void calculate_HSV(int R, int G, int B, double& hue, double& sat, double& value);
	void calculate_hue_image(image& rgb, image& hue_image);
	void save_hue();
	double get_ic() { return ic; }
	double get_jc() { return jc; }

	int label_objects();
	int select_object();
	int find_object();
	int track_object();
	int search_object(int is, int js);
	i2byte label_at_coordinate(int is, int js);

	void red_filter();
	void hue_filter(double min_hue, double max_hue, double min_sat, double max_sat, double min_val, double max_val);
	void hue_filter2(double min_hue, double max_hue, double min_sat, double max_sat, double min_val, double max_val);

	int sobel(image& a, image& mag, image& theta);

	void draw_border();
	void overwrite_border_labels();

	int Area_of_label(i2byte label_selection);
	void coordinate_finder(int pt_i[], int pt_j[]);

	~Camera();
};
