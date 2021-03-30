#pragma once

class Camera
{
private:
	image rgb, a, b;
	int cam_number, width, height, type;
	bool state, is_simulator;
	int processing_type;
public:
	Camera(bool state, int cam_number, int width, int height, int type, bool is_simulator, int processing_type);
	int get_cam_number() { return cam_number; }
	bool get_state() { return state; }
	void view();
	void processing();
	image& return_image() { return rgb; }

	~Camera();
};
