#define KEY(c) ( GetAsyncKeyState((int)(c)) & (SHORT)0x8000 )

using namespace std;

#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <iostream>
#include <fstream>

#include <Windows.h>

#include <conio.h>
#include <windows.h>

#include "image_transfer.h"

#include "vision.h"
#include "robot.h"
#include "vision_simulation.h"
#include "timer.h"

#include "Camera.h"
#include "Serial.h"

int Camera::count = 0;

Camera::Camera(bool state, int cam_number, int width, int height, int type, bool is_simulator, int processing_type)
{
	this->cam_number = cam_number;
	this->width = width;
	this->height = height;
	this->is_simulator = is_simulator;
	this->processing_type = processing_type;
	this->state = state;

	t_value = 79;
	ic = 200.0;
	jc = 300.0;

	if (state != false)
	{
		if (is_simulator != true)
		{
			activate_camera(cam_number, height, width);	// activate camera
		}

		rgb.width = width;
		rgb.height = height;
		rgb.type = type;

		original.width = width;
		original.height = height;
		original.type = type;

		a.type = GREY_IMAGE;
		a.width = width;
		a.height = height;

		b.type = GREY_IMAGE;
		b.width = width;
		b.height = height;

		label.type = LABEL_IMAGE;
		label.width = width;
		label.height = height;

		mag.type = GREY_IMAGE;
		mag.width = width;
		mag.height = height;

		theta.type = GREY_IMAGE;
		theta.width = width;
		theta.height = height;

		allocate_image(original);
		allocate_image(rgb);
		allocate_image(a);
		allocate_image(b);
		allocate_image(label);
		allocate_image(mag);
		allocate_image(theta);

		
	}
	count++;
}

void Camera::acquire()
{
	if (state == false) return;		//If the camera is disabled, do nothing

	if (is_simulator == true)		//If it's a simulator, use the simulator image function
	{								//If it's a camera, use the camera function
		acquire_image_sim(rgb);
	}
	else
	{
		acquire_image(rgb, cam_number);
	}
}

void Camera::processing()
{

	switch (processing_type)
	{
	case 0:
		copy(rgb, original);
		break;
	case 1:
		copy(rgb, a);
		copy(a, rgb);			//Grayscale
		scale(a, b);			//Scale
		copy(b, a);
		threshold(a, b, 100);	//Threshold
		copy(b, a);
		invert(a, b);			//Invert
		copy(b, a);
		erode(a, b);			//Erode
		copy(b, a);
		dialate(a, b);			//Dilate
		copy(b, a);

		copy(a, rgb);

		break;
	case 3:
		copy(original, rgb);
		draw_point_rgb(rgb, ic, jc, 0, 255, 0);
		break;
	case 4: //Our robot's center of rotation finder
		copy(original, rgb);
		red_filter();
		break;
	case 5:
		copy(original, rgb);				//Bring back the original to the rgb
		calculate_hue_image(rgb, a);		//Calculate the hue for each element
		save_hue();							//Save it in a .csv file
		copy(a, rgb);						//Bring it back to the rgb image
		break;
	case 6:				//Centroid from Hue test
		hue_filter(2, 9, 0.55, 0.75, 150, 250);			//Red Filter
		break;
	case 7:
		hue_filter(20, 40, 0.4, 0.6, 200, 260);		//Orange filter
		break;
	case 8:
		hue_filter(145, 165, 0.5, 0.70, 170, 190);	//Green Filter
		break;
	case 9:
		hue_filter(190, 210, 0.7, 0.85, 218, 235);		//Blue Filter
		break;
	case 10:
		copy(original, rgb);
		copy(rgb, a);
		sobel(a, mag, theta);
		copy(theta, rgb);
	case 11:
		label_image(a, label, nlabels);
		break;
	}
	
}

void Camera::view()
{
	if (state == false) return;

	view_rgb_image(rgb);
}

Camera::~Camera()
{
	free_image(rgb);
	free_image(a);
	free_image(b);
	free_image(original);
	free_image(label);
	free_image(mag);
	free_image(theta);
}

int Camera::label_objects()		//Mostly code from the lecture, but used processing() so it's custom.
{
	int nlabels;

	processing();

	// label the objects in a binary image
	// labels go from 1 to nlabels
	label_image(a, label, nlabels);

	return 0; // no errors
}

int Camera::find_object()		//Code from lecture, but edited
{
	cout << "\npress space to get an image";
	pause();

	//acquire_image(rgb, cam_number); // acquire an image from a video source (RGB format)

	// label objects in the image
	label_objects();

	// select an object from the binary image
	select_object();

	cout << "\nobject # = " << nlabel;

	return 0; // no errors
}

int Camera::select_object()		//Code from lecture, combined tracking with RGB to HSV functionality
{
	// select an object from a binary image
// a - image
// b - temp. image
		i2byte* pl;
		int i, j;

		ibyte R, G, B;
		ibyte* p, * pc;
		static double hue = 0;
		static double sat = 0;
		static double value = 0;

		// start in the image
		i = 200;
		j = 300;

		cout << "\nselect an object to track by moving the";
		cout << "\ngrey point to the desired object,";
		cout << "\nand then pressing the c key to continue";

		while (1) {

			// acquire image
			//acquire_image_sim(rgb);
			acquire();

			//REFERENCE MUNEEB
			p = rgb.pdata;

			int k = i + width * j;
			pc = p + 3 * k; // pointer to the kth pixel (3 bytes/pixel)

			B = *pc;
			G = *(pc + 1);
			R = *(pc + 2);

			if(KEY('H'))calculate_HSV(R, G, B, hue, sat, value);

			cout << "Hue: " << hue << "\tSat: " << sat << "\tVal: " << value << endl;

			// label objects
			t_value = 100;
			label_objects();

			copy(a, b); // threshold image is in a
			draw_point(b, i, j, 128); // draw the new point
			copy(b, rgb);


			draw_point_rgb(rgb, i, j, 0, 0, 255);
			draw_point_rgb(rgb, 320, 240, 0, 255, 0);
			view_rgb_image(rgb);

			// read the keyboard if a key is pressed
			if (KEY(VK_UP)) j += 3; // up key
			if (KEY(VK_DOWN)) j -= 3; // down key
			if (KEY(VK_LEFT)) i -= 3; // left key
			if (KEY(VK_RIGHT)) i += 3; // right key

			// limit (i,j) from going off the image
			if (i < 0) i = 0;
			if (i > b.width - 1) i = b.width - 1;
			if (j < 0) j = 0;
			if (j > b.height - 1) j = b.height - 1;

			if (KEY('C')) break;

		} // end while

		// pointer to a label image
		pl = (i2byte*)label.pdata;

		// get the label value at co-ordinate (i,j)
		nlabel = *(pl + j * label.width + i);

		return 0; // no errors
	
}


i2byte Camera::label_at_coordinate(int is, int js)
{
	i2byte* pl;

	ibyte* p, * pc;

	p = rgb.pdata;

	//label_image(a, label, nlabels);

	//draw_point_rgb(rgb, is, js, 0, 255, 0);

	if (is < 0) is = 0;
	if (is > b.width - 1) is = b.width - 1;
	if (js < 0) js = 0;
	if (js > b.height - 1) js = b.height - 1;

	pl = (i2byte*)label.pdata;

	nlabel = *(pl + js * label.width + is);

	return nlabel;
}

int Camera::search_object(int is, int js)		//Code from lecture
// search for a labeled object in an outward spiral pattern
// and inital search location (is,js)
// *** Please study this function carefully
// -- more upcoming assignment and project problems 
// are related to this
{
	i2byte* pl;
	double r, rmax, dr, s, smax, ds, theta;
	int i, j;

	// pointer to a label image
	pl = (i2byte*)label.pdata;

	// check if an object exists at the current location
	nlabel = *(pl + js * label.width + is);
	if (nlabel != 0) return 0;

	rmax = 60.0; // maximum radius of search (pixels)
	dr = 3.0; // radius divisions (pixels)
	ds = 3.0; // arc-length divisions (pixels)

	// search for a labeled object in an outward concentic ring pattern
	for (r = 1.0; r <= rmax; r += dr) {
		smax = 2 * 3.1416 * r; // maximum arc length
		for (s = 0; s <= smax; s += ds) {
			theta = s / r; // s = r*theta
			i = (int)(is + r * cos(theta));
			j = (int)(js + r * sin(theta));

			// limit (i,j) from going off the image
			if (i < 0) i = 0;
			if (i > label.width - 1) i = label.width - 1;
			if (j < 0) j = 0;
			if (j > label.height - 1) j = label.height - 1;

			//			*( b.pdata + j*b.width + i ) = 128; // check pattern

						// check if there is an object at location (i,j)
			nlabel = *(pl + j * label.width + i);
			if (nlabel != 0) return 0;
		}
	}

	return 0; // no errors
}


int Camera::track_object()           //Code from lecture
{

	cout << "\n\nnow tracking the object.";
	cout << "\nif the object moves the centroid marker";
	cout << "\nwill follow the object.";

	// compute the centroid of the object
	centroid(a, label, nlabel, ic, jc);

	label_objects();

	// search for an object at the last known centroid location
	search_object((int)ic, (int)jc);

	// compute the centroid of the object
	centroid(a, label, nlabel, ic, jc);
	cout << "\ncentroid: ic = " << ic << " " << jc;

	draw_point_rgb(rgb, (int)ic, (int)jc, 0, 0, 255);
	draw_point_rgb(rgb, 320, 240, 0, 255, 0);

	return 0;
}

void Camera::red_filter()		//Copy from the "find blue image centroid" assignment
{
	// always initialize summation variables
	double mi, mj, m, eps;
	mi = mj = m = 0.0;
	eps = 1.0e-10;
	int k;
	ibyte R, G, B;
	ibyte* p, * pc;

	p = rgb.pdata;

	copy(original, rgb);

	for (int j = 0; j < height; j++) { // j coord

		for (int i = 0; i < width; i++) { // i coord

			k = i + width * j;
			pc = p + 3 * k; // pointer to the kth pixel (3 bytes/pixel)

			B = *pc;
			G = *(pc + 1);
			R = *(pc + 2);

			// find blue pixels and calculate their centroid stats
			if ((B < 150) && (R < 150) && (G > 170)) {

				R = 255;
				G = 255;
				B = 255;

				// highlight blue pixels in the image
				*(pc + 0) = B;
				*(pc + 1) = G;
				*(pc + 2) = R;

				// to calculate the centroid you need to calculate mk 
				// - the mass of each pixel k

				// mk = volume * density
				// mk = (pixel area) * (blue intensity)
				// mk = (blue intensity) = B
				// since (pixel area) = 1 pixel x 1 pixel = 1

				// calculate total mass m = sum (mk)
				m += B;

				// calculate total moments in the i and j directions
				mi += i * B; // (i moment of mk) = mk * i
				mj += j * B; // (j moment of mk) = mk * j

			} // end if

		} // end for i

	} // end for j

	eps = 1.0e-10; // small constant to protect against /0
	ic = mi / (m + eps);
	jc = mj / (m + eps);

	draw_point_rgb(rgb, ic, jc, 0, 255, 0);

	cout << "\n ic = " << ic << " , jc = " << jc << endl;
}

void Camera::calculate_hue_image(image& rgb, image& hue_image)		//Convert rgb to hue image
{
	int i, j, k;
	ibyte* p; // pointer to colour components in the rgb image
	ibyte* ph; // pointer to the hue image
	int R, G, B, hint;
	double hue, sat, value;

	p = rgb.pdata;
	ph = hue_image.pdata;

	for (j = 0; j < height; j++) {

		for (i = 0; i < width; i++) {

			// equivalent 1D array index k
			k = j * width + i; // pixel number

			// how to get j and i from k ?
			// i = k % width
			// j = (k - i) / width

			// 3 bytes per pixel -- colour in order BGR
			B = p[k * 3]; // 1st byte in pixel
			G = p[k * 3 + 1]; // 2nd byte in pixel
			R = p[k * 3 + 2]; // 3rd

			calculate_HSV(R, G, B, hue, sat, value);

			// hue image value for i,j
			hint = hue / 360.0 * 255; // scale hue from 0 to 255
			ph[k] = hint;

			// note: it might be better to neglect the hue if the sat and value are out of range

		} // for i

	} // for j

}


void Camera::calculate_HSV(int R, int G, int B, double& hue, double& sat, double& value)  //Lecture
{
	int max, min, delta;
	double H;

	max = min = R;

	if (G > max) max = G;
	if (B > max) max = B;

	if (G < min) min = G;
	if (B < min) min = B;

	delta = max - min;

	value = max;

	if (delta == 0) {
		sat = 0.0;
	}
	else {
		sat = delta / value;
	}

	if (delta == 0) {
		H = 0; // hue undefined, maybe set hue = -1
	}
	else if (max == R) {
		H = (double)(G - B) / delta;
	}
	else if (max == G) {
		H = (double)(B - R) / delta + 2;
	}
	else if (max == B) {
		H = (double)(R - G) / delta + 4;
	}

	hue = 60 * H;

	if (hue < 0) hue += 360;

}

void Camera::save_hue()				//Also from his lecture
{
	int nhist;
	double hist[255], hmin, hmax, x;
	FILE* fp;
	nhist = 60;

	histogram(a, hist, nhist, hmin, hmax);

	// save to a csv file you can open/plot with microsoft excel
	fp = fopen("hist1.csv", "w");
	for (int j = 0; j < nhist; j++) {
		if (j != 0) fprintf(fp, "\n");
		x = hmin + (hmax - hmin) / nhist * j;
		fprintf(fp, "%lf , %lf", x, hist[j]);
	}
	fclose(fp);

}

void Camera::hue_filter(double min_hue, double max_hue, double min_sat, double max_sat, double min_val, double max_val)
{	//Re-made the "find blue object centroid" assignment but with HSV range 

	// always initialize summation variables
	double mi, mj, m, eps;
	mi = mj = m = 0.0;
	eps = 1.0e-10;
	int k;
	ibyte R, G, B;
	ibyte* p, * pc;
	double hue, sat, value;

	p = rgb.pdata;

	//copy(original, rgb);

	for (int j = 0; j < height; j++) { // j coord

		for (int i = 0; i < width; i++) { // i coord

			k = i + width * j;
			pc = p + 3 * k; // pointer to the kth pixel (3 bytes/pixel)

			B = *pc;
			G = *(pc + 1);
			R = *(pc + 2);

			calculate_HSV(R, G, B, hue, sat, value);

			// find blue pixels and calculate their centroid stats //8 0.6 0.6
			if ((hue < max_hue) && (hue >= min_hue) && (sat >= min_sat) && (sat < max_sat) && (value < max_val) && (value >= min_val)) {
			//if (hue < 8 && hue > 0 && sat > 0.6 && sat < 1.0 && value > 150 && value < 255) {
				R = 255;
				G = 255;
				B = 255;

				// highlight blue pixels in the image
				*(pc + 0) = B;
				*(pc + 1) = G;
				*(pc + 2) = R;

				// to calculate the centroid you need to calculate mk 
				// - the mass of each pixel k

				// mk = volume * density
				// mk = (pixel area) * (blue intensity)
				// mk = (blue intensity) = B
				// since (pixel area) = 1 pixel x 1 pixel = 1

				// calculate total mass m = sum (mk)
				m += B;

				// calculate total moments in the i and j directions
				mi += i * B; // (i moment of mk) = mk * i
				mj += j * B; // (j moment of mk) = mk * j

			} // end if

		} // end for i

	} // end for j

	eps = 1.0e-10; // small constant to protect against /0
	ic = mi / (m + eps);
	jc = mj / (m + eps);

	//draw_point_rgb(rgb, ic, jc, 0, 255, 0);

	//cout << "\n ic = " << ic << " , jc = " << jc << endl;
}


int Camera::sobel(image& a, image& mag, image& theta)	//Full copy paste from the lecture material
{
	i4byte size, i, j;
	ibyte* pa, * pa1, * pa2, * pa3, * pa4, * pa5, * pa6, * pa7, * pa8, * pa9;
	ibyte* p_mag, * p_theta;
	i2byte width, height;

	// note we use a signed in here since sx, sy could be < 0
	int sx, sy, M;
	int kx[10], ky[10];
	double A;

	// check for compatibility image sizes and types
	if (a.height != mag.height || a.width != mag.width ||
		a.height != theta.height || a.width != theta.width)
	{
		printf("\nerror in convolution: sizes images are not the same!");
		return 1;
	}

	if (a.type != GREY_IMAGE || mag.type != GREY_IMAGE
		|| theta.type != GREY_IMAGE)
	{
		printf("\nerror in convolution: input types are not valid!");
		return 1;
	}

	width = a.width;
	height = a.height;

	// initialize pointers
	pa = a.pdata + width + 1;
	p_mag = mag.pdata + width + 1;
	p_theta = theta.pdata + width + 1;

	// set neighbourhood pointers

	// make sure they don't point outside of the images at the boundaries
	// when you use them

	// note the order of the neighbourhood is correctly given below
	// as discussed in class (the old order was for a different
	// image coord system in an older version of the library).
	// pa7 pa8 pa9
	// pa4 pa5 pa6
	// pa1 pa2 pa3
	pa1 = pa - width - 1;
	pa2 = pa - width;
	pa3 = pa - width + 1;
	pa4 = pa - 1;
	pa5 = pa;
	pa6 = pa + 1;
	pa7 = pa + width - 1;
	pa8 = pa + width;
	pa9 = pa + width + 1;

	// number of pixels to process
	size = (i4byte)a.width * a.height - 2 * width - 2;

	// set convolution coefficients for sx and sy
	// k7 k8 k9
	// k4 k5 k6
	// k1 k2 k3
	kx[7] = -1; kx[8] = 0; kx[9] = 1;
	kx[4] = -2; kx[5] = 0; kx[6] = 2;
	kx[1] = -1; kx[2] = 0; kx[3] = 1;

	ky[7] = 1;  ky[8] = 2;  ky[9] = 1;
	ky[4] = 0;  ky[5] = 0;  ky[6] = 0;
	ky[1] = -1; ky[2] = -2; ky[3] = -1;

	// calculate sx and sy
	// here I calculate both at the same time in the loop
	// since I don't want to store them into an image array
	// (they can't store negative numbers which might occur for sx
	// and sy) and I need both to calculate mag and theta.
	for (i = 0; i < size; i++) {

		sx = kx[1] * (*pa1) + kx[2] * (*pa2) + kx[3] * (*pa3) +
			kx[4] * (*pa4) + kx[5] * (*pa5) + kx[6] * (*pa6) +
			kx[7] * (*pa7) + kx[8] * (*pa8) + kx[9] * (*pa9);

		sy = ky[1] * (*pa1) + ky[2] * (*pa2) + ky[3] * (*pa3) +
			ky[4] * (*pa4) + ky[5] * (*pa5) + ky[6] * (*pa6) +
			ky[7] * (*pa7) + ky[8] * (*pa8) + ky[9] * (*pa9);

		// might consider directly substituting kx, ky above
		// to reduce computation time

		// calculate mag and theta
		M = abs(sx) + abs(sy); // fast approx of sqrt(sx*sx + sy*sy)

		if (M > 255) M = 255; // check for overflow
		// alternatively M can be scaled by 1/2, 1/4, 1/8
		// to reduce (1/2) or avoid (1/8) possibility of overlow

		*p_mag = M;

		A = atan2((double)sy, (double)sx) / 3.14159 * 180; // deg
		// note that A ranges from -180 to 180 deg

		// scale A so that it ranges from 0 to 255
		// and will fit in a greyscale image range
		// -- add 0.01 to account for roundoff error
		A = (A + 180) / 360 * 255 + 0.01;

		*p_theta = (int)A;

		// note this line might be useful to cut down
		// on the noise / irrelevant info from theta
		if (M < 75) *p_theta = 0;

		// increment pointers
		pa1++; pa2++; pa3++; pa4++; pa5++;
		pa6++; pa7++; pa8++; pa9++;
		p_mag++, p_theta++;
	}

	// copy edges of image from valid regions
	p_mag = mag.pdata;
	p_theta = theta.pdata;

	// number of pixels
	size = (i4byte)a.width * a.height;

	for (i = 0; i < width; i++) {
		p_mag[i] = p_mag[i + width]; // bottom
		p_mag[size - i - 1] = p_mag[size - i - 1 - width]; // top
		p_theta[i] = p_theta[i + width]; // bottom
		p_theta[size - i - 1] = p_theta[size - i - 1 - width]; // top
	}

	for (i = 0, j = 0; i < height; i++, j += width) {
		p_mag[j] = p_mag[j + 1]; // left
		p_mag[size - j - 1] = p_mag[size - j - 2]; // right
		p_theta[j] = p_theta[j + 1]; // left
		p_theta[size - j - 1] = p_theta[size - j - 2]; // right
	}

	return 0;
}
