/*
 * CassieVis.h
 *
 *  Created on: Jul 3, 2017
 *      Author: tapgar
 */

#ifndef VISUALIZER_H_
#define VISUALIZER_H_

#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include <vector>
#include <Eigen/Dense>
#include "RobotInterface.h"
#include <sys/time.h>
#include "HelperFunctions.h"

#define PAUSE_VIS false
#define TRACKING true

using namespace std;
using namespace Eigen;

class CassieVis {
public:
	CassieVis(mjModel *m, bool save_vid, const char* win_title);
	virtual ~CassieVis();

	int Init(bool save_video, const char* win_title);
	void Close();
	void Scroll(double xoffset, double yoffset);
	void Mouse_Button(int button, int act, int mods);
	void Mouse_Move(double xpos, double ypos);
	void Keyboard(int key, int scancode, int act, int mods);

  bool Draw(mjData* data);

private:

	mjModel *mj_model_;

	GLFWwindow* window_;
	mjvCamera mj_cam_;
	mjvOption mj_opt_;
	mjvScene mj_scn_;
	mjrContext mj_con_;
	unsigned char* image_rgb_;
	float* image_depth_;

	FILE* file_;

	int width_;
	int height_;
	bool save_video_;

	bool button_left_ = false;
	bool button_middle_ = false;
	bool button_right_ =  false;
	double cursor_lastx_ = 0;
	double cursor_lasty_ = 0;

	static constexpr double kcx[] = {0.079, -0.079};

	static constexpr double kmax_frame_rate_ = 60.0;


};

#endif /* VISUALIZER_H_ */
