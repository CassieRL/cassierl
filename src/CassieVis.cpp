/*
 * CassieVis.cpp
 *
 *  Created on: Jul 3, 2017
 *      Author: tapgar
 */

#include "CassieVis.h"

using namespace std;
using namespace Eigen;

CassieVis::CassieVis(mjModel *m, bool save_vid, const char* win_title) {
	mj_model_ = m;
	width_ = 1200;
	height_ = 900;

	Init(save_vid, win_title);
}

CassieVis::~CassieVis() {
	// TODO Auto-generated destructor stub
}

static void window_close_callback(GLFWwindow* window)
{
	((CassieVis*)(glfwGetWindowUserPointer(window)))->Close();
}
static void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
	((CassieVis*)(glfwGetWindowUserPointer(window)))->Scroll(xoffset, yoffset);
}
static void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
	((CassieVis*)(glfwGetWindowUserPointer(window)))->Mouse_Move(xpos, ypos);
}
static void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
	((CassieVis*)(glfwGetWindowUserPointer(window)))->Mouse_Button(button, act, mods);
}

static void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
	((CassieVis*)(glfwGetWindowUserPointer(window)))->Keyboard(key, scancode, act, mods);
}


int CassieVis::Init(bool save_video, const char* win_title) {

	if (!glfwInit()) {
		mju_error("Could not initialize GLFW");
		return 1;
	}

	// Create window
	window_ = glfwCreateWindow(width_, height_, win_title, NULL, NULL);
	glfwMakeContextCurrent(window_);
	glfwSwapInterval(1);
	mjv_defaultCamera(&mj_cam_);
	// Set up mujoco visualization objects
	mj_cam_.lookat[0] = mj_model_->stat.center[0];
	mj_cam_.lookat[1] = mj_model_->stat.center[1];
	mj_cam_.lookat[2] = 0.3 + mj_model_->stat.center[2];
	mj_cam_.type = mjCAMERA_FREE;
	mj_cam_.distance = 2.0;

#if TRACKING

	mj_cam_.azimuth = 0;


	mj_cam_.distance = 3.5;
	mj_cam_.lookat[2] += 0.8;
	mj_cam_.type = mjCAMERA_TRACKING;
	mj_cam_.trackbodyid = 1;

#endif
	mj_cam_.elevation = 0;

	mjv_defaultOption(&mj_opt_);
	mjr_defaultContext(&mj_con_);
	mjv_makeScene(&mj_scn_, 1E5);

	mjr_makeContext(mj_model_, &mj_con_, mjFONTSCALE_100);

	mjv_moveCamera(mj_model_, mjMOUSE_ROTATE_H, 0.0, 0.0, &mj_scn_, &mj_cam_);

	// Set callback for user-initiated window close events
	glfwSetWindowUserPointer(window_, this);
	glfwSetWindowCloseCallback(window_, window_close_callback);
	glfwSetCursorPosCallback(window_, mouse_move);
	glfwSetMouseButtonCallback(window_, mouse_button);
	glfwSetScrollCallback(window_, scroll);
	glfwSetKeyCallback(window_, keyboard);

	if (save_video)
	{
		image_rgb_ = (unsigned char*)malloc(3*width_*height_);
		image_depth_ = (float*)malloc(sizeof(float)*width_*height_);

		// create output rgb file
		file_ = fopen("../out/temp.out", "wb");
		if( !file_ )
			mju_error("Could not open rgbfile for writing");
	}

	save_video_ = save_video;

	return 0;
}

bool CassieVis::Draw(mjData* data)
{
	if (!window_)
		return false;

	timespec ts;
	static timespec tf;
	static bool first_time = true;

	clock_gettime(CLOCK_REALTIME, &ts);
	double freq = 1e9/double(diff(tf,ts).tv_nsec);
	if (freq > kmax_frame_rate_ && !first_time)
		return false;
	first_time = false;

	mjrRect viewport = {0, 0, 0, 0};

  // Set up for rendering
  glfwMakeContextCurrent(window_);
  glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);

  mjv_updateScene(mj_model_, data, &mj_opt_, NULL, &mj_cam_, mjCAT_ALL, &mj_scn_);

  glfwGetFramebufferSize(window_, &viewport.width, &viewport.height);


  mjr_render(viewport, &mj_scn_, &mj_con_);

  // Show updated scene
  glfwSwapBuffers(window_);
  glfwPollEvents();

  if (save_video_)
  {
    mjr_readPixels(image_rgb_, image_depth_, viewport, &mj_con_);
    fwrite(image_rgb_, 3, width_*height_, file_);
  }

	clock_gettime(CLOCK_REALTIME, &tf);

	return false;
}

// mouse button
void CassieVis::Mouse_Button(int button, int act, int mods)
{
	// update button state
	button_left_ =   (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
	button_middle_ = (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
	button_right_ =  (glfwGetMouseButton(window_, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

	// update mouse position
	glfwGetCursorPos(window_, &cursor_lastx_, &cursor_lasty_);
}

// mouse move
void CassieVis::Mouse_Move(double xpos, double ypos)
{
	// no buttons down: nothing to do
	if( !button_left_ && !button_middle_ && !button_right_ )
		return;

	// compute mouse displacement, save
	double dx = xpos - cursor_lastx_;
	double dy = ypos - cursor_lasty_;
	cursor_lastx_ = xpos;
	cursor_lasty_ = ypos;

	// get current window size
	int width, height;
	glfwGetWindowSize(window_, &width, &height);

	// get shift key state
	bool mod_shift = (glfwGetKey(window_, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
			glfwGetKey(window_, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

	// determine action based on mouse button
	mjtMouse action;
	if( button_right_ )
		action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
	else if( button_left_ )
		action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
	else
		action = mjMOUSE_ZOOM;

	mjv_moveCamera(mj_model_, action, dx/height, dy/height, &mj_scn_, &mj_cam_);
}

void CassieVis::Scroll(double xoffset, double yoffset)
{
	// scroll: emulate vertical mouse motion = 5% of window height
	mjv_moveCamera(mj_model_, mjMOUSE_ZOOM, 0, -0.05*yoffset, &mj_scn_, &mj_cam_);
}

// keyboard
void CassieVis::Keyboard(int key, int scancode, int act, int mods)
{
	// do not act on release
	if( act==GLFW_RELEASE )
		return;

	if (key == GLFW_KEY_W)
		mj_cam_.azimuth += 45;
	if (key == GLFW_KEY_Q)
		mj_cam_.azimuth -= 45;

}

void CassieVis::Close() {
	// Free mujoco objects
	mjv_freeScene(&mj_scn_);
	mjr_freeContext(&mj_con_);

	// Close window
	glfwDestroyWindow(window_);
	window_ = NULL;
}
