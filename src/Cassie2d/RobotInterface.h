/*
 * RobotInterface.h
 *
 *  Created on: Feb 3, 2018
 *      Author: tapgar
 */

#ifndef ROBOTINTERFACE_H_
#define ROBOTINTERFACE_H_

#include <string>

// Change this to where your files are. Use ABSOLUTE path!
static const std::string xml_model_filename = "home/phi/work/cassierl/model/cassie2d_stiff.xml";

struct ControllerTorque {
  double torques[6];
};

struct ControllerForce {
  double left_force[3];
  double right_force[3];
};

struct ControllerOsc {
  double body_xdd[2];
  double left_xdd[2];
  double right_xdd[2];
  double pitch_add;
};

struct StateGeneral {
  double base_pos[3];
  double base_vel[3];
  double left_pos[5];
  double left_vel[5];
  double right_pos[5];
  double right_vel[5];
};

struct StateOperationalSpace {
  double body_x[3];
  double body_xd[3];
  double left_x[3];
  double left_xd[3];
  double right_x[3];
  double right_xd[3];
};

#define nQstiff 13
#define nQ 13 //# joints
#define nX nQ //pos=vel+1 (quaternion)
#define nU 6 //# actuated joints
#define nCON 4 //# contact points
#define nEQ 6 //#of equality constraints 4*3

#define XDD_TARGETS 5 //# of cartesian target accelerations
#define QDD_TARGETS 1 //# of joint target accels

#define DOF 3 // 2D vs 3D

static const double mu = 0.5;

static void StateOperationalSpaceToArray(StateOperationalSpace* state, double* pos, double* vel)
{
  for (int i = 0; i < 3; i++)
    pos[i] = state->body_x[i];
  for (int i = 0; i < 3; i++)
    vel[i] = state->body_xd[i];
  for (int i = 0; i < 3; i++)
    pos[i+3] = state->left_x[i];
  for (int i = 0; i < 3; i++)
    vel[i+3] = state->left_xd[i];
  for (int i = 0; i < 3; i++)
    pos[i+6] = state->right_x[i];
  for (int i = 0; i < 3; i++)
    vel[i+6] = state->right_xd[i];
}

static void ArrayToStateOperationalSpace(double* pos, double* vel, StateOperationalSpace* state)
{
  for (int i = 0; i < 3; i++)
    state->body_x[i] = pos[i];
  for (int i = 0; i < 3; i++)
    state->body_xd[i] = vel[i];
  for (int i = 0; i < 3; i++)
    state->left_x[i] = pos[i+3];
  for (int i = 0; i < 3; i++)
    state->left_xd[i] = vel[i+3];
  for (int i = 0; i < 3; i++)
    state->right_x[i] = pos[i+6];
  for (int i = 0; i < 3; i++)
    state->right_xd[i] = vel[i+6];
}

static void StateGeneralToArray(StateGeneral* state, double* pos, double* vel)
{
  for (int i = 0; i < 3; i++)
    pos[i] = state->base_pos[i];
  for (int i = 0; i < 3; i++)
    vel[i] = state->base_vel[i];
  for (int i = 0; i < 5; i++)
    pos[i+3] = state->left_pos[i];
  for (int i = 0; i < 5; i++)
    vel[i+3] = state->left_vel[i];
  for (int i = 0; i < 5; i++)
    pos[i+8] = state->right_pos[i];
  for (int i = 0; i < 5; i++)
    vel[i+8] = state->right_vel[i];
}

static void ArrayToStateGeneral(double* pos, double* vel, StateGeneral* state)
{
  for (int i = 0; i < 3; i++)
    state->base_pos[i] = pos[i];
  for (int i = 0; i < 3; i++)
    state->base_vel[i] = vel[i];
  for (int i = 0; i < 5; i++)
    state->left_pos[i] = pos[i+3];
  for (int i = 0; i < 5; i++)
    state->left_vel[i] = vel[i+3];
  for (int i = 0; i < 5; i++)
    state->right_pos[i] = pos[i+8];
  for (int i = 0; i < 5; i++)
    state->right_vel[i] = vel[i+8];
}


#endif /* ROBOTINTERFACE_H_ */
