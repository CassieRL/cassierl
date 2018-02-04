/*
 * Cassie2d.h
 *
 *  Created on: Feb 3, 2018
 *      Author: tapgar
 */

#ifndef CASSIE2D_H_
#define CASSIE2D_H_

#include "RobotInterface.h"
#include "DynamicModel.h"
#include "DynamicState.h"
#include "OSC_RBDL.h"
#include "mujoco.h"
#include "CassieVis.h"

class Cassie2d {
public:
  Cassie2d();
  virtual ~Cassie2d();

  void Reset(StateGeneral* state);
  void StepOsc(ControllerOsc* action);
  void StepJacobian(ControllerForce* action);
  void Step(ControllerTorque* action);
  void GetGeneralState(StateGeneral* state);
  void GetOperationalSpaceState(StateOperationalSpace* state);

  bool display_;

private:

  DynamicModel dyn_model_;
  DynamicState dyn_state_;
  OSC_RBDL* osc_;

  CassieVis* vis_;

  mjData* mj_data_;
  mjModel* mj_model_;

  int targetIds[XDD_TARGETS];

  static constexpr double kcx[] = {0.079, -0.079};


};

#endif /* CASSIE2D_H_ */
