/*
 * Cassie2d.cc
 *
 *  Created on: Feb 3, 2018
 *      Author: tapgar
 */

#include "Cassie2d.h"

// change this to where your files are. Use ABSOLUTE path!
const char* MUJOCO_LICENSE_PATH = "/absolute/path/to/your/mjkey.txt";
const char* XML_FILE_PATH = "/absolute/path/to/your/cassie2d_stiff.xml";

/*
 * external functions used by python interface
 */

extern "C" {

Cassie2d* Cassie2dInit() { return new Cassie2d(); }
void Reset(Cassie2d* cassie, StateGeneral* state) { cassie->Reset(state); }
void StepOsc(Cassie2d* cassie, ControllerOsc* action) { cassie->StepOsc(action); }
void StepTorque(Cassie2d* cassie, ControllerTorque* action) { cassie->Step(action); }
void StepJacobian(Cassie2d* cassie, ControllerForce* action) { cassie->StepJacobian(action); }
void GetGeneralState(Cassie2d* cassie, StateGeneral* state) { cassie->GetGeneralState(state); }
void GetOperationalSpaceState(Cassie2d* cassie, StateOperationalSpace* state) { cassie->GetOperationalSpaceState(state); }
void Display(Cassie2d* cassie, bool display) { cassie->display_ = display; }
void Render(Cassie2d* cassie) { cassie->Render(); }
}

Cassie2d::Cassie2d() {
  // TODO Auto-generated constructor stub

  display_ = false;

  int contactIds[] = {2, 3, 4, 5};
  for (int i = 0; i < XDD_TARGETS; i++)
    targetIds[i] = i+1;

  dyn_state_.Init(contactIds);

  osc_ = new OSC_RBDL(targetIds);
  osc_->AddQDDIdx(2);

  dyn_model_.LoadModel(xml_model_filename);

  //active mujoco and create data
  mj_activate(MUJOCO_LICENSE_PATH);
  char error[1000] = "Could not load model";
  mj_model_ = mj_loadXML(XML_FILE_PATH, 0, error, 1000);
  if (!mj_model_) {
    mju_error_s("Load model error: %s", error);
    return;
  }

  mj_data_ = mj_makeData(mj_model_);

  double qpos_init[] = {0.0, 0.939, 0.0,
      0.68111815, -1.40730357, 1.62972042, -1.77611107, -0.61968407,
      0.68111815, -1.40730353, 1.62972043, -1.77611107, -0.61968402};

  mju_copy(mj_data_->qpos, qpos_init, nQ);

  mj_forward(mj_model_, mj_data_);

  dyn_model_.setState(mj_data_->qpos, mj_data_->qvel);

  osc_->InitMatrices(&dyn_model_);

  vis_ = new CassieVis(mj_model_, false, "cassie");

  display_ = false;

}

Cassie2d::~Cassie2d() {
  // TODO Auto-generated destructor stub
}

void Cassie2d::Reset(StateGeneral* state)
{
  StateGeneralToArray(state, mj_data_->qpos, mj_data_->qvel);
  mj_forward(mj_model_, mj_data_);
}

void Cassie2d::Render() { if (display_) vis_->Draw(mj_data_); }

void Cassie2d::Step(ControllerTorque* action)
{
  dyn_model_.setState(mj_data_->qpos, mj_data_->qvel);
  dyn_state_.UpdateDynamicState(&dyn_model_);

  mju_copy(mj_data_->ctrl, action->torques, nU);
  mj_step(mj_model_, mj_data_);
  Render();
}

void Cassie2d::StepJacobian(ControllerForce* action)
{
  dyn_model_.setState(mj_data_->qpos, mj_data_->qvel);
  dyn_state_.UpdateDynamicState(&dyn_model_);

  Eigen::MatrixXd M = Eigen::MatrixXd::Zero(nQstiff, nQstiff);
  Eigen::VectorXd bias = Eigen::VectorXd::Zero(nQstiff);
  Eigen::MatrixXd Bt = Eigen::MatrixXd::Zero(nQstiff, nU);
  Eigen::MatrixXd Jc = Eigen::MatrixXd::Zero(nCON*DOF, nQstiff);
  Eigen::MatrixXd Jeq = Eigen::MatrixXd::Zero(nEQ, nQstiff);
  Eigen::VectorXd JeqdotQdot = Eigen::VectorXd::Zero(nEQ);
  dyn_state_.GetStiffConfig(&M, &bias, &Bt, &Jc, &Jeq, &JeqdotQdot);

  Eigen::Matrix<double, nQstiff, nQstiff> I = Eigen::Matrix<double, nQstiff, nQstiff>::Identity();
  Eigen::Matrix<double, nQstiff, nQstiff> Hinv = M.inverse();
  Eigen::Matrix<double, nEQ, nEQ> JHinvJ = pseudoinverse(Jeq*Hinv*Jeq.transpose(), 1e-3);

  Eigen::Matrix<double, nQstiff, nQstiff> Nc = I - Jeq.transpose()*JHinvJ*Jeq*Hinv;
  Eigen::Matrix<double, nQstiff, 1> gamma = Jeq.transpose()*JHinvJ*JeqdotQdot;

  Eigen::MatrixXd Jc6 = Eigen::MatrixXd::Zero(12, nQstiff);

  int contactIds[] = {2, 3, 4, 5};
  for (int i = 0; i < 2; i++)
  {
    MatrixXd jc = MatrixXd::Zero(6, nQstiff);
    dyn_model_.GetSite6DJacobian(&jc, contactIds[i]);
    Jc6.block<6,nQstiff>(0,0) += jc;
  }
  for (int i = 2; i < 4; i++)
  {
    MatrixXd jc = MatrixXd::Zero(6, nQstiff);
    dyn_model_.GetSite6DJacobian(&jc, contactIds[i]);
    Jc6.block<6,nQstiff>(6,0) += jc;
  }
  Jc6 /= 2.0;

  Eigen::Matrix<double, 12, 1> f = Eigen::Matrix<double, 12, 1>::Zero();
  //moments then forces
  f(1,0) = action->left_force[2];
  f(3,0) = action->left_force[0];
  f(5,0) = action->left_force[1];
  f(7,0) = action->right_force[2];
  f(9,0) = action->right_force[0];
  f(11,0) = action->right_force[1];

  Eigen::Matrix<double, nU, 1> u = pseudoinverse(Nc*Bt)*(Nc*bias + gamma -Nc*Jc6.transpose()*f);

//  cout << (Nc*bias + gamma).transpose() << endl;
//  cout << (pseudoinverse(Nc*Bt, 1e-3)*(Nc*bias + gamma)).transpose() << endl;
//  cout << f.transpose() << endl;
//  cout << u.transpose() << endl;

  mju_copy(mj_data_->ctrl, u.data(), nU);

  mj_step(mj_model_, mj_data_);

  Render();
}

void Cassie2d::StepOsc(ControllerOsc* action)
{

  dyn_model_.setState(mj_data_->qpos, mj_data_->qvel);
  dyn_state_.UpdateDynamicState(&dyn_model_);

  Eigen::Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, 1> xdd = Eigen::Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, 1>::Zero();

  for (int i = 0; i < 2; i++)
  {
    xdd(i*2,0) = action->body_xdd[i];
    xdd(DOF + i*2, 0) = xdd(2*DOF + i*2, 0) = action->left_xdd[i];
    xdd(3*DOF + i*2, 0) = xdd(4*DOF + i*2, 0) = action->right_xdd[i];
  }
  xdd(XDD_TARGETS*DOF, 0) = action->pitch_add;

  Eigen::Matrix<bool, DOF*XDD_TARGETS+QDD_TARGETS, 1> bActive;
  for (int i = 0; i < DOF*XDD_TARGETS+QDD_TARGETS; i++)
    bActive(i,0) = true;

  bool bDesiredContact[] = {true, true, true, true};

  Eigen::Matrix<double, nU, 1> u = Eigen::Matrix<double, nU, 1>::Zero();
  osc_->RunPTSC(&dyn_model_, &dyn_state_, xdd, bActive, bDesiredContact, &u);

  mju_copy(mj_data_->ctrl, u.data(), nU);

  mj_step(mj_model_, mj_data_);

  Render();
}



void Cassie2d::GetGeneralState(StateGeneral* state)
{
  ArrayToStateGeneral(mj_data_->qpos, mj_data_->qvel, state);
}

void Cassie2d::GetOperationalSpaceState(StateOperationalSpace* state)
{
  Eigen::VectorXd x = Eigen::VectorXd::Zero(DOF*XDD_TARGETS);
  Eigen::VectorXd xd = Eigen::VectorXd::Zero(DOF*XDD_TARGETS);

  dyn_model_.GetTargetPoints(&x, &xd, targetIds);

  for (int i = 0; i < 2; i++)
  {
    state->body_x[i] = x(i*2,0);
    state->body_xd[i] = xd(i*2,0);
    state->left_x[i] = (x(i*2 + DOF, 0) + x(i*2 + 2*DOF, 0))/2.0;
    state->left_xd[i] = (xd(i*2 + DOF, 0) + xd(i*2 + 2*DOF, 0))/2.0;
    state->right_x[i] = (x(i*2 + 3*DOF, 0) + x(i*2 + 4*DOF, 0))/2.0;
    state->right_xd[i] = (xd(i*2 + 3*DOF, 0) + xd(i*2 + 4*DOF, 0))/2.0;
  }
  state->body_x[2] = mj_data_->qpos[2];
  state->body_xd[2] = mj_data_->qvel[2];

}
