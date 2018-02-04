/*
 * DynamicModel.cpp
 *
 *  Created on: Dec 20, 2017
 *      Author: tapgar
 */

#include "DynamicModel.h"
#include "xml_parser.h"

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

DynamicModel::DynamicModel() {
  // TODO Auto-generated constructor stub
  total_mass_ = 0.0;
}

DynamicModel::~DynamicModel() {
  // TODO Auto-generated destructor stub
}

void DynamicModel::LoadModel(std::string xml_file) {

  std::vector<XML_Parser::body> bodies;
  std::vector<XML_Parser::site> sites;
  std::vector<XML_Parser::constraint> constraints;
  std::vector<XML_Parser::motor> motors;

  std::vector<std::vector<unsigned int>> id_lookup;

  Vector3d com;
  Matrix3d I;

  Vector3d pos;
  Matrix3d rot;

  joint_daming_ = VectorNd::Zero(nX);
  joint_stiffness_ = VectorNd::Zero(nX);
  selector_matrix_ = MatrixNd::Zero(nQ, nU);
  motor_limits_ = MatrixNd::Zero(nU,2);
  joint_reference_ = VectorNd::Zero(nX);

  //	printf("parsing %s\n", xml_file.c_str());
  unsigned int parent_id = 0;
  XML_Parser::parse_xml_model(xml_file, &bodies, &sites, &constraints, &motors);
  //	printf("parsed\n");

  std::vector<double> rotor_inertia;
  std::vector<double> ref;

  for (unsigned int i = 0; i < bodies.size(); i++)
  {
    Joint joint = XML_Parser::joint_list_to_joint(bodies[i].joints);

    //		printf("%d\t%d\n", i, joint.mJointType);

    if (joint.mJointType == RigidBodyDynamics::JointTypeFloatingBase)
    {
      for (unsigned int j = 0; j < 6; j++)
      {
        rotor_inertia.push_back(0.0);
        ref.push_back(0.0);
      }
    }
    else
    {
      for (unsigned int j = 0; j < bodies[i].joints.size(); j++)
      {
        rotor_inertia.push_back(bodies[i].joints[j].armature);
        ref.push_back(bodies[i].joints[j].ref);
        joint_daming_(ref.size()-1) = bodies[i].joints[j].damping;
        joint_stiffness_(ref.size()-1) = bodies[i].joints[j].stiffness;
        joint_reference_(ref.size()-1) = bodies[i].joints[j].ref;
      }
    }
    for (int j = 0; j < 3; j++)
      com(j) = bodies[i].I.pos[j];
    I << bodies[i].I.inertia[0], bodies[i].I.inertia[3], bodies[i].I.inertia[4],
        bodies[i].I.inertia[3], bodies[i].I.inertia[1], bodies[i].I.inertia[5],
        bodies[i].I.inertia[4], bodies[i].I.inertia[5], bodies[i].I.inertia[2];


    if ((bodies[i].joints.size() == 0) ||
        (bodies[i].joints.size() > 0 && fabs((bodies[i].joints.back()).ref) < 1e-3))
    {
      //a bit of a hack
      Eigen::Vector3d xaxis, yaxis;
      xaxis << bodies[i].xyaxes[0], bodies[i].xyaxes[1], bodies[i].xyaxes[2];
      xaxis.normalize();
      yaxis << bodies[i].xyaxes[3], bodies[i].xyaxes[4], bodies[i].xyaxes[5];
      yaxis.normalize();
      Eigen::Vector3d zaxis = xaxis.cross(yaxis);
      rot << xaxis(0), yaxis(0), zaxis(0),
          xaxis(1), yaxis(1), zaxis(1),
          xaxis(2), yaxis(2), zaxis(2);
    }
    else
    {
      rot << 1.0 , 0.0, 0.0,
          0.0, 1.0, 0.0,
          0.0, 0.0, 1.0;
    }

    for (int j = 0;j < 3;j++)
    {
      pos(j) = bodies[i].pos[j];
      if (joint.mJointType == RigidBodyDynamics::JointType1DoF)
        pos(j) -= joint_reference_(ref.size()-1)*(*joint.mJointAxes)(j+3);
    }

    for (unsigned int j = 0 ; j < id_lookup.size(); j++)
    {
      if (id_lookup[j][0] == bodies[i].p_id)
      {
        parent_id = id_lookup[j][1];
        break;
      }
    }

    Body body = Body(bodies[i].I.mass, com, I);

    total_mass_ += bodies[i].I.mass;

    SpatialTransform tf(rot.transpose(), pos);

    unsigned int body_id = model_.AddBody(parent_id, tf, joint, body);

    if (parent_id == 0)
      main_body_id_ = body_id;
    //		printf("Added %u to %u\n", body_id, parent_id);

    //		printf("Joint type: %u\n", joint.mJointType);

    //		printf("body: %u\pos:\n", body_id);
    //		std::cout << pos << std::endl;

    std::vector<unsigned int> id_pair;
    id_pair.push_back(bodies[i].id);
    id_pair.push_back(body_id);
    id_lookup.push_back(id_pair);
  }

  double qpos_init[nX];
  for (int i = 0; i < nQ; i++)
  {
    qpos_init[i] = ref[i]*M_PI/180.0;
    //		printf("%f\n",ref[i]);
  }
  qpos_init[nQ] = 1.0; //quaternion... cause they are assholes

  for (unsigned int i = 0; i < constraints.size(); i++)
  {
    Constraint new_constraint;
    for (unsigned int j = 0 ; j < id_lookup.size(); j++)
    {
      if (id_lookup[j][0] == constraints[i].bodyA_id)
        new_constraint.bodyA_id = id_lookup[j][1];
      if (id_lookup[j][0] == constraints[i].bodyB_id)
        new_constraint.bodyB_id = id_lookup[j][1];
    }
    Vector3d posALocal = Eigen::Map<Eigen::Vector3d>(constraints[i].posA);
    new_constraint.posA = posALocal;
    Vector3d baseCoord = CalcBodyToBaseCoordinates(model_, Eigen::Map<VectorNd>(qpos_init, nX), new_constraint.bodyA_id, posALocal, true);
    new_constraint.posB = CalcBaseToBodyCoordinates(model_, Eigen::Map<VectorNd>(qpos_init, nX), new_constraint.bodyB_id, baseCoord, true);
    //		std::cout << "Constraint: " << i << std::endl << "posA: " << new_constraint.posA << std::endl << "posB: " << new_constraint.posB << std::endl;
    constraints_.push_back(new_constraint);
  }

  rotor_inertia_ = MatrixNd::Zero(nQ,nQ);
  for(int i = 0; i < nQ; i++)
    rotor_inertia_(i,i) = rotor_inertia[i];

  for (unsigned int i = 0; i < bodies.size(); i++)
  {
    unsigned int body_id = 0;
    for (unsigned int j = 0 ; j < id_lookup.size(); j++)
      if (id_lookup[j][0] == bodies[i].id)
        body_id = id_lookup[j][1];
    Vector3d baseCoord = CalcBodyToBaseCoordinates(model_, Eigen::Map<VectorNd>(qpos_init, nX), body_id, VectorNd::Zero(3), true);
    //		printf("%u\t\t%f,%f,%f\n", body_id, baseCoord(0),baseCoord(1),baseCoord(2));
  }

  for (unsigned int i = 0; i < bodies.size(); i++)
  {
    unsigned int body_id = 0;
    for (unsigned int j = 0 ; j < id_lookup.size(); j++)
      if (id_lookup[j][0] == bodies[i].id)
        body_id = id_lookup[j][1];
    Matrix3d rot_mat = CalcBodyWorldOrientation(model_, Eigen::Map<VectorNd>(qpos_init, nX), body_id, false);
    //		std::cout << "body rot mat\n" << rot_mat << std::endl;
  }
  //	for (unsigned int i = 0; i < id_lookup.size(); i++)
  //		printf("Body: %u,%u\n", id_lookup[i][0], id_lookup[i][1]);


  for (unsigned int i = 0; i < motors.size(); i++)
  {
    int idx = 0;
    for (unsigned int j = 0; j < bodies.size(); j++)
    {
      for (unsigned int k = 0; k < bodies[j].joints.size(); k++)
      {
        if (motors[i].joint_name.compare(bodies[j].joints[k].name) == 0)
        {
          selector_matrix_(idx,i) = motors[i].gearN;
          motor_limits_(i,0) = motors[i].range[0];
          motor_limits_(i,1) = motors[i].range[1];
        }
        if (bodies[j].joints[k].type.compare("free") == 0)
          idx+=6;
        else
          idx++;
      }
    }
  }

  for (unsigned int i = 0; i < sites.size(); i++)
  {
    Site new_site;
    new_site.name = sites[i].name;
    for (unsigned int j = 0 ; j < id_lookup.size(); j++)
      if (id_lookup[j][0] == sites[i].body_id)
        new_site.body_id = id_lookup[j][1];
    new_site.pos = Eigen::Map<Eigen::Vector3d>(sites[i].pos);
    sites_.push_back(new_site);
  }

  model_.gravity(0) = 0.0;
  model_.gravity(1) = 0.0;
  model_.gravity(2) = -9.806;

  //	printf("%d\t%d\n", m.)

}

void DynamicModel::setState(double* qpos, double* qvel)
{
  state_.qpos = Eigen::Map<VectorNd>(qpos, nX);
  state_.qvel = Eigen::Map<VectorNd>(qvel, nQ);
  UpdateKinematics(model_, state_.qpos, state_.qvel, VectorNd::Zero(nQ));
}

void DynamicModel::setPos(double* qpos)
{
  state_.qpos = Eigen::Map<VectorNd>(qpos, nX);
  UpdateKinematicsCustom(model_, &state_.qpos, NULL, NULL);
}

void DynamicModel::SetQuaternion(RigidBodyDynamics::Math::Quaternion quat, double* qpos)
{
  state_.qpos = Eigen::Map<VectorNd>(qpos, nX);
  model_.SetQuaternion(main_body_id_, quat, state_.qpos);
  for (int i = 0; i < nX; i++)
    qpos[i] = state_.qpos(i);
}

void DynamicModel::GetMainBodyQuaternion(double* quat)
{
  RigidBodyDynamics::Math::Quaternion vQuat = model_.GetQuaternion(main_body_id_, state_.qpos);
  quat[0] = vQuat(3);
  quat[1] = vQuat(0);
  quat[2] = vQuat(1);
  quat[3] = vQuat(2);
}

void DynamicModel::GetMassMatrix(MatrixNd* mat)
{
  *mat = MatrixNd::Zero(nQ,nQ);
  CompositeRigidBodyAlgorithm(model_, state_.qpos, *mat, false);
  *mat += rotor_inertia_;
}

void DynamicModel::GetConstraintJacobian(MatrixNd* mat)
{
  *mat = MatrixNd::Zero(3*constraints_.size(), nQ);
  for (unsigned int i = 0; i < constraints_.size(); i++)
  {
    MatrixNd J1 = MatrixNd::Zero(3, nQ);
    CalcPointJacobian(model_, state_.qpos, constraints_[i].bodyA_id, constraints_[i].posA, J1, false);
    MatrixNd J2 = MatrixNd::Zero(3, nQ);
    CalcPointJacobian(model_, state_.qpos, constraints_[i].bodyB_id, constraints_[i].posB, J2, false);
    (*mat).block<3,nQ>(3*i,0) = J1 - J2;
  }
}

void DynamicModel::GetConstraintAccel(VectorNd* accel)
{
  for (unsigned int i = 0; i < constraints_.size(); i++)
  {
    VectorNd accelA = CalcPointAcceleration(model_, state_.qpos, state_.qvel, VectorNd::Zero(nQ), constraints_[i].bodyA_id, constraints_[i].posA);
    VectorNd accelB = CalcPointAcceleration(model_, state_.qpos, state_.qvel, VectorNd::Zero(nQ), constraints_[i].bodyB_id, constraints_[i].posB);
    (*accel).block<3,1>(3*i,0) = accelA - accelB;
  }
}

void DynamicModel::GetConstraintVel(VectorNd* vel)
{
  for (unsigned int i = 0; i < constraints_.size(); i++)
  {
    VectorNd velA = CalcPointVelocity(model_, state_.qpos, state_.qvel, constraints_[i].bodyA_id, constraints_[i].posA);
    VectorNd velB = CalcPointVelocity(model_, state_.qpos, state_.qvel, constraints_[i].bodyB_id, constraints_[i].posB);
    (*vel).block<3,1>(3*i,0) = velA - velB;
  }
}

void DynamicModel::GetDrivenVelocityAtConstraint(VectorNd* qdot, unsigned int constraint_id)
{
  //wrong wrong wrong...
  //vel from independent joints
  VectorNd velInd = CalcPointVelocity(model_, state_.qpos, state_.qvel, constraints_[constraint_id].bodyB_id, constraints_[constraint_id].posB);
  VectorNd velDep = CalcPointVelocity(model_, state_.qpos, state_.qvel, constraints_[constraint_id].bodyA_id, constraints_[constraint_id].posA);
  printf("%f,%f,%f\t%f,%f,%f\n", velDep(0), velDep(1), velDep(2), velInd(0), velInd(1), velInd(2));
  MatrixNd J1 = MatrixNd::Zero(3, nQ);
  CalcPointJacobian(model_, state_.qpos, constraints_[constraint_id].bodyA_id, constraints_[constraint_id].posA, J1, false);
  std::cout << J1 << std::endl;
  (*qdot) = J1.transpose()*(velInd-velDep);
}

void DynamicModel::GetBiasForce(VectorNd* qfrc_bias)
{
  NonlinearEffects(model_, state_.qpos, state_.qvel, *qfrc_bias);
}

void DynamicModel::GetPassiveForce(VectorNd* qfrc_passive)
{
  for (int i = 0; i < nQ; i++)
    (*qfrc_passive)(i) = -joint_daming_(i)*state_.qvel(i);// - jointStiffness(i)*(jointReference(i)-m_State.qpos(i));
}

void DynamicModel::GetSiteJacobian(MatrixNd* mat, int siteId)
{
  CalcPointJacobian(model_, state_.qpos, sites_[siteId].body_id, sites_[siteId].pos, *mat, false);
}

void DynamicModel::GetSite6DJacobian(MatrixNd* mat, int siteId)
{
  CalcPointJacobian6D(model_, state_.qpos, sites_[siteId].body_id, sites_[siteId].pos, *mat, false);
}

void DynamicModel::GetSiteAccel(VectorNd* accel, int siteId)
{
  *accel = CalcPointAcceleration(model_, state_.qpos, state_.qvel, VectorNd::Zero(nQ), sites_[siteId].body_id, sites_[siteId].pos);
}

void DynamicModel::GetSelectorMatrix(MatrixNd* mat)
{
  *mat = selector_matrix_;
}

void DynamicModel::GetMotorLimits(MatrixNd* mlb, MatrixNd* mub)
{
  for (int i = 0; i < nU; i++)
  {
    (*mlb)(i,0) = motor_limits_(i,0);
    (*mub)(i,0) = motor_limits_(i,1);
  }
}

void DynamicModel::GetTargetPoints(VectorNd* x, VectorNd* xd, int* targ_ids)
{
  for (unsigned int i = 0; i < XDD_TARGETS; i++)
  {
    (*x).block<3,1>(3*i,0) = CalcBodyToBaseCoordinates(model_, state_.qpos, sites_[targ_ids[i]].body_id, sites_[targ_ids[i]].pos, false);
    (*xd).block<3,1>(3*i,0) = CalcPointVelocity(model_, state_.qpos, state_.qvel, sites_[targ_ids[i]].body_id, sites_[targ_ids[i]].pos, false);
  }
}

void DynamicModel::GetConstraintPointsDependent(VectorNd* x, VectorNd* x_base)
{
  for (unsigned int i = 0; i < constraints_.size(); i++)
  {
    VectorNd worldA = CalcBodyToBaseCoordinates(model_, state_.qpos, constraints_[i].bodyB_id, constraints_[i].posB, false);
    Vector3d base_offset = Vector3d::Zero();
    VectorNd worldB_base = CalcBodyToBaseCoordinates(model_, state_.qpos, constraints_[i].bodyA_id, base_offset, false);
    (*x).block<3,1>(3*i,0) = worldA;
    (*x_base).block<3,1>(3*i,0) = worldB_base;
  }
}

void DynamicModel::GetConrodAngles(VectorNd* q_con)
{
  for (unsigned int i = 0; i < constraints_.size(); i++)
  {
    RigidBodyDynamics::Math::Vector3d zeroPos = RigidBodyDynamics::Math::Vector3d::Zero();

    VectorNd hip_base = CalcBodyToBaseCoordinates(model_, state_.qpos, constraints_[i].bodyA_id, zeroPos, false);
    VectorNd conrod = CalcBodyToBaseCoordinates(model_, state_.qpos, constraints_[i].bodyA_id, constraints_[i].posA, false);
    VectorNd heel_spring = CalcBodyToBaseCoordinates(model_, state_.qpos, constraints_[i].bodyB_id, constraints_[i].posB, false);
    VectorNd heel_spring_base = CalcBodyToBaseCoordinates(model_, state_.qpos, constraints_[i].bodyB_id, zeroPos, false);

    //		printf("Constraint %d\n", i);
    //		std::cout << hip_base.transpose() << std::endl;
    //		std::cout << conrod.transpose() << std::endl;
    //		std::cout << heel_spring.transpose() << std::endl;

    VectorNd vecA = conrod - hip_base;
    VectorNd vecB = heel_spring - hip_base;
    VectorNd vecC = heel_spring_base - hip_base;

    double num = 0.0;
    for (int j = 0; j < 3; j++)
      num += vecA(j)*vecB(j);
    double magA = sqrt(pow(vecA(0),2.0) + pow(vecA(1),2.0) + pow(vecA(2),2.0));
    double magB = sqrt(pow(vecB(0),2.0) + pow(vecB(1),2.0) + pow(vecB(2),2.0));
    (*q_con)(i) = acos(num/(magA*magB));

    Vector3d rot_axis_1, rot_axis_2;
    rot_axis_1(0) = vecC(1)*vecB(2) - vecC(2)*vecB(1);
    rot_axis_1(1) = vecC(2)*vecB(0) - vecC(0)*vecB(2);
    rot_axis_1(2) = vecC(0)*vecB(1) - vecC(1)*vecB(0);

    rot_axis_2(0) = vecA(1)*vecB(2) - vecA(2)*vecB(1);
    rot_axis_2(1) = vecA(2)*vecB(0) - vecA(0)*vecB(2);
    rot_axis_2(2) = vecA(0)*vecB(1) - vecA(1)*vecB(0);

    if (rot_axis_1(0)*rot_axis_2(0) + rot_axis_1(1)*rot_axis_2(1) + rot_axis_1(2)*rot_axis_2(2) > 0.0)
      (*q_con)(i) *= -1.0;
    //
    //		std::cout << "vecC -> vecB\n" << rot_axis_1.transpose() << std::endl;
    //		std::cout << "vecA -> vecB\n" << rot_axis_2.transpose() << std::endl;
    //
    //		for (int j = 0; j < 3; j++)
    //			printf("%f,",vecA(j));
    //		for (int j = 0; j < 3; j++)
    //			printf("%f,",vecB(j));
    //		printf("%f",(*q_con)(i));
    //		if (i == 0)
    //			printf(",");
    //		else
    //			printf("\n");
  }
}
