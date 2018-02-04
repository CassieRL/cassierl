/*
 * DynamicModel.h
 *
 *  Created on: Dec 20, 2017
 *      Author: tapgar
 */

#ifndef DYNAMICMODEL_H_
#define DYNAMICMODEL_H_

#include <rbdl/rbdl.h>
#include <Eigen/Dense>
#include "RobotInterface.h"

struct RobotState {
	RigidBodyDynamics::Math::VectorNd qpos;
	RigidBodyDynamics::Math::VectorNd qvel;
};

struct Site {
	RigidBodyDynamics::Math::Vector3d pos;
	unsigned int body_id;
	std::string name;
};

struct Constraint {
	RigidBodyDynamics::Math::Vector3d posA;
	unsigned int bodyA_id;

	RigidBodyDynamics::Math::Vector3d posB;
	unsigned int bodyB_id;
};

class DynamicModel {
public:
	DynamicModel();
	virtual ~DynamicModel();

	void LoadModel(std::string xml_file);

	void setState(double* qpos, double* qvel);
	void setPos(double* qpos);

	void SetQuaternion(RigidBodyDynamics::Math::Quaternion quat, double* qpos);

	void GetMainBodyQuaternion(double* quat);

	void GetMassMatrix(RigidBodyDynamics::Math::MatrixNd* mat);
	void GetConstraintJacobian(RigidBodyDynamics::Math::MatrixNd* mat);
	void GetConstraintAccel(RigidBodyDynamics::Math::VectorNd* accel);
	void GetConstraintVel(RigidBodyDynamics::Math::VectorNd* vel);
	void GetDrivenVelocityAtConstraint(RigidBodyDynamics::Math::VectorNd* qdot, unsigned int constraint_id);

	void GetBiasForce(RigidBodyDynamics::Math::VectorNd* qfrc_bias);
	void GetPassiveForce(RigidBodyDynamics::Math::VectorNd* qfrc_passive);

	void GetSiteJacobian(RigidBodyDynamics::Math::MatrixNd* mat, int site_id);
	void GetSite6DJacobian(RigidBodyDynamics::Math::MatrixNd* mat, int site_id);
	void GetSiteAccel(RigidBodyDynamics::Math::VectorNd* accel, int site_id);

	void GetSelectorMatrix(RigidBodyDynamics::Math::MatrixNd* mat);

	void GetMotorLimits(RigidBodyDynamics::Math::MatrixNd* mlb, RigidBodyDynamics::Math::MatrixNd* mub);

	void GetTargetPoints(RigidBodyDynamics::Math::VectorNd* x, RigidBodyDynamics::Math::VectorNd* xd, int* targ_ids);

	void GetConstraintPointsDependent(RigidBodyDynamics::Math::VectorNd* x, RigidBodyDynamics::Math::VectorNd* constraint_base_pose);

	void GetConrodAngles(RigidBodyDynamics::Math::VectorNd* q_con);

	double GetMass() { return total_mass_; }

	bool IsSpringJoint(int idx) { return joint_stiffness_(idx,0) > 0.0; }

private:

	RigidBodyDynamics::Model model_;
	RobotState state_;

	RigidBodyDynamics::Math::MatrixNd rotor_inertia_;
	RigidBodyDynamics::Math::MatrixNd selector_matrix_;
	RigidBodyDynamics::Math::VectorNd joint_daming_;
	RigidBodyDynamics::Math::VectorNd joint_stiffness_;
	RigidBodyDynamics::Math::VectorNd joint_reference_;

	RigidBodyDynamics::Math::MatrixNd motor_limits_;

	std::vector<Site> sites_;
	std::vector<Constraint> constraints_;

	unsigned int main_body_id_;

	double total_mass_;

};

#endif /* DYNAMICMODEL_H_ */
