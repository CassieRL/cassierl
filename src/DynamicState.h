/*
 * DynamicState.h
 *
 *  Created on: Jan 11, 2018
 *      Author: tapgar
 */

#ifndef DYNAMICSTATE_H_
#define DYNAMICSTATE_H_

#include "DynamicModel.h"
#include <Eigen/Dense>

typedef struct {
	Eigen::MatrixXd M; //Mass matrix
	Eigen::VectorXd bias; //coriolis, grav, spring
	Eigen::MatrixXd Bt; //B transpose
	Eigen::MatrixXd Jc; //contact jacobian
	Eigen::MatrixXd Jeq; //constrian jacobian
	Eigen::VectorXd JeqdotQdot; //accel difference at constraint
} DynamicMatrices;

class DynamicState {
public:
	DynamicState();
	virtual ~DynamicState();

	void Init(int* conIds);

	void UpdateDynamicState(DynamicModel* dyn);

	DynamicMatrices config_stiff_;
	DynamicMatrices config_spring_;

	void GetStiffConfig(Eigen::MatrixXd* M, Eigen::VectorXd* bias, Eigen::MatrixXd* Bt,
			Eigen::MatrixXd* Jc, Eigen::MatrixXd* Jeq, Eigen::VectorXd* JeqdotQdot)
	{
		*M = config_stiff_.M;
		*bias = config_stiff_.bias;
		*Bt = config_stiff_.Bt;
		*Jc = config_stiff_.Jc;
		*Jeq = config_stiff_.Jeq;
		*JeqdotQdot = config_stiff_.JeqdotQdot;
	}

	void GetSpringConfig(Eigen::MatrixXd* M, Eigen::VectorXd* bias, Eigen::MatrixXd* Bt,
			Eigen::MatrixXd* Jc, Eigen::MatrixXd* Jeq, Eigen::VectorXd* JeqdotQdot)
	{
		*M = config_spring_.M;
		*bias = config_spring_.bias;
		*Bt = config_spring_.Bt;
		*Jc = config_spring_.Jc;
		*Jeq = config_spring_.Jeq;
		*JeqdotQdot = config_spring_.JeqdotQdot;
	}

private:
	int contact_site_ids_[nCON];
};

#endif /* DYNAMICSTATE_H_ */
