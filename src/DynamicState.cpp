/*
 * DynamicState.cpp
 *
 *  Created on: Jan 11, 2018
 *      Author: tapgar
 */

#include "DynamicState.h"

using namespace Eigen;
using namespace std;

DynamicState::DynamicState() {

}


DynamicState::~DynamicState() {
	// TODO Auto-generated destructor stub
}

void DynamicState::Init(int* conIds) {
	// TODO Auto-generated constructor stub


	config_stiff_.M = MatrixXd::Zero(nQstiff,nQstiff);
	config_stiff_.bias = VectorXd::Zero(nQstiff);
	config_stiff_.Bt = MatrixXd::Zero(nQstiff, nU);
	config_stiff_.Jc = MatrixXd::Zero(nCON*DOF, nQstiff);
	config_stiff_.Jeq = MatrixXd::Zero(nEQ, nQstiff);
	config_stiff_.JeqdotQdot = VectorXd::Zero(nEQ);

	config_spring_.M = MatrixXd::Zero(nQ,nQ);
	config_spring_.bias = VectorXd::Zero(nQ);
	config_spring_.Bt = MatrixXd::Zero(nQ, nU);
	config_spring_.Jc = MatrixXd::Zero(nCON*DOF, nQ);
	config_spring_.Jeq = MatrixXd::Zero(nEQ, nQ);
	config_spring_.JeqdotQdot = VectorXd::Zero(nEQ);

	for (int i = 0; i < nCON; i++)
		contact_site_ids_[i] = conIds[i];


}
void DynamicState::UpdateDynamicState(DynamicModel* dyn)
{
	dyn->GetMassMatrix(&config_spring_.M);

	dyn->GetBiasForce(&config_spring_.bias);
	VectorXd passive = VectorXd::Zero(nQ);
	dyn->GetPassiveForce(&passive);
	config_spring_.bias -= passive;

	dyn->GetSelectorMatrix(&config_spring_.Bt);

	//Contact jacobians
	for (int i = 0; i < nCON; i++)
	{
		MatrixXd jc = MatrixXd::Zero(DOF, nQ);
		dyn->GetSiteJacobian(&jc, contact_site_ids_[i]);
		config_spring_.Jc.block<DOF,nQ>(i*DOF,0) = jc;
	}

	//Constraint jacobians
	dyn->GetConstraintJacobian(&config_spring_.Jeq);
	dyn->GetConstraintAccel(&config_spring_.JeqdotQdot);

	int idx_i = 0;
	for (int i = 0; i < nQ; i++)
	{
		if (dyn->IsSpringJoint(i))
			continue;

		int idx_j = 0;
		for (int j = 0; j < nQ; j++)
		{
			if (dyn->IsSpringJoint(j))
				continue;
			config_stiff_.M(idx_i,idx_j) = config_spring_.M(i,j);
			idx_j++;
		}

		config_stiff_.bias(idx_i) = config_spring_.bias(i);
		config_stiff_.Bt.row(idx_i) = config_spring_.Bt.row(i);
		config_stiff_.Jc.col(idx_i) = config_spring_.Jc.col(i);
		config_stiff_.Jeq.col(idx_i) = config_spring_.Jeq.col(i);

		idx_i++;
	}
	config_stiff_.JeqdotQdot = config_spring_.JeqdotQdot;
}
