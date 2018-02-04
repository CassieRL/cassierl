/*
 * OSC_RBDL.h
 *
 *  Created on: Nov 8, 2017
 *      Author: tapgar
 */

#ifndef OSC_RBDL_H_
#define OSC_RBDL_H_

#include <qpOASES.hpp>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <Eigen/QR>
#include "HelperFunctions.h"
#include "RobotInterface.h"
#include "DynamicModel.h"
#include "DynamicState.h"
#include <fstream>


class OSC_RBDL {

public:
	OSC_RBDL(int* targIds);

	void RunPTSC(DynamicModel* dyn, DynamicState* dyn_state, Eigen::Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd, Eigen::Matrix<bool, DOF*XDD_TARGETS+QDD_TARGETS, 1> bActive, bool* bContact, Eigen::Matrix<double, nU, 1>* u);

	int AddQDDIdx(int idx) {

		if (m_nAssignedIndices == QDD_TARGETS || idx < 0 || idx >= nQstiff)
			return 1;
		qdd_targ_idx[m_nAssignedIndices] = idx;
		m_nAssignedIndices++;
		return 0;
	}

	void InitMatrices(DynamicModel* dyn);


private:

	qpOASES::SQProblem* qp; //for QPs where H or A may change
	qpOASES::Options qpOptions;

	void GetMotorLimits(DynamicModel* dyn, Eigen::MatrixXd* mlb, Eigen::MatrixXd* mub);

	bool SolveQP(double* H_, double* g_, double* CE_, double* ce_,
			double* C_, double* cilb_, double* ciub_,
			double* lb_, double* ub_, double* x_res);

	Eigen::MatrixXd M; //Mass matrix
	Eigen::VectorXd bias; //coriolis, grav, spring
	Eigen::MatrixXd Bt; //B transpose
	Eigen::MatrixXd Jc; //contact jacobian
	Eigen::MatrixXd A; //jacobian relating target
	Eigen::VectorXd AdotQdot; //time deriv above
	Eigen::MatrixXd Jeq; //constrian jacobian
	Eigen::VectorXd JeqdotQdot;

	Eigen::Matrix<double, nQstiff, nQstiff> Nc; //constraint projector
	Eigen::Matrix<double, nQstiff, 1> gamma; //

	Eigen::Matrix<double, nCON*DOF, nCON*(2*(DOF-1)+1)> V; //friction cone

	Eigen::Matrix<double, nQstiff+nU+nCON*(2*(DOF-1)+1), 1> x;//open vars

	Eigen::Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, XDD_TARGETS*DOF + QDD_TARGETS> W; //QP weighting matrix

	//QP matrices
	// z = 0.5*x'Hx + g'x;
	Eigen::Matrix<double, nQstiff+nU+nCON*(2*(DOF-1)+1), nQstiff+nU+nCON*(2*(DOF-1)+1), Eigen::RowMajor> H;//hessian
	Eigen::Matrix<double, 1, nQstiff+nU+nCON*(2*(DOF-1)+1), Eigen::RowMajor> gt; //jacobian
	//Equality constraints
	//Ax = b
	Eigen::Matrix<double, nQstiff, nQstiff+nU+nCON*(2*(DOF-1)+1), Eigen::RowMajor> CE;
	Eigen::Matrix<double, nQstiff, 1> ce;
	//Inequality constraints
	//d <= Cx <= f
	Eigen::Matrix<double, nCON*4*(DOF-1), nQstiff+nU+nCON*(2*(DOF-1)+1), Eigen::RowMajor> C;
	Eigen::Matrix<double, nCON*4*(DOF-1), 1> cilb;
	Eigen::Matrix<double, nCON*4*(DOF-1), 1> ciub;
	//Upper/Lower bounds
	Eigen::Matrix<double, nQstiff+nU+nCON*(2*(DOF-1)+1), 1> ub;
	Eigen::Matrix<double, nQstiff+nU+nCON*(2*(DOF-1)+1), 1> lb;

	int targetSiteIds[XDD_TARGETS];

	int qdd_targ_idx[QDD_TARGETS];
	int m_nAssignedIndices;

	static constexpr double m_dWeight_Stance = 1e1;
	static constexpr double m_dWeight_Swing = 1e0;//0.2;
	static constexpr double m_dWeight_COM = 5.0;
	static constexpr double m_dWeight_Rest = 1e-1;//0.000002;
	static constexpr double m_dWeight_Tau = 1e-6;//0.000002;
	static constexpr double m_dWeight_Fx = 1e-4;
	static constexpr double m_dWeight_Fz = 1e-4;

	void LogMatrices(double* x);

	std::ofstream hFile;
	std::ofstream xFile;
	std::ofstream gtFile;
	std::ofstream CFile;
	std::ofstream CEFile;
	std::ofstream mFile;
	std::ofstream JeqFile;
};


#endif /* OSC_RBDL_H_ */
