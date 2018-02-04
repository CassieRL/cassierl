/*
 * OSC_RBDL.cpp
 *
 *  Created on: Nov 8, 2017
 *      Author: tapgar
 */

#include "OSC_RBDL.h"
#include <sys/time.h>

using namespace std;
using namespace Eigen;


OSC_RBDL::OSC_RBDL(int* targIds) {
	m_nAssignedIndices = 0;

	for (int i = 0; i < XDD_TARGETS; i++)
		targetSiteIds[i] = targIds[i];

	qp = new qpOASES::SQProblem((nQstiff+nU+nCON*(2*(DOF-1)+1)), nQstiff+nCON*4*(DOF-1));

	qpOptions.setToMPC( );
	qpOptions.printLevel = qpOASES::PL_NONE;
	qp->setOptions( qpOptions );

}

void OSC_RBDL::InitMatrices(DynamicModel* dyn) {

	//initialize weight matrix
	W = Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, XDD_TARGETS*DOF + QDD_TARGETS>::Zero();
	for (int i = 0; i < DOF; i++)
		W(i,i) = m_dWeight_COM;
	for (int i = DOF; i < XDD_TARGETS*DOF; i++)
		W(i,i) = m_dWeight_Stance;
	for (int i = XDD_TARGETS*DOF; i < XDD_TARGETS*DOF + QDD_TARGETS; i++)
		W(i,i) = m_dWeight_Rest;

	//initialize friction cone matrix
	V = Matrix<double, nCON*DOF, nCON*(2*(DOF-1)+1)>::Zero();
	for (int i = 0; i < nCON; i++)
	{
		for (int j = 0; j < DOF-1; j++)
		{
			V(i*DOF+j, i*(2*(DOF-1)+1) + j*2) = -1.0;
			V(i*DOF+j, i*(2*(DOF-1)+1) + j*2 + 1) = 1.0;
		}
		V(i*DOF+DOF-1, i*(2*(DOF-1)+1) + 2*(DOF-1)) = 1.0; //normal force
	}

	//initialize inequality constraint matrix
	C = Matrix<double, nCON*4*(DOF-1), nQstiff+nU+nCON*(2*(DOF-1)+1)>::Zero();
	for (int i = 0; i < nCON; i++)
	{
		for (int j = 0; j < DOF-1; j++)
		{
			int l = 0;
			for (int k = 0; k < 4; k++)
			{
				if (k % 2)
				{
					C(i*4*(DOF-1) + 4*j + k, nQstiff+nU+i*(2*(DOF-1)+1)+2*j+l) = 1.0;
					l++;
				}
				else
					C(i*4*(DOF-1) + 4*j + k, nQstiff+nU+i*(2*(DOF-1)+1)+2*j+l) = -1.0;
				C(i*4*(DOF-1) + 4*j + k, nQstiff+nU+(i+1)*(2*(DOF-1)+1)-1) = -mu;
			}
		}
	}

	//set up lb and ub
	lb = -Matrix<double, nQstiff+nU+nCON*(2*(DOF-1)+1), 1>::Ones()*numeric_limits<double>::max();
	ub = Matrix<double, nQstiff+nU+nCON*(2*(DOF-1)+1), 1>::Ones()*numeric_limits<double>::max();
	MatrixXd mlb = MatrixXd::Zero(nU,1);
	MatrixXd mub = MatrixXd::Zero(nU,1);
	GetMotorLimits(dyn, &mlb, &mub);
	lb.block<nU,1>(nQstiff,0) = mlb;
	ub.block<nU,1>(nQstiff,0) = mub;

	Matrix<double, nCON*(2*(DOF-1) + 1), 1> flb = -Matrix<double, nCON*(2*(DOF-1) + 1), 1>::Ones()*numeric_limits<double>::max();
	for (int i = 0; i < nCON*(2*(DOF-1) + 1); i++)
		flb(i,0) = 0.0; //normal force always positive
	lb.block<nCON*(2*(DOF-1) + 1),1>(nU+nQstiff,0) = flb;


	ciub = Matrix<double, nCON*4*(DOF-1), 1>::Zero();
	cilb = -1.0*Matrix<double, nCON*4*(DOF-1), 1>::Ones()*numeric_limits<double>::max();


	//Initialize everything to zeros
	M = MatrixXd::Zero(nQstiff,nQstiff);
	bias = VectorXd::Zero(nQstiff);
	Bt = MatrixXd::Zero(nQstiff, nU);
	Jc = MatrixXd::Zero(nCON*DOF, nQstiff);
	A = MatrixXd::Zero(XDD_TARGETS*DOF + QDD_TARGETS, nQstiff);
	AdotQdot = VectorXd::Zero(XDD_TARGETS*DOF + QDD_TARGETS);
	Jeq = MatrixXd::Zero(nEQ, nQstiff);
	JeqdotQdot = VectorXd::Zero(nEQ);
	x = Matrix<double, nQstiff+nU+nCON*(2*(DOF-1)+1), 1>::Zero();

	H = Matrix<double, nQstiff+nU+nCON*(2*(DOF-1)+1), nQstiff+nU+nCON*(2*(DOF-1)+1)>::Zero();
	gt = Matrix<double, 1, nQstiff+nU+nCON*(2*(DOF-1)+1)>::Zero();
	CE = Matrix<double, nQstiff, nQstiff+nU+nCON*(2*(DOF-1)+1)>::Zero();
	ce = Matrix<double, nQstiff, 1>::Zero();
}

void OSC_RBDL::GetMotorLimits(DynamicModel* dyn, MatrixXd* mlb, MatrixXd* mub)
{
	dyn->GetMotorLimits(mlb, mub);
}

void OSC_RBDL::RunPTSC(DynamicModel* dyn, DynamicState* dyn_state, Matrix<double, DOF*XDD_TARGETS+QDD_TARGETS, 1> xdd, Matrix<bool, DOF*XDD_TARGETS+QDD_TARGETS, 1> bActive, bool* bContact, Matrix<double, nU, 1>* u)
{

	timespec ts, tf;
	clock_gettime(CLOCK_REALTIME, &ts);

	dyn_state->GetStiffConfig(&M, &bias, &Bt, &Jc, &Jeq, &JeqdotQdot);

	//target site jacobians
	for (int i = 0; i < XDD_TARGETS; i++)
	{
		MatrixXd jc = MatrixXd::Zero(DOF, nQ);
		VectorXd accel = VectorXd::Zero(DOF);
		dyn->GetSiteJacobian(&jc, targetSiteIds[i]);
		dyn->GetSiteAccel(&accel, targetSiteIds[i]);
		int j_idx = 0;
		for (int j = 0; j < nQ; j++)
		{
			if (dyn->IsSpringJoint(j))
				continue;
			A.block<DOF,1>(i*DOF,j_idx) = jc.block<DOF,1>(0,j);
			j_idx++;
		}
		AdotQdot.block<DOF,1>(i*DOF,0) = accel;
	}

	for (int i = 0; i < QDD_TARGETS; i++)
	{
		int idx = XDD_TARGETS*DOF + i;
		A(idx, qdd_targ_idx[i]) = 1.0;
	}

	clock_gettime(CLOCK_REALTIME, &tf);
	int dynamics_time_us = int(double(diff(ts,tf).tv_nsec)/1e3);
	ts = tf;

	Matrix<double, XDD_TARGETS*DOF + QDD_TARGETS, XDD_TARGETS*DOF + QDD_TARGETS> tempW = W;

	//adjust limits based on contact forces
	for (int i = 0; i < nCON; i++)
	{
		if (bContact[i])
		{
			ub(nQstiff+nU+i*(2*(DOF-1)+1)+2*(DOF-1),0) = numeric_limits<double>::max();
			for (int j = 0; j < DOF; j++)
				tempW(DOF+(i/2)*DOF+j,DOF+(i/2)*DOF+j) = m_dWeight_Stance;
		}
		else
		{
			ub(nQstiff+nU+i*(2*(DOF-1)+1)+2*(DOF-1),0) = 0.0;
			for (int j = 0; j < DOF; j++)
				tempW(DOF+(i/2)*DOF+j,DOF+(i/2)*DOF+j) = m_dWeight_Swing;
		}
	}

	Matrix<double, nQstiff, nQstiff> I = Matrix<double, nQstiff, nQstiff>::Identity();
	Matrix<double, nQstiff, nQstiff> Hinv = M.inverse();
	Matrix<double, nEQ, nEQ> JHinvJ = pseudoinverse(Jeq*Hinv*Jeq.transpose(), 1e-3);//.completeOrthogonalDecomposition().pseudoInverse();

	Nc = I - Jeq.transpose()*JHinvJ*Jeq*Hinv;
	gamma = Jeq.transpose()*JHinvJ*JeqdotQdot;

	for (int i = 0; i < DOF*XDD_TARGETS+QDD_TARGETS; i++)
		if (!bActive(i,0))
			tempW(i,i) = 0.0;

	CE.block<nQstiff,nQstiff>(0,0) = M;
	CE.block<nQstiff,nU>(0,nQstiff) = -Nc*Bt;
	CE.block<nQstiff,nCON*(2*(DOF-1)+1)>(0,nQstiff+nU) = -Nc*(Jc.transpose())*V;

	ce = -Nc*bias - gamma;

	H.block<nQstiff,nQstiff>(0,0) = 2.0*A.transpose()*tempW*A;

	int j = 0;
	for (int i = nQstiff+nU; i < H.cols(); i++)
	{
		if (j < 2*(DOF-1))
		{
			H(i,i) = m_dWeight_Fx;
		}
		else
		{
			H(i,i) = m_dWeight_Fz;
			j = -1;
		}
		j++;
	}

	gt.block<1,nQstiff>(0,0) = 2.0*AdotQdot.transpose()*tempW*A -2.0*xdd.transpose()*tempW*A;

	double xtemp[(nQstiff+nU+nCON*(2*(DOF-1)+1))];

	clock_gettime(CLOCK_REALTIME, &tf);
	int mat_time_us = int(double(diff(ts,tf).tv_nsec)/1e3);
	ts = tf;

	bool status = SolveQP(H.data(), gt.data(), CE.data(), ce.data(),
			C.data(), cilb.data(), ciub.data(), lb.data(), ub.data(), xtemp);

	clock_gettime(CLOCK_REALTIME, &tf);
	int qp_time_us = int(double(diff(ts,tf).tv_nsec)/1e3);

//	std::cout<< dynamics_time_us << "," << mat_time_us << "," << qp_time_us << ",";

//	(*u) = x.block<nU,1>(nQstiff,0);
	for (int i = 0; i < nU; i++)
		(*u)(i,0) = xtemp[nQstiff+i];

//	LogMatrices(xtemp);

//	if (!status)
//		printf("QP SOLVE ERROR!!!\n");

//	std::cout << "CE:" << std::endl << CE << std::endl;
//	std::cout << "ce:" << std::endl << ce << std::endl;
//	std::cout << "C:" << std::endl << C << std::endl;
//	std::cout << "cilb:" << std::endl << cilb << std::endl;
//	std::cout << "ciub:" << std::endl << ciub << std::endl;
//	std::cout << "lb:" << std::endl << lb << std::endl;
//	std::cout << "ub:" << std::endl << ub << std::endl;
//	std::cout << "V:" << std::endl << V << std::endl;
}

bool OSC_RBDL::SolveQP(double* H_, double* g_, double* CE_, double* ce_,
			double* C_, double* cilb_, double* ciub_,
			double* lb_, double* ub_, double* x_res)
{
	static bool bFirstCall = true;

	qpOASES::int_t nWSR_first = 1000;
	qpOASES::int_t nWSR_hot = 100;
	qpOASES::real_t dTimeLimit_s = 5e-4;
	qpOASES::returnValue eRet = qpOASES::SUCCESSFUL_RETURN;

	double CE_C[(nQstiff+nCON*4*(DOF-1))*(nQstiff+nU+nCON*(2*(DOF-1)+1))];
	double lbA[nQstiff+nCON*4*(DOF-1)];
	double ubA[nQstiff+nCON*4*(DOF-1)];

	for (int i = 0; i < nQstiff*(nQstiff+nU+nCON*(2*(DOF-1)+1)); i++)
		CE_C[i] = CE_[i];
	for (int i = 0; i < nQstiff; i++)
		lbA[i] = ubA[i] = ce_[i];
	for (int i = nQstiff*(nQstiff+nU+nCON*(2*(DOF-1)+1)); i < (nQstiff+nCON*4*(DOF-1))*(nQstiff+nU+nCON*(2*(DOF-1)+1)); i++)
		CE_C[i] = C_[i - nQstiff*(nQstiff+nU+nCON*(2*(DOF-1)+1))];
	for (int i = nQstiff; i < (nQstiff+nCON*4*(DOF-1)); i++)
	{
		lbA[i] = cilb_[i-nQstiff];
		ubA[i] = ciub_[i-nQstiff];
	}

//	for (int i = 0; i < (nQstiff+nCON*4*(DOF-1)); i++)
//	{
//		for (int j = 0; j < (nQstiff+nU+nCON*(2*(DOF-1)+1)); j++)
//		{
//			printf("%f\t", CE_C[(nQstiff+nU+nCON*(2*(DOF-1)+1))*i + j]);
//		}
//		printf("\n");
//	}


	if (bFirstCall)
		eRet = qp->init(H_, g_, CE_C, lb_, ub_, lbA, ubA, nWSR_first, 0);
	else
		eRet = qp->hotstart(H_, g_, CE_C, lb_, ub_, lbA, ubA, nWSR_hot, &dTimeLimit_s);

	qp->getPrimalSolution(x_res);

//	printf("qp ret: %d\n", (int)(eRet));
	bFirstCall = false;

	if (eRet == qpOASES::SUCCESSFUL_RETURN)
	{
		return true;
	}

	return false;
}

void OSC_RBDL::LogMatrices(double* x)
{
	for (int i = 0; i < nQstiff+nU+nCON*(2*(DOF-1)+1) - 1; i++)
		xFile << x[i] << ",";
	xFile << x[nQstiff+nU+nCON*(2*(DOF-1)+1) - 1] << std::endl;

	for (int i = 0; i < H.rows(); i++)
		for (int j = 0; j < H.cols(); j++)
			hFile << H(i,j) << ",";
	hFile << std::endl;

	for (int i = 0; i < gt.cols(); i++)
		gtFile << gt(0,i) << ",";
	gtFile << std::endl;

	for (int i = 0; i < CE.rows(); i++)
		for (int j = 0; j < CE.cols(); j++)
			CEFile << CE(i,j) << ",";
	CEFile << std::endl;

	for (int i = 0; i < C.rows(); i++)
		for (int j = 0; j < C.cols(); j++)
			CFile << C(i,j) << ",";
	CFile << std::endl;

	for (int i = 0; i < Jeq.rows(); i++)
		for (int j = 0; j < Jeq.cols(); j++)
			JeqFile << Jeq(i,j) << ",";
	JeqFile << std::endl;

	for (int i = 0; i < M.rows(); i++)
		for (int j = 0; j < M.cols(); j++)
			mFile << M(i,j) << ",";
	mFile << std::endl;

}
