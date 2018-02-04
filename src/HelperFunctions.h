#ifndef HELPER_FUNCTIONS_H_
#define HELPER_FUNCTIONS_H_

#include <Eigen/Dense>
#include <sys/time.h>


template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime>
pseudoinverse(const MatT &mat, typename MatT::Scalar tolerance = typename MatT::Scalar{1e-4}) // choose appropriately
{
    typedef typename MatT::Scalar Scalar;
    auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
    const auto &singularValues = svd.singularValues();
    Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());

    singularValuesInv.setZero();
    for (unsigned int i = 0; i < singularValues.size(); ++i) {
        if (singularValues(i) > tolerance)
        {
            singularValuesInv(i, i) = Scalar{1} / singularValues(i);
        }
        else
        {
            singularValuesInv(i, i) = Scalar{0};
        }
    }
    return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

static void eulerToQuaternion(const double euler[], double quat[])
{
	double cy = cos(euler[2] * 0.5);
	double sy = sin(euler[2] * 0.5);
	double cr = cos(euler[0] * 0.5);
	double sr = sin(euler[0] * 0.5);
	double cp = cos(euler[1] * 0.5);
	double sp = sin(euler[1] * 0.5);

	quat[0] = cy * cr * cp + sy * sr * sp;
	quat[1] = cy * sr * cp - sy * cr * sp;
	quat[2] = cy * cr * sp + sy * sr * cp;
	quat[3] = sy * cr * cp - cy * sr * sp;
}

static void quaternionToEuler(const double quat[], double euler[])
{
	// roll (x-axis rotation)
	double sinr = +2.0 * (quat[0] * quat[1] + quat[2] * quat[3]);
	double cosr = +1.0 - 2.0 * (quat[1] * quat[1] + quat[2] * quat[2]);
	euler[0] = atan2(sinr, cosr);

	// pitch (y-axis rotation)
	double sinp = +2.0 * (quat[0] * quat[2] - quat[3] * quat[1]);
	if (fabs(sinp) >= 1)
		euler[1] = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
	else
		euler[1] = asin(sinp);

	// yaw (z-axis rotation)
	double siny = +2.0 * (quat[0] * quat[3] + quat[1] * quat[2]);
	double cosy = +1.0 - 2.0 * (quat[2] * quat[2] + quat[3] * quat[3]);
	euler[2] = atan2(siny, cosy);
}

static timespec diff(timespec start, timespec end)
{
        timespec temp;
        if ((end.tv_nsec-start.tv_nsec)<0) {
                temp.tv_sec = end.tv_sec-start.tv_sec-1;
                temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
        } else {
                temp.tv_sec = end.tv_sec-start.tv_sec;
                temp.tv_nsec = end.tv_nsec-start.tv_nsec;
        }
        return temp;
}
#endif
