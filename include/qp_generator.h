#ifndef _TRAJECTORY_GENERATOR_H_
#define _TRAJECTORY_GENERATOR_H_
/*#include <eigen3/Eigen/Dense>*/
#include <Eigen/Eigen>
#include <vector>


class TrajectoryGenerator {

private:
		std::vector<double> qp_cost;
        Eigen::MatrixXd _A; // Mapping matrix
        Eigen::MatrixXd _Q; // Hessian matrix
        Eigen::MatrixXd _C; // Selection matrix
        Eigen::MatrixXd _L; // A.inv() * C.transpose()

        Eigen::MatrixXd _R;
        Eigen::MatrixXd _Rff;
        Eigen::MatrixXd _Rpp;
        Eigen::MatrixXd _Rpf;
        Eigen::MatrixXd _Rfp;

        Eigen::VectorXd _Pxi;
        Eigen::VectorXd _Pyi;
        Eigen::VectorXd _Pzi;

        Eigen::VectorXd _Dx;
        Eigen::VectorXd _Dy;
        Eigen::VectorXd _Dz;
public:
        Eigen::MatrixXd _Path;
        Eigen::VectorXd _Time;

        TrajectoryGenerator();

        ~TrajectoryGenerator();

        Eigen::MatrixXd PolyQPGeneration(
            const Eigen::MatrixXd &Path,
            const Eigen::Vector3d &Vel,
            const Eigen::Vector3d &Acc,
            const Eigen::VectorXd &Time,
            const int &type);
        
        Eigen::MatrixXd PloyCoeffGeneration(
            const Eigen::MatrixXd &PathCorridor,
            const Eigen::MatrixXd &PathConnect,
            const Eigen::VectorXd &Radius,
            const Eigen::VectorXd &Path_Radius,
            const Eigen::VectorXd &Time,
            const Eigen::MatrixXd &vel,
            const Eigen::MatrixXd &acc,
            const double maxVel,
            const double maxAcc );

        std::vector<double> getCost();

        void StackOptiDep(); // Stack the optimization's dependency, the intermediate matrix and initial derivatives

        std::pair< Eigen::MatrixXd, Eigen::MatrixXd > getInitialD(); // Initial Derivatives variable for the following optimization 

        Eigen::MatrixXd getA();
        Eigen::MatrixXd getQ();        
        Eigen::MatrixXd getC();
        Eigen::MatrixXd getL();

        Eigen::MatrixXd getR();
        Eigen::MatrixXd getRpp();
        Eigen::MatrixXd getRff();
        Eigen::MatrixXd getRfp();
        Eigen::MatrixXd getRpf();
};

#endif
