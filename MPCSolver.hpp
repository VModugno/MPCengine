#ifndef MPCSOLVER_HPP
#define MPCSOLVER_HPP
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>
#include "qpOASES/qpOASES.hpp"

struct Aff3d{ Eigen::Vector3d translation; Eigen::MatrixXd rotation; };

namespace mpcSolver{

    class MPCSolver{
	public:
        MPCSolver(double, double, double, Eigen::Vector3d, double, double, double, double, double, double, double, double, double, double);

        // Main method
        void solve(Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, Aff3d, bool, double, double, double, double);

        // Get stuff
        /*Eigen::VectorXd getOptimalCoMPosition();
        Eigen::VectorXd getOptimalCoMVelocity();
        Eigen::VectorXd getOptimalZMPPosition();
        Eigen::VectorXd getOptimalFootsteps();
        Eigen::MatrixXd getPredictedZmp();
        bool supportFootHasChanged();
	    double getOmega(); */

	    // Set stuff
	    /*void setComTargetHeight(double);
	    void setReferenceVelocityX(double);
	    void setReferenceVelocityY(double);
	    void setReferenceVelocityOmega(double);
	    void setVelGain(double);
	    void setZmpGain(double);*/

        // Generate matrices
        void genCostFunction();
        void genStabilityConstraint();
        void genBalanceConstraint();
        void genFeasibilityConstraint();
        void genSwingFootConstraint(Aff3d);

        // Solve
        void computeOrientations();
        Eigen::VectorXd solveQP();
        //Eigen::VectorXd solveQPdart();

        // Update the state
        Eigen::Vector3d updateState(double,int,double);
        void changeReferenceFrame(Aff3d);

        // Log
        void logToFile();

	private:

        // Constant parameters


        int n,m,p,N;

        //double singleSupportDuration, doubleSupportDuration, thetaMax; // outside
        //double footConstraintSquareWidth;
        //double deltaXMax;
        //double deltaYIn;
        //double deltaYOut;
        double mpcTimeStep;
        double controlTimeStep;
        // double comTargetHeight;
        //double omega;
        //double measuredComWeight = 0;
        //double measuredZmpWeight = 0;

        // Parameters for the current iteration
        //bool supportFoot;
        double simulationTime;
        //double vRefX=0;
        //double vRefY=0;
        //double omegaRef=0;
        int mpcIter,controlIter;

        // Matrices for prediction
        Eigen::VectorXd p;
        Eigen::MatrixXd P;
        Eigen::MatrixXd Vu;
        Eigen::MatrixXd Vs;

        Eigen::MatrixXd S;
        Eigen::MatrixXd T;

        // Matrices for cost function
        Eigen::MatrixXd H;
        Eigen::VectorXd F;

        // Matrices for  constraint
        Eigen::MatrixXd G;
        Eigen::VectorXd W;
        Eigen::VectorXd S;

        // Cost function weights
        double qZd = 1;
        double qVx = 0;//100;
        double qVy = 0;//100;
        double qZ = 1000;

        // State
        //Eigen::Vector3d comPos;
        //Eigen::Vector3d comVel;
        //Eigen::Vector3d zmpPos;
        //Eigen::Vector4d predictedFootstep;

	    // Quadratic problem
	    qpOASES::QProblem qp;

	    // Some vectors to plot
	    Eigen::MatrixXd predictedZmp;
   };

}

#endif
