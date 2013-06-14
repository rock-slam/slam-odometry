/**\file KinematicModel.hpp
 *
 * This is the Template Abstract class for the Forward kinematics of  Wheel or Leg Robot
 * considering all the DOFs in the Kinematics Chains (Trees). Basically the forward kinematics
 * transform wheel/Leg contact points position with respect to the body_frame considering the
 * joint values (chain configuration). The wheelJacobian or legJacobian works in velocity space
 * and gives information of each wheel/leg contact point to the contribution of the movement
 * (rover angular and linear velocities).
 * It has been designed to be used together with the MotionModel Solver (Least-Squares solver)
 *
 * It first computes the forward kinematics in order to have afterwards the Robot Jacobian.
 *
 * Further Details at:  P.Muir et. al Kinematic Modeling of Wheeled Mobile Robots
 *                      M. Tarokh et. al Kinematics Modeling and Analyses of Articulated Rovers
 *                      J. Hidalgo et. al Kinematics Modeling of a Hybrid Wheeled-Leg Planetary Rover
 *                      J. Hidalgo, Navigation and Slip Kinematics for High Performance Motion Models
 *
 * To-DO: Document how to use the class
 * 
 * @author Javier Hidalgo Carrio | DFKI RIC Bremen | javier.hidalgo_carrio@dfki.de
 * @date May 2013.
 * @version 1.0.
 */



#ifndef ODOMETRY_KINEMATIC_MODEL_HPP
#define ODOMETRY_KINEMATIC_MODEL_HPP

#include <string> // std string library
#include <vector> // std vector library
#include <base/eigen.h> //base eigen of rock

namespace odometry
{
    template <typename _Scalar, int _RobotTrees, int _RobotJointDoF, int _SlipDoF, int _ContactDoF>
    class KinematicModel
    {
        public:
            static const unsigned int MAX_CHAIN_DOF = _RobotJointDoF+_SlipDoF+_ContactDoF;
            static const unsigned int MODEL_DOF = _RobotJointDoF+_RobotTrees*(_SlipDoF+_ContactDoF);

        public:
            inline int getNumberOfTrees()
            {
                return _RobotTrees;
            }
            inline int getRobotJointDoF(void)
            {
                return _RobotJointDoF;
            }
            inline int getSlipDoF(void)
            {
                return _SlipDoF;
            }
            inline int getContactDoF()
            {
                return _ContactDoF;
            }
            inline unsigned int getMaxChainDoF()
            {
                return MAX_CHAIN_DOF;
            }
            inline unsigned int getModelDOF()
            {
                return MODEL_DOF;
            }

            virtual std::string name() = 0;

            virtual void contactPointsPerTree (std::vector<unsigned int> &contactPoints) = 0;

            virtual void setPointsInContact (const std::vector<int> &pointsInContact) = 0;

            virtual void fkBody2ContactPointt(const int chainIdx, const std::vector<_Scalar> &positions, Eigen::Affine3d &fkTrans, base::Matrix6d &fkCov) = 0;

            virtual void fkSolver(const std::vector<_Scalar> &positions, std::vector<Eigen::Affine3d> &fkRobot, std::vector<base::Matrix6d> &fkCov) = 0;

            virtual Eigen::Matrix<_Scalar, 6*_RobotTrees, MODEL_DOF> jacobianSolver(const std::vector<_Scalar> &positions) = 0;
    };
}

#endif // ODOMETRY_KINEMATIC_MODEL_HPP

