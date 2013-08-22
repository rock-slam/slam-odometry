/**@file KinematicModel.hpp
 *
 * @brief Kinematic Model Abstract class
 *
 * This is the Template Abstract class for the Forward kinematics of  Wheel or Leg Robot
 * considering all the DOFs in the Kinematics Chains (Trees). Basically the forward kinematics
 * transform wheel/Leg contact points position with respect to the body_frame considering the
 * joint values (chain configuration). The wheelJacobian or legJacobian works in velocity space
 * and gives information of each wheel/leg contact point to the contribution of the movement
 * (rover angular and linear velocities).
 *
 * It has been designed to use together with the MotionModel Solver (Least-Squares solver) imple
 * mented in the MOtionModel.hpp file of this library.
 *
 * It should first compute the forward kinematics in order to have afterwards the Robot Jacobian.
 * Neither the forward kinematics nor the Jacobian are provided by this class since this is an
 * abstract class and the kinematics is platform dependent. Your class should make this job.
 *
 *
 * The rational behind is to have a wrapper abstract class to solve the statistical motion model
 * of any complex robot. The goal is for odometry (localization) purpose in a unify way.
 * The real implemenetation of the Robot Kinematic Model which would inherits from this class can use
 * any method. This means that either analytical or numerical solutions can be internaly apply.
 * An example of numerical solution is the KDL library. An example of analytical is your own closed-form.
 * Since forward kinematics is normaly quite easy to get an analytical form is recomended.
 * However, if your robot kinematics is complex
 * and for control purpose you have a Tree structure representation for numerical
 * inverse kinematics in KDL/or similar. Perhaps, it is worthy to wrap this kinematic tree
 * implementation with this class.
 *
 * Anyway, the important aspect is to commit the interface. The nomenclature attempts to be generic.
 * An important issue with respect to a conventional kinematics is that a 3D slip vector
 * is modeled as part of the DoF of the kinematics in the form of displacement/rotation
 * vector (x, y and rot theta). Located at the frame between the robot contact point (feet/wheel)
 * and the ground.
 *
 * Further Details at:  P.Muir et. al Kinematic Modeling of Wheeled Mobile Robots
 *                      M. Tarokh et. al Kinematics Modeling and Analyses of Articulated Rovers
 *                      J. Hidalgo et. al Kinematics Modeling of a Hybrid Wheeled-Leg Planetary Rover
 *                      J. Hidalgo, Navigation and Slip Kinematics for High Performance Motion Models
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
    /**@class KinematicModel
     *
     * Kinematic Model Abtsract class
     *
     * @param _Scalar: is the typename of your desired implementation (float, double, etc..)
     * @param _RobotTrees: is the number of independent Trees connected to your desired Body Center.
     *              Trees is understood as connection of kinematics chains. As an example:
     *              <a href="http://robotik.dfki-bremen.de/en/forschung/robotersysteme/asguard-ii.html">Asguard</a>
     *              hybrid wheels model as Trees. Therefore Asguard has 4 Trees. Each Tree has 5 open
     *              kinematics chains, one per each foot which are potential points in contact with
     *              the ground.
     * @param _RobotJointDoF: Complete number of DoF in the Joint Space of your robot.This is every joint
     *              passive or active that your robot (or the part of your robot related to odometry)
     *              has. This is required to teh Jacobian in a a general form. Therefore, this would
     *              be the part of th enumber of columns in the Kinematic Model Jacobian.
     * @param _SlipDoF: The number of DoF than you want to model the slip of the contact point. If you
     *           are not interested to model slip velocity/displacement set it to zero. Otherwise,
     *           a common slip model has 3DoF (in X an Y direction and Z rotation)
     * @param _ContactDoF: This is the angle of contact between the ground and the contact point. If it 
     *              is not interesting for you model (i.e: the robot moves in indoor environment)
     *              set it to zero. Otherwise in uneven terrains normally is modeled as 1DoF along th
     *              pitch axis. For walking robots it could have 2DoF one along the Y axis/Pitch at the feet frame
     *              (dominant when moving forward) and another along the X axis/Roll at the feet frame
     *              (dominant when moving sideways).
     *
     * Note that it is very important how you especify the number of Trees according to the contact points.
     * It is assumed that one Tree can only have one single point of contact with the ground at the same time.
     * Therefore, if your real/physical kinematic Tree can have more than one contact point at a time (e.g: two)
     * the Tree needs to be split according to it (e.g: two Trees).
     *
     * For example, taking <a href="http://robotik.dfki-bremen.de/en/forschung/robotersysteme/asguard-ii.html">Asguard</a> wheel
     * as an example. If we want to model the wheel as two feet can have point in contact, two Trees
     * needs to be created in the wheel (virtualy increasing the number of Trees).
     *
     */
    template <typename _Scalar, int _RobotTrees, int _RobotJointDoF, int _SlipDoF, int _ContactDoF>
    class KinematicModel
    {
        public:
            /** The maximum number of DoF that the robot has in the Joint Space (Joints + Slip + Contact angle */
            static const unsigned int MAX_CHAIN_DOF = _RobotJointDoF+_SlipDoF+_ContactDoF;

            /** The complete number of DoF of the Robot Model. This is compute for the whole Robot Jacobian Matrix */
            static const unsigned int MODEL_DOF = _RobotJointDoF+_RobotTrees*(_SlipDoF+_ContactDoF);

        public:
            /** @brief return the number of Trees @_RobotTrees
             */
            inline int getNumberOfTrees()
            {
                return _RobotTrees;
            }
            /** @brief return the complete robot number of Joints
             */
            inline int getRobotJointDoF(void)
            {
                return _RobotJointDoF;
            }
            /** @brief return the dimension of the slip model.
             */
            inline int getSlipDoF(void)
            {
                return _SlipDoF;
            }
            /**@brief return the dimension of the contact angle model
             */
            inline int getContactDoF()
            {
                return _ContactDoF;
            }
            /**@brief return @MAX_CHAIN_DOF. Equivalent to the number of columns in teh Jacobian.
             */
            inline unsigned int getMaxChainDoF()
            {
                return MAX_CHAIN_DOF;
            }
            /**@brief return @MODEL_DOF.
             */
            inline unsigned int getModelDOF()
            {
                return MODEL_DOF;
            }

            /**@brief The name of the Kinematic Model
             */
            virtual std::string name() = 0;

            /**@brief Number of contact points per Tree
             *
             * Candidate points in contact with the groudn. For example in the case of
             * <a href="http://robotik.dfki-bremen.de/en/forschung/robotersysteme/asguard-ii.html">Asguard</a>
             * all the Trees have the same number of contact poins (5).
             * 
             * @param[out] contactPoints the vector of size number of Trees with the number of contact points
             */
            virtual void contactPointsPerTree (std::vector<unsigned int> &contactPoints) = 0;

            /**@brief Assign the current point in contact
             *
             * Point in contact among the number of contact points.
             *
             * @param[in] pointsInContact vector of size number of Trees with the point in contact per each Tree
             */
            virtual void setPointsInContact (const std::vector<int> &pointsInContact) = 0;

            /**@brief Forward Kinematic
             *
             * Computes the Forward Kinematics. It is computed for a chain, between one coordinate frame (the base)
             * to another coordinate frame in the chain (the tip).
             *
             * @param[in] chainIdx Identifier(idx) per chain. If one robot has four Trees with for example five kinematics chains per each Tree.
             *            The idx of the first chain in the first Tree is 0. The idx for the second chain of the 
             *            second Tree is 6 and so on.
             * @param[in] positions vector with the joint position of the kinematic chain. Since a priori is not possible to know the number
             *                          of DoF of the chain, the vector could have dimension MAX_CHAIN_DOF
             * @param[out] homogeneus transformation of the Forward Kinematics
             * @para,[out] noise covariance of the transformation.
             */
            virtual void fkBody2ContactPointt(const int chainIdx, const std::vector<_Scalar> &positions, Eigen::Affine3d &fkTrans, base::Matrix6d &fkCov) = 0;

            /**@brief Forward Kinematics Solver
             *
             * Compute the Forward Kinematics per each Tree of teh robot.
             * Since Forwadr Kinematics only makse sense in chains and not in Tree. The selected chain per each Tree is that one with the current 
             * point in contact @sa{setPointsInContact}
             *
             * @param[in] positions vector of size @sa{MODEL_DOF} with the joint position values of the complete robot.
             * @param [out] Eigen::Affine3d> &fkRobot, vector of homogeneus transformation of the FK per Tree.
             * @param[out] std::vector<base::Matrix6d> &fkCov, vector of noise covariance associated to each transformation.
             */
            virtual void fkSolver(const std::vector<_Scalar> &positions, std::vector<Eigen::Affine3d> &fkRobot, std::vector<base::Matrix6d> &fkCov) = 0;

            /**@brief Computes the Robot Jacobian
             *
             * Computes the robot Jacobian. Dimension 6*@sa{_RobotTrees} x @sa{MODEL_DOF}
             *
             * @param[in] positions with the current values joints values
             *
             * @return the robot jacobian matrix.
             */
            virtual Eigen::Matrix<_Scalar, 6*_RobotTrees, MODEL_DOF> jacobianSolver(const std::vector<_Scalar> &positions) = 0;
    };
}

#endif // ODOMETRY_KINEMATIC_MODEL_HPP

