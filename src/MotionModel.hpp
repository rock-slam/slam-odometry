/**\file MotionModel.hpp
 *
 * This clase provided a Motion Model solver to any mobile robot (wheel or leg)
 * with articulated joints.
 *
 * TO-DO: Document how to use the class and how to properly set the template values
 * with some robot examples.
 *
 * Further Details at:  P.Muir et. al Kinematic Modeling of Wheeled Mobile Robots
 *                      M. Tarokh et. al Kinematics Modeling and Analyses of Articulated Rovers
 *                      J. Hidalgo et. al Kinematics Modeling of a Hybrid Wheeled-Leg Planetary Rover
 *                      J. Hidalgo, Navigation and Slip Kinematics for High Performance Motion Models
 *
 * @author Javier Hidalgo Carrio | DFKI RIC Bremen | javier.hidalgo_carrio@dfki.de
 * @date June 2013.
 * @version 1.0.
 */


#ifndef ODOMETRY_MOTION_MODEL_HPP
#define ODOMETRY_MOTION_MODEL_HPP

#include <iostream>
#include <vector> //! For std vector
#include <boost/shared_ptr.hpp> //! For std shared pointer
#include <base/logging.h> //! Log meassges
#include <Eigen/Geometry> /** Eigen data type for Matrix, Quaternion, etc... */
#include <Eigen/Core> /** Core methods of Eigen implementation **/
#include <Eigen/Dense> /** for the algebra and transformation matrices **/
#include <Eigen/Cholesky> /** For the Cholesky decomposition **/
#include "KinematicModel.hpp"


#define DEBUG_PRINTS_ODOMETRY_MOTION_MODEL 1 //TO-DO: Remove this. Only for testing (master branch) purpose

namespace odometry
{
    struct TreeContactPoint
    {
        /** Number of contact point of this tree. It cannot be zero or negative **/
        unsigned int number;

        /** Point in contact between 0 and (number-1) or NO_CONTACT if the case **/
        int contactId;

        TreeContactPoint()
            : number(1), contactId(0) {}
    };

    template <typename _Scalar, int _RobotTrees, int _RobotJointDoF, int _SlipDoF, int _ContactDoF>
    class MotionModel
    {
        public:
            static const unsigned int MAX_CHAIN_DOF = _RobotJointDoF+_SlipDoF+_ContactDoF;
            static const unsigned int MODEL_DOF = _RobotJointDoF+_RobotTrees*(_SlipDoF+_ContactDoF);
            static const int NO_CONTACT = -1;

        public:
            /** Pointer to the kinematic model **/
            typedef boost::shared_ptr< KinematicModel <_Scalar, _RobotTrees, _RobotJointDoF, _SlipDoF, _ContactDoF> > kinematics_ptr;

            /** Types **/
            enum methodContactPoint
            {
                LOWEST_POINT  = 0,
                COMBINATORICS  = 1
            };

            methodContactPoint contactSelection; /** Method to select the contact points **/
            kinematics_ptr robotModel; /** Kinematic model of a robot **/
            std::vector<Eigen::Affine3d> fkRobot; /** Forward kinematics of the robot chains **/
            std::vector<base::Matrix6d> fkCov; /** Uncertainty of the forward kinematics (if any) **/
            std::vector<TreeContactPoint> contactPoints; /** Number and point in contact per each tree of the model **/

        protected:

            bool contactPointComparison(Eigen::Affine3d fkRobot1, Eigen::Affine3d fkRobot2)
            {
                return fkRobot1.translation()[2] < fkRobot2.translation()[2];
            }

            void lowestPointInContact()
            {
                register int j = 0;

                /** For all the trees of the model **/
                for (std::vector<TreeContactPoint>::iterator it = contactPoints.begin() ; it != contactPoints.end(); it++)
                {
                    /** More than one contact point per this tree **/
                    if ((*it).number > 1)
                    {
                        double zdistance = 0.00;
                        for (register unsigned int i=0; i<(*it).number; ++i)
                        {
                            if (zdistance > fkRobot[j+i].translation()[2])
                            {
                                zdistance = fkRobot[j+i].translation()[2];
                                (*it).contactId = i;
                            }
                        }
                    }
                    else if ((*it).number == 1) /** If there is only one contact point then it is that one **/
                    {
                        contactPoints[j].contactId = 0;
                    }
                    else /** This tree does not have contact **/
                    {
                        contactPoints[j].contactId = NO_CONTACT;
                    }

                    j = j+(*it).number;
                }

                #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                std::cout<<"[MOTION_MODEL] Selected Points in Contact:";
                for (std::vector<TreeContactPoint>::iterator it = contactPoints.begin() ; it != contactPoints.end(); it++)
                {
                    std::cout<<" "<<(*it).contactId;
                }
                std::cout<<"\n";
                #endif


                return;
            }

            void combinatoricsPointInContact()
            {
                return;
            }

            void selectPointsInContact(MotionModel::methodContactPoint method)
            {

                /** Select the method for the points in contact **/
                if (method == LOWEST_POINT)
                {
                    #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                    std::cout<< "[MOTION_MODEL SelectPoints] Lowest Points method\n";
                    #endif
                    this->lowestPointInContact();
                }
                else if (method == COMBINATORICS)
                {
                    #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                    std::cout<< "[MOTION_MODEL SelectPoints] Combinatorics Points method\n";
                    #endif

                    this->combinatoricsPointInContact();
                }

               return;
            }

            inline void navEquations(const Eigen::Matrix <_Scalar, 6, 1> &cartesianVelocities,
                                    const Eigen::Matrix <_Scalar, MODEL_DOF, 1> &modelVelocities,
                                    const Eigen::Matrix <_Scalar, 6*_RobotTrees, MODEL_DOF> &J,
                                    const Eigen::Matrix <_Scalar, 6, 6> &cartesianVelCov,
                                    const Eigen::Matrix <_Scalar, MODEL_DOF, MODEL_DOF> &modelVelCov,
                                    Eigen::Matrix <_Scalar, 6*_RobotTrees, 3+_RobotTrees+(_RobotTrees*_ContactDoF)> &unknownA,
                                    Eigen::Matrix <_Scalar, 3+_RobotTrees+(_RobotTrees*_ContactDoF), 1> &unknownx,
                                    Eigen::Matrix <_Scalar, 6*_RobotTrees, 3+_RobotJointDoF> &knownB,
                                    Eigen::Matrix <_Scalar, 3+_RobotJointDoF, 1> &knowny,
                                    Eigen::Matrix <_Scalar, 6*_RobotTrees, 6*_RobotTrees> &Weight)
            {
                Eigen::Matrix<_Scalar, 6*_RobotTrees, 6> spareI;

                #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                std::cout<<"[MOTION_MODEL] navEquations\n";
                #endif

                /** Compute the composite rover equation matrix I **/
                for (register int i=0; i<_RobotTrees; ++i)
                    spareI.template block<6, 6>(i*6, 0) = Eigen::Matrix <_Scalar, 6, 6>::Identity();

                /** Form the unknownA matrix **/
                unknownA.template block<6*_RobotTrees, 3> (0,0) = spareI.template block<6*_RobotTrees, 3> (0,0);//Linear Velocities

                /** Form the unknownx **/
                unknownx.template block<3, 1> (0,0) = cartesianVelocities.template block<3, 1> (0,0);

                /** Non-holonomic constraint Slip vector **/
                for (register int i=0; i<_RobotTrees; ++i)
                {
                    unknownA.col(3+i) = -J.col(_RobotJointDoF+(_SlipDoF*(i+1)-1));
                    unknownx[3+i] = modelVelocities[_RobotJointDoF+(_SlipDoF*(i+1)-1)];
                }

                /** Contact angles per each Tree **/
                for (register int i=0; i<(_RobotTrees*_ContactDoF); ++i)
                {
                    unknownA.col(3+_RobotTrees+i) = -J.col(_RobotJointDoF+(_RobotTrees*_SlipDoF)+i);
                    unknownx[3+_RobotTrees+i] = modelVelocities[_RobotJointDoF+(_RobotTrees*_SlipDoF)+i];
                }

                /** Form the knownB matrix **/
                knownB.template block< (6* _RobotTrees), 3> (0,0) = -spareI.template block<6*_RobotTrees, 3> (0,3); //Angular velocities
                knownB.template block< (6* _RobotTrees), _RobotJointDoF> (0,3) = J.template block<6*_RobotTrees, _RobotJointDoF> (0,0);

                /** Form the knowny **/
                knowny.template block<3, 1> (0,0) = cartesianVelocities.template block<3, 1> (3,0);
                knowny.template block<_RobotJointDoF, 1> (3,0) = modelVelocities.template block<_RobotJointDoF, 1> (0,0);

                /** Form the Weighting matrix **/
                Weight.setIdentity();

                #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                std::cout<< "[MOTION_MODEL] spareI is of size "<<spareI.rows()<<"x"<<spareI.cols()<<"\n";
                std::cout << "[MOTION_MODEL] The spareI matrix \n" << spareI << std::endl;
                std::cout<< "[MOTION_MODEL] unknownA is of size "<<unknownA.rows()<<"x"<<unknownA.cols()<<"\n";
                std::cout << "[MOTION_MODEL] The unknownA matrix \n" << unknownA << std::endl;
                std::cout<< "[MOTION_MODEL] knownB is of size "<<knownB.rows()<<"x"<<knownB.cols()<<"\n";
                std::cout << "[MOTION_MODEL] The knownB matrix \n" << knownB << std::endl;
                #endif

                return;
            }

        public:

            MotionModel()
            {
            }

            MotionModel(bool &status, MotionModel::methodContactPoint method,  kinematics_ptr robotModel)
            {
                std::vector<unsigned int> numberContactPoints (_RobotTrees, 0);

                /** Size the contactPoints variable **/
                contactPoints.resize (_RobotTrees);

                /** Assign the robot model **/
                this->robotModel = robotModel;
                #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                std::cout<<"[MOTION_MODEL] Constructor\n";
                #endif

                /** Know the number of contact points per tree **/
                robotModel->contactPointsPerTree(numberContactPoints);

                /** Assign the type of method to select the contact points in contact **/
                this->contactSelection = method;

                #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                std::cout<<"[MOTION_MODEL] Get number of trees "<<robotModel->getNumberOfTrees()<<"\n";
                #endif

                if ((robotModel->getNumberOfTrees() == static_cast<int>(numberContactPoints.size())) &&(numberContactPoints.size() == _RobotTrees))
                {
                    /** Fill the internal contact point variable **/
                    for (register unsigned int i=0; i<numberContactPoints.size(); ++i)
                    {
                        contactPoints[i].number = numberContactPoints[i]; //!Number of contact points
                        contactPoints[i].contactId = 0; //! By default set the contact point zero.
                    }

                    /** Print information message **/
                    LOG_INFO("[MOTION_MODEL] Created Motion Model using %s Robot Kinematics implementation\n", this->robotModel->name().c_str());
                    LOG_INFO("[MOTION_MODEL] %s has %d independent kinematic trees.\n", this->robotModel->name().c_str(), this->robotModel->getNumberOfTrees());
                    status = true;
                }
                else
                {
                    LOG_ERROR("[MOTION_MODEL] Malfunction of the MotionModel. WRONG number of template parameters\n");
                    status = false;
                }

                #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                std::cout<<"[MOTION_MODEL] contactPoints contains: ";
                for (std::vector<TreeContactPoint>::iterator it = contactPoints.begin() ; it != contactPoints.end(); it++)
                {
                    std::cout<<" number: "<<(*it).number<<" contactId: "<<(*it).contactId;
                }

                std::cout<<"\n[MOTION_MODEL] **** END ****\n";
                #endif

                return;
            }

            ~MotionModel()
            {
            }

            void updateKinematics (const Eigen::Matrix <_Scalar, MODEL_DOF, 1> &modelPositions)
            {
                register int i=0;
                std::vector<_Scalar> vectorPositions(MODEL_DOF, 0);
                std::vector<int> contactId (_RobotTrees, 0);

                /** Copy Eigen to vector **/
                Eigen::Map <Eigen::Matrix <_Scalar, MODEL_DOF, 1> > (&(vectorPositions[0]), MODEL_DOF) = modelPositions;

                /** Solve the forward kinematics for the complete Robot (all the chains)**/
                robotModel->fkSolver(vectorPositions, fkRobot, fkCov);

                /** Select the contact point **/
                this->selectPointsInContact (this->contactSelection);

                /** Set the contact point to the kinematic model **/
                for (std::vector<TreeContactPoint>::iterator it = contactPoints.begin() ; it != contactPoints.end(); it++)
                {
                    contactId[i] = (*it).contactId;
                    i++;
                }
                robotModel->setPointsInContact(contactId);

                return;
            }

            inline virtual void getKinematics (std::vector<Eigen::Affine3d> &currentFkRobot, std::vector<base::Matrix6d> &currentFkCov)
            {
                currentFkRobot = this->fkRobot;
                currentFkCov = this->fkCov;

                return;
            }

            virtual std::vector< int > getPointsInContact ()
            {
                register int i=0;
                std::vector<int> contactId (_RobotTrees, 0);

                for (std::vector<TreeContactPoint>::iterator it = contactPoints.begin() ; it != contactPoints.end(); it++)
                {
                    contactId[i] = (*it).contactId;
                    i++;
                }

                return contactId;
            }



            virtual double navSolver(const Eigen::Matrix <_Scalar, MODEL_DOF, 1> &modelPositions,
                                            Eigen::Matrix <_Scalar, 6, 1> &cartesianVelocities,
                                            Eigen::Matrix <_Scalar, MODEL_DOF, 1> &modelVelocities,
                                            Eigen::Matrix <_Scalar, 6, 6> &cartesianVelCov,
                                            Eigen::Matrix <_Scalar, MODEL_DOF, MODEL_DOF> &modelVelCov)
            {
                double leastSquaresError = std::numeric_limits<double>::quiet_NaN(); //solution error of the Least-Squares
                std::vector<_Scalar> vectorPositions(MODEL_DOF, 0); // model positions in std_vector form
                Eigen::Matrix <_Scalar, 6*_RobotTrees, MODEL_DOF> J; // Robot Jacobian
                Eigen::Matrix <_Scalar, 6*_RobotTrees, 3+_RobotTrees+(_RobotTrees*_ContactDoF)> unknownA; // Nav non-sensed values matrix
                Eigen::Matrix <_Scalar, 3+_RobotTrees+(_RobotTrees*_ContactDoF), 1> unknownx; // Nav non-sensed values vector
                Eigen::Matrix <_Scalar, 6*_RobotTrees, 3+_RobotJointDoF> knownB; // Nav sensed values matrix
                Eigen::Matrix <_Scalar, 3+_RobotJointDoF, 1> knowny; // Nav sensed values vector
                Eigen::Matrix <_Scalar, 6*_RobotTrees, 6*_RobotTrees> Weight; // (Sensor noise)^(-1) and Trees Weighting matrix

                /** Initialize  variables **/
                unknownA.setZero(); unknownx.setZero();
                knownB.setZero(); knowny.setZero();
                Weight.setZero();

                #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                std::cout << "[MOTION_MODEL] cartesianVelocities is of size "<<cartesianVelocities.rows()<<"x"<<cartesianVelocities.cols()<<"\n";
                std::cout << "[MOTION_MODEL] cartesianVelocities is \n" << cartesianVelocities<< std::endl;

                std::cout << "[MOTION_MODEL] modelVelocities is of size "<<modelVelocities.rows()<<"x"<<modelVelocities.cols()<<"\n";
                std::cout << "[MOTION_MODEL] modelVelocities is \n" << modelVelocities<< std::endl;
                #endif

                /** Copy Eigen to vector **/
                Eigen::Map <Eigen::Matrix <_Scalar, MODEL_DOF, 1> > (&(vectorPositions[0]), MODEL_DOF) = modelPositions;

                /** Solve the Robot Jacobian Matrix **/
                J = robotModel->jacobianSolver (vectorPositions);

                #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                std::cout<< "[MOTION_MODEL] J is of size "<<J.rows()<<"x"<<J.cols()<<"\n";
                std::cout << "[MOTION_MODEL] The J matrix \n" << J << std::endl;
                #endif

                /** Form the Composite Navigation Equations and Noise Covariance **/
                this->navEquations (cartesianVelocities, modelVelocities, J,
                        cartesianVelCov, modelVelCov, unknownA, unknownx, knownB, knowny, Weight);

                /** Solve the Motion Model by Least-Squares (navigation kinematics) **/
                Eigen::Matrix <_Scalar,3+_RobotTrees+(_RobotTrees*_ContactDoF), 3+_RobotTrees+(_RobotTrees*_ContactDoF)> pseudoInvUnknownA;
                Eigen::Matrix <_Scalar, 6*_RobotTrees, 1> knownb = knownB*knowny;

                #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                /** DEBUG OUTPUT **/
                typedef Eigen::Matrix <_Scalar, 6*_RobotTrees, 3+_RobotTrees+(_RobotTrees*_ContactDoF)> matrixAType;
                typedef Eigen::Matrix <_Scalar, 6*_RobotTrees, 3+_RobotJointDoF> matrixBType;
                typedef Eigen::Matrix <_Scalar, 6*_RobotTrees, 3+_RobotTrees+(_RobotTrees*_ContactDoF)+1> matrixConjType; // columns are columns of A + 1

                matrixConjType Conj;

                Eigen::FullPivLU<matrixAType> lu_decompA(unknownA);
                std::cout << "[MOTION_MODEL] The rank of A is " << lu_decompA.rank() << std::endl;

                Eigen::FullPivLU<matrixBType> lu_decompB(knownB);
                std::cout << "[MOTION_MODEL] The rank of B is " << lu_decompB.rank() << std::endl;

                Conj.template block<6*_RobotTrees,3+_RobotTrees+(_RobotTrees*_ContactDoF)>(0,0) = unknownA;
                Conj.template block<6*_RobotTrees, 1>(0,3+_RobotTrees+(_RobotTrees*_ContactDoF)) = knownb;
                Eigen::FullPivLU<matrixConjType> lu_decompConj(Conj);
                std::cout << "[MOTION_MODEL] The rank of A|B*y is " << lu_decompConj.rank() << std::endl;
                std::cout << "[MOTION_MODEL] Pseudoinverse of A\n" << (unknownA.transpose() * Weight * unknownA).inverse() << std::endl;
                /*******************/
                #endif

                pseudoInvUnknownA = (unknownA.transpose() * Weight * unknownA).inverse();
                unknownx = pseudoInvUnknownA * unknownA.transpose() * Weight * knownb;

                /** Error of the solution **/
                Eigen::Matrix<double, 1,1> squaredError = (((unknownA*unknownx - knownb).transpose() * Weight * (unknownA*unknownx - knownb)));
                if (knownb.norm() != 0.00)
                    leastSquaresError = sqrt(squaredError[0]) / knownb.norm();

                /** Save the results in the parameters (NaN previous values are now known quantities) **/
                cartesianVelocities.template block<3, 1>(0,0) = unknownx.template block<3, 1>(0,0); // Linear velocities
                cartesianVelCov.template block<3, 3> (0,0) = pseudoInvUnknownA.template block<3, 3> (0,0);//Linear Velocities noise

                /** Non-holonomic constraint Slip vector **/
                for (register int i=0; i<_RobotTrees; ++i)
                {
                    modelVelocities[_RobotJointDoF+(_SlipDoF*(i+1)-1)] = unknownx[3+i];
                    modelVelCov.col(_RobotJointDoF+(_SlipDoF*(i+1)-1))[_RobotJointDoF+(_SlipDoF*(i+1)-1)] = pseudoInvUnknownA.col(3+i)[3+i];
                }

                /** Contact angles per each Tree **/
                for (register int i=0; i<(_RobotTrees*_ContactDoF); ++i)
                {
                    modelVelocities[_RobotJointDoF+(_RobotTrees*_SlipDoF)+i] = unknownx[3+_RobotTrees+i];
                    modelVelCov.col(_RobotJointDoF+(_RobotTrees*_SlipDoF)+i)[_RobotJointDoF+(_RobotTrees*_SlipDoF)+i] = pseudoInvUnknownA.col(3+_RobotTrees+i)[3+_RobotTrees+i];
                }

                #ifdef DEBUG_PRINTS_ODOMETRY_MOTION_MODEL
                std::cout << "[MOTION_MODEL] L-S solution:\n"<<unknownx<<std::endl;
                std::cout << "[MOTION_MODEL] cartesianVelocities is of size "<<cartesianVelocities.rows()<<"x"<<cartesianVelocities.cols()<<"\n";
                std::cout << "[MOTION_MODEL] cartesianVelocities is \n" << cartesianVelocities<< std::endl;

                std::cout << "[MOTION_MODEL] modelVelocities is of size "<<modelVelocities.rows()<<"x"<<modelVelocities.cols()<<"\n";
                std::cout << "[MOTION_MODEL] modelVelocities is \n" << modelVelocities<< std::endl;
                std::cout << "[MOTION_MODEL] modelVelCov is of size "<<modelVelCov.rows()<<"x"<<modelVelCov.cols()<<"\n";
                std::cout << "[MOTION_MODEL] modelVelCov is \n" << modelVelCov<< std::endl;
                std::cout << "[MOTION_MODEL] The absolute squared error is:\n" << squaredError << std::endl;
                std::cout << "[MOTION_MODEL] The relative error is:\n" << leastSquaresError << std::endl;
                #endif


                return leastSquaresError;
            }

            virtual base::Vector6d slipSolver(void)
            {
                base::Vector6d s;

                return s;
            }
    };
}

#endif //ODOMETRY_MOTION_MODEL_HPP

