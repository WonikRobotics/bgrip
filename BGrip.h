/* Allegro, Copyright 2021 Wonik Robotics Co., Ltd. All rights reserved.
 *
 * This library is commercial and cannot be redistributed, and/or modified
 * WITHOUT ANY ALLOWANCE OR PERMISSION OF Wonik Robotics Co., LTD.
 */

/**
 * @file BGrip.h
 * @author Wonik Robotics
 * @brief Class definition of BGrip.
 *
 * @mainpage BGrip
 * @section INTRODUCTION
 * Implementation of Allegro Gripper grasping algorithm.<br>
 * 
 * @section CREATEINFO
 * - Author: Sangyup (Sean) Yi
 * - Data: 2021/07/21
 * @section MODIFYINFO
 * - 2021/07/21: Commit this document. (by Sean)
 */

#ifndef __BGRIP_H__
#define __BGRIP_H__

#include "BGripDef.h"

#define NOF 4 // number of fingers
#define FI_I 2	// index of index finger
#define FI_M 0	// index of middle finger
#define FI_T 1	// index of thumb
#define NOJ 4 // number of joints in each finger
#define SIZEOF_VARRAY (NOF*NOJ*8) // double array

/**
 * Mode of grasp motion.
 * @brief Grasp mode set by user.
 */
enum eGraspMode
{
	eGraspMode_NONE,
	eGraspMode_GROPED,
	eGraspMode_PARALLEL,
	eGraspMode_PINCH_INDEX,
	eGraspMode_PINCH_MIDDLE,
	eGraspMode_PINCH_PEG,
	eGraspMode_ENVELOP,
	NUMBER_OF_GRASP_MODE
};

/**
* Type of motion.
* @brief Motion type.
*/
enum eMotionType
{
	eMotionType_NONE,
	eMotionType_READY,
	eMotionType_GRASP,
	eMotionType_GRAVITY_COMP,
	eMotionType_JOINT_PD,
	NUMBER_OF_MOTION_TYPE
};

/**
 * Type of gripper. It is input argument to instanciate BGrip class.
 * @brief It specifies whether it is for 9-dof or 11-dof gripper.
 */
enum eGripperType
{
	eGripperType_9DOF = 0,				///< 9-dof gripper
	eGripperType_11DOF					///< 11-dof gripper
};

/**
 * BGrip class.
 * @brief Allegro Gripper grasping algorithm.
 * @author Jihoon Bae, Seay Yi
 */
class BGRIPEXPORT BGrip
{
public:
	BGrip(eGripperType gt);
	virtual ~BGrip(void);

	/**
	 * Get the type of hand.
	 * @return It returns gripper type, whether it is 9-dof or 11-dof gripper.
	 * @see eGripperType
	 */
	eGripperType GetType();

	/**
	 * Set time interval.
	 * Algorithm assumes UpdateControl() is called once in this time interval.
	 * @param dT Time interval in seconds.
	 */
	void SetTimeInterval(double dT);

	/**
	 * Get time interval.
	 * Algorithm assumes UpdateControl() is called once in this time interval.
	 * @return Time interval in seconds.
	 */
	double GetTimeInterval();

	/**
	 * Set grasp mode.
	 * @param graspMode mode of grasp to set.
	 * @see eGraspMode
	 */
	void SetGraspMode(int graspMode);

	/**
	 * Set motion type.
	 * @param motionType type of motion to set.
	 * @see eMotionType
	 */
	void SetMotionType(int motionType);

	/**
	 * Enable or disable gravity compensation.
	 * @param on enable(true) or not(false).
	 */
	void EnableGravCompensation(bool on);

	/**
	* Set gripper open size.
	* @param openSize how wide gripper tips are open. (0~100)
	*/
	void SetOpenSize(double openSize);

	/**
	 * Set joint position of each joint. 
	 * This function should be called once in a control loop before UpdateControl() is called.
	 * @param q Array of joint positions in radian.
	 */
	void SetJointPosition(double* q);

	/**
	 * Update control algorithm callback. This function should be called in each control time step manually.
	 * Desired joint torques are computed once in every control loop.
	 * @param time Current time in second.
	 */
	void UpdateControl(double time);

	/**
	 * Get desired(computed) joint torques.
	 * This function should be called once in a control loop after UpdateControl() is called.
	 * @param tau [out] Array of desired joint torques.
	 */
	void GetJointTorque(double* tau);

	/**
	 * Get forward kinematic solution for each finger.
	 * @param x [out] x coordinate of each fingertip.
	 * @param y [out] y coordinate of each fingertip.
	 * @param z [out] z coordinate of each fingertip.
	 */
	void GetFKResult(double x[NOF], double y[NOF], double z[NOF]);

	/**
	 * Set desired joint position.
	 * @param q Array of desired joint positions in radian.
	 */
	void SetJointDesiredPosition(double* q);

	/**
	 * Set control gains explicitly.
	 * @param kp proportional gains
	 * @param kd derivative gains
	 */
	void SetGainsEx(double* kp, double* kd);

	/**
	 * Set desired grasping forces.
	 * @param f desired grasping force for each finger
	 */
	void SetGraspingForce(double f[NOF]);

	/**
	 * Set scalar for the preset envelop grasp torque.
	 * @param set_scalar will be multiplied by the pre-computed grasping torque
	 * at each joint. Default is one to rest the torque to its original value
	 */
	void SetEnvelopTorqueScalar(double set_scalar=1.0);

	/**
	 * Get desired grasping forces.
	 * @param fx [out] desired grasping force in x-direction for each finger
	 * @param fy [out] desired grasping force in y-direction for each finger
	 * @param fz [out] desired grasping force in z-direction for each finger
	 */
	void GetGraspingForce(double fx[NOF], double fy[NOF], double fz[NOF]);

	/**
	 * Set orientation
	 */
	void SetOrientation(double roll, double pitch, double yaw);
	void SetOrientation(double R[9]);


private:
	BGrip();

protected:
	virtual void SetGains(int motionType);
	virtual void SolveFK() = 0;
	virtual void CalculateJacobian() = 0;
	virtual void CalculateGravity() = 0;
	virtual void CalculateGravityEx() = 0;

	virtual void Motion_Ready() = 0;
	virtual void Motion_GravityComp() = 0;
	virtual void Motion_Grasp() = 0;
	virtual void Motion_JointPD() = 0;

protected:
	double _dT;							///< control time step (second)
	double _curT;
	
	eGripperType _gripperType;			///< whether it is 9-dof or 11-dof gripper
	eGraspMode _graspMode;				///< mode of grasp motion currently set
	eMotionType _motionType;			///< type of motion
	double _openSize = 0; // how wide girpper tips are open in degree.
	
	double _q[NOF][NOJ];				///< current joint angle (radian)
	double _q_filtered[NOF][NOJ];		///< current joint angle (radian, low pass filtered)
	double _q_pre[NOF][NOJ];			///< previous(last) joint angle (radian)
	double _q_filtered_pre[NOF][NOJ];	///< previous(last) joint angle (radian, low pass filtered)
	double _qdot[NOF][NOJ];				///< joint velocity (radian/sec)
	double _qdot_filtered[NOF][NOJ];	///< joint velocity (radian/sec, low pass filtered)
	double _qdot_pre[NOF][NOJ];			///< previous(last) joint velocity (radian/sec)
	double _tau_des[NOF][NOJ];			///< desired joint torque
	double _q_des[NOF][NOJ];			///< desired joint position used in joint pd control motion

	double _envelop_torque_scalar;		///< used to control (scale) the torque applied during enveloping grasp

	double _kp[NOF][NOJ];				///< proportional control gain for each joint
	double _kd[NOF][NOJ];				///< derivative control gain for each joint
	double _kp_task[NOF][NOJ];			///<
	double _kd_task[NOF][NOJ];			///<

	double _x[NOF];						///< x position of finger tip along with cartesian space
	double _y[NOF];						///< y position of finger tip along with cartesian space
	double _z[NOF];						///< z position of finger tip along with cartesian space
	double _x_filtered[NOF];			///< x position of finger tip along with cartesian space (low pass filtered)
	double _y_filtered[NOF];			///< y position of finger tip along with cartesian space (low pass filtered)
	double _z_filtered[NOF];			///< z position of finger tip along with cartesian space (low pass filtered)
	double _x_pre[NOF];					///< previous x position of finger tip along with cartesian space
	double _y_pre[NOF];					///< previous y position of finger tip along with cartesian space
	double _z_pre[NOF];					///< previous z position of finger tip along with cartesian space
	double _x_filtered_pre[NOF];		///< previous x position of finger tip along with cartesian space (low pass filtered)
	double _y_filtered_pre[NOF];		///< previous y position of finger tip along with cartesian space (low pass filtered)
	double _z_filtered_pre[NOF];		///< previous z position of finger tip along with cartesian space (low pass filtered)
	
	double _xdot[NOF];					///< velocity of finger tip along with x-axis
	double _ydot[NOF];					///< velocity of finger tip along with y-axis
	double _zdot[NOF];					///< velocity of finger tip along with z-axis
	double _xdot_filtered[NOF];			///< velocity of finger tip along with x-axis (low pass filtered)
	double _ydot_filtered[NOF];			///< velocity of finger tip along with y-axis (low pass filtered)
	double _zdot_filtered[NOF];			///< velocity of finger tip along with z-axis (low pass filtered)
	double _xdot_pre[NOF];				///< previous velocity of finger tip along with x-axis
	double _ydot_pre[NOF];				///< previous velocity of finger tip along with y-axis
	double _zdot_pre[NOF];				///< previous velocity of finger tip along with z-axis

	double _J[NOF][3][NOJ];				///< Jacobian

	double _G[NOF][NOJ];				///< gravitational vector
	bool _grav_comp_enabled;			///< enabled of disabled of gravity compensation

	double _f_des[NOF];					///< desired force

	double _R[9];						///< body(palm) orientation

	double _tau_task[NOF][NOJ];			///< computed joint torque to do task

	double x_d_object;
	double y_d_object;
	double z_d_object;

	double x_move;
	double y_move;
	double z_move;

	double _x_des[NOF];					///< desired x position in cartesian coordinate for the center of each finger tip
	double _y_des[NOF];					///< desired y position in cartesian coordinate for the center of each finger tip
	double _z_des[NOF];					///< desired z position in cartesian coordinate for the center of each finger tip

	double _set_x_des[NOF];				// Set by user in MoveFingerTip 
	double _set_y_des[NOF];				// Set by user in MoveFingerTip
	double _set_z_des[NOF];				// Set by user in MoveFingerTip

	// sin: S{joint_index}[finger_index]
	// cos: C{joint_index}[finger_index]
	double S0[NOF], S1[NOF], S2[NOF], S3[NOF], S12[NOF], S123[NOF], S23[NOF];
	double C0[NOF], C1[NOF], C2[NOF], C3[NOF], C12[NOF], C123[NOF], C23[NOF];
	
	// gripper geometries and mass
	double _mass[NOF][NOJ];				///< link mass
	double _link[NOF][NOJ];				///< link length
	double _baseX;
	double _baseY;
};



/**
* BGrip9DOF class.
* @brief Allegro Gripper grasping algorithm.
* @author Jihoon Bae, Seay Yi
*/
class BGRIPEXPORT BGrip9DOF : public BGrip
{
public:
	BGrip9DOF();
	~BGrip9DOF(void);

protected:
	virtual void SetGains(int motionType);
	virtual void SolveFK();
	virtual void CalculateJacobian();
	virtual void CalculateGravity();
	virtual void CalculateGravityEx();

	virtual void Motion_Ready();
	virtual void Motion_GravityComp();
	virtual void Motion_Grasp();
	virtual void Motion_JointPD();
};


/**
* BGrip11DOF class.
* @brief Allegro Gripper grasping algorithm.
* @author Jihoon Bae, Seay Yi
*/
class BGRIPEXPORT BGrip11DOF : public BGrip
{
public:
	BGrip11DOF();
	~BGrip11DOF(void);

protected:
	virtual void SetGains(int motionType);
	virtual void SolveFK();
	virtual void CalculateJacobian();
	virtual void CalculateGravity();
	virtual void CalculateGravityEx();

	virtual void Motion_Ready();
	virtual void Motion_GravityComp();
	virtual void Motion_Grasp();
	virtual void Motion_JointPD();
};




BGRIP_EXTERN_C_BEGIN

/**
 * @brief Creates an instance of Allegro Grippper control algorithm.
 */
BGRIPEXPORT BGrip* bgCreateGripper9DOF();

/**
 * @brief Creates an instance of Allegro Hand control algorithm. It is for right hand.
 */
BGRIPEXPORT BGrip* bgCreateGripper11DOF();

BGRIP_EXTERN_C_END

#endif // __BGRIP_H__