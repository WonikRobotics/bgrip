#include "BGrip.h"
#include "memory.h"
#include "math.h"
#include "stdio.h"

BGrip9DOF::BGrip9DOF()
	: BGrip(eGripperType_9DOF)
{
	_mass[0][0] = 0.0;
	_mass[0][1] = 0.16;
	_mass[0][2] = 0.16;
	_mass[1][0] = 0.0;
	_mass[1][1] = 0.16;
	_mass[1][2] = 0.16;
	_mass[2][0] = 0.0;
	_mass[2][1] = 0.16;
	_mass[2][2] = 0.16;

	_link[0][0] = 0.01;
	_link[0][1] = 0.054;
	_link[0][2] = 0.068;
	_link[1][0] = 0.01;
	_link[1][1] = 0.054;
	_link[1][2] = 0.068;
	_link[2][0] = 0.01;
	_link[2][1] = 0.054;
	_link[2][2] = 0.068;

	_baseX = 0.03;
	_baseY = 0.035;
}

BGrip9DOF::~BGrip9DOF(void)
{
}

void BGrip9DOF::SetGains(int motionType)
{
	switch (motionType)
	{
	case eMotionType_READY:
		_kp[0][0] = _kp[1][0] = _kp[2][0] = 500; //500 40
		_kp[0][1] = _kp[1][1] = _kp[2][1] = 800;
		_kp[0][2] = _kp[1][2] = _kp[2][2] = 500;
		_kp[0][3] = _kp[1][3] = _kp[2][3] = 0;
		_kd[0][0] = _kd[1][0] = _kd[2][0] = 25;   // this finger has really low friction at the first joint on BR014
		_kd[0][1] = _kd[1][1] = _kd[2][1] = 50;   // make sure these values look good on all hands. likely change them 
		_kd[0][2] = _kd[1][2] = _kd[2][2] = 40;   // the to match the ones below.
		_kd[0][3] = _kd[1][3] = _kd[2][3] = 0;
		break;

	case eMotionType_GRASP:
		break;

	case eMotionType_JOINT_PD:
		_kp[0][0] = _kp[1][0] = _kp[2][0] = 500; //500 40
		_kp[0][1] = _kp[1][1] = _kp[2][1] = 800;
		_kp[0][2] = _kp[1][2] = _kp[2][2] = 500;
		_kp[0][3] = _kp[1][3] = _kp[2][3] = 0;
		_kd[0][0] = _kd[1][0] = _kd[2][0] = 25;   // this finger has really low friction at the first joint on BR014
		_kd[0][1] = _kd[1][1] = _kd[2][1] = 50;   // make sure these values look good on all hands. likely change them 
		_kd[0][2] = _kd[1][2] = _kd[2][2] = 40;   // the to match the ones below.
		_kd[0][3] = _kd[1][3] = _kd[2][3] = 0;
		break;

	case eMotionType_GRAVITY_COMP:
		_kp[0][0] = _kp[1][0] = _kp[2][0] = 80; // was 120 orignially
		_kp[0][1] = _kp[1][1] = _kp[2][1] = 90;
		_kp[0][2] = _kp[1][2] = _kp[2][2] = 80;
		_kp[0][3] = _kp[1][3] = _kp[2][3] = 0;
		_kd[0][0] = _kd[1][0] = _kd[2][0] = sqrt(_kp[0][0])*0.004*1.0;
		_kd[0][1] = _kd[1][1] = _kd[2][1] = sqrt(_kp[0][1])*0.004*2.0;
		_kd[0][2] = _kd[1][2] = _kd[2][2] = sqrt(_kp[0][2])*0.004*1.0;
		_kd[0][3] = _kd[1][3] = _kd[2][3] = 0;
		break;

	default:
		BGrip::SetGains(motionType);
		return;
	}
}

void BGrip9DOF::SolveFK()
{
	for (int i = 0; i<NOF; i++)
	{
		_x_pre[i] = _x[i];
		_y_pre[i] = _y[i];
		_z_pre[i] = _z[i];
		_x_filtered_pre[i] = _x_filtered[i];
		_y_filtered_pre[i] = _y_filtered[i];
		_z_filtered_pre[i] = _z_filtered[i];
	}


	//The forward kinematics of each finger.
	
	_x[FI_I] =  _baseX - _link[FI_I][2] * (C0[FI_I] * C1[FI_I] * S2[FI_I] + C0[FI_I] * C2[FI_I] * S1[FI_I]) - _link[FI_I][1] * C0[FI_I] * S1[FI_I];
	_y[FI_I] = -_baseY - _link[FI_I][2] * (C1[FI_I] * S0[FI_I] * S2[FI_I] + C2[FI_I] * S0[FI_I] * S1[FI_I]) - _link[FI_I][1] * S0[FI_I] * S1[FI_I];
	_z[FI_I] = _link[FI_I][2] * (C1[FI_I] * C2[FI_I] - S1[FI_I] * S2[FI_I]) + _link[FI_I][1] * C1[FI_I];

	_x[FI_M] = _baseX - _link[FI_M][2] * (C0[FI_M] * C1[FI_M] * S2[FI_M] + C0[FI_M] * C2[FI_M] * S1[FI_M]) - _link[FI_M][1] * C0[FI_M] * S1[FI_M];
	_y[FI_M] = _baseY - _link[FI_M][2] * (C1[FI_M] * S0[FI_M] * S2[FI_M] + C2[FI_M] * S0[FI_M] * S1[FI_M]) - _link[FI_M][1] * S0[FI_M] * S1[FI_M];
	_z[FI_M] = _link[FI_M][2] * (C1[FI_M] * C2[FI_M] - S1[FI_M] * S2[FI_M]) + _link[FI_M][1] * C1[FI_M];

	_x[FI_T] = _link[FI_T][2] *(C0[FI_T] * C1[FI_T] * S2[FI_T] + C0[FI_T] * C2[FI_T] * S1[FI_T]) + _link[FI_T][1] * C0[FI_T] * S1[FI_T] - _baseX;
	_y[FI_T] = _link[FI_T][2] *(C1[FI_T] * S0[FI_T] * S2[FI_T] + C2[FI_T] * S0[FI_T] * S1[FI_T]) + _link[FI_T][1] * S0[FI_T] * S1[FI_T];
	_z[FI_T] = _link[FI_T][2] *(C1[FI_T] * C2[FI_T] - S1[FI_T] * S2[FI_T]) + _link[FI_T][1] * C1[FI_T];


	
	for (int i = 0; i<NOF; i++)
	{
		_x_filtered[i] = (0.6f*_x_filtered[i]) + (0.198f*_x_pre[i]) + (0.198f*_x[i]);
		_y_filtered[i] = (0.6f*_y_filtered[i]) + (0.198f*_y_pre[i]) + (0.198f*_y[i]);
		_z_filtered[i] = (0.6f*_z_filtered[i]) + (0.198f*_z_pre[i]) + (0.198f*_z[i]);

		_xdot_pre[i] = _xdot[i];
		_ydot_pre[i] = _ydot[i];
		_zdot_pre[i] = _zdot[i];

		_xdot[i] = (_x_filtered[i] - _x_filtered_pre[i]) / _dT;
		_ydot[i] = (_y_filtered[i] - _y_filtered_pre[i]) / _dT;
		_zdot[i] = (_z_filtered[i] - _z_filtered_pre[i]) / _dT;

		_xdot_filtered[i] = (0.6f*_xdot_filtered[i]) + (0.198f*_xdot_pre[i]) + (0.198f*_xdot[i]);
		_ydot_filtered[i] = (0.6f*_ydot_filtered[i]) + (0.198f*_ydot_pre[i]) + (0.198f*_ydot[i]);
		_zdot_filtered[i] = (0.6f*_zdot_filtered[i]) + (0.198f*_zdot_pre[i]) + (0.198f*_zdot[i]);
	}
}

void BGrip9DOF::CalculateJacobian()
{
	// jacobian
	// index finger
	_J[FI_I][0][0] =  _link[FI_I][2] *(C1[FI_I] * S0[FI_I] * S2[FI_I] + C2[FI_I] * S0[FI_I] * S1[FI_I]) + _link[FI_I][1] * S0[FI_I] * S1[FI_I];
	_J[FI_I][0][1] = -_link[FI_I][2] *(C0[FI_I] * C1[FI_I] * C2[FI_I] - C0[FI_I] * S1[FI_I] * S2[FI_I]) - _link[FI_I][1] * C0[FI_I] * C1[FI_I];
	_J[FI_I][0][2] = -_link[FI_I][2] *(C0[FI_I] * C1[FI_I] * C2[FI_I] - C0[FI_I] * S1[FI_I] * S2[FI_I]);

	_J[FI_I][1][0] = -_link[FI_I][2] *(C0[FI_I] * C1[FI_I] * S2[FI_I] + C0[FI_I] * C2[FI_I] * S1[FI_I]) - _link[FI_I][1] * C0[FI_I] * S1[FI_I];
	_J[FI_I][1][1] =  _link[FI_I][2] *(S0[FI_I] * S1[FI_I] * S2[FI_I] - C1[FI_I] * C2[FI_I] * S0[FI_I]) - _link[FI_I][1] * C1[FI_I] * S0[FI_I];
	_J[FI_I][1][2] =  _link[FI_I][2] *(S0[FI_I] * S1[FI_I] * S2[FI_I] - C1[FI_I] * C2[FI_I] * S0[FI_I]);

	_J[FI_I][2][0] =  0.0f;
	_J[FI_I][2][1] = -_link[FI_I][2] *(C1[FI_I] * S2[FI_I] + C2[FI_I] * S1[FI_I]) - _link[FI_I][1] * S1[FI_I];
	_J[FI_I][2][2] = -_link[FI_I][2] *(C1[FI_I] * S2[FI_I] + C2[FI_I] * S1[FI_I]);

	// middle Finger
	_J[FI_M][0][0] =  _link[FI_M][2] *(C1[FI_M] * S0[FI_M] * S2[FI_M] + C2[FI_M] * S0[FI_M] * S1[FI_M]) + _link[FI_M][1] * S0[FI_M] * S1[FI_M];
	_J[FI_M][0][1] = -_link[FI_M][2] *(C0[FI_M] * C1[FI_M] * C2[FI_M] - C0[FI_M] * S1[FI_M] * S2[FI_M]) - _link[FI_M][1] * C0[FI_M] * C1[FI_M];
	_J[FI_M][0][2] = -_link[FI_M][2] *(C0[FI_M] * C1[FI_M] * C2[FI_M] - C0[FI_M] * S1[FI_M] * S2[FI_M]);

	_J[FI_M][1][0] = -_link[FI_M][2] *(C0[FI_M] * C1[FI_M] * S2[FI_M] + C0[FI_M] * C2[FI_M] * S1[FI_M]) - _link[FI_M][1] * C0[FI_M] * S1[FI_M];
	_J[FI_M][1][1] =  _link[FI_M][2] *(S0[FI_M] * S1[FI_M] * S2[FI_M] - C1[FI_M] * C2[FI_M] * S0[FI_M]) - _link[FI_M][1] * C1[FI_M] * S0[FI_M];
	_J[FI_M][1][2] =  _link[FI_M][2] *(S0[FI_M] * S1[FI_M] * S2[FI_M] - C1[FI_M] * C2[FI_M] * S0[FI_M]);

	_J[FI_M][2][0] =  0.0f;
	_J[FI_M][2][1] = -_link[FI_M][2] *(C1[FI_M] * S2[FI_M] + C2[FI_M] * S1[FI_M]) - _link[FI_M][1] * S1[FI_M];
	_J[FI_M][2][2] = -_link[FI_M][2] *(C1[FI_M] * S2[FI_M] + C2[FI_M] * S1[FI_M]);
	
	// thumb
	_J[FI_T][0][0] = -_link[FI_T][2] *(C1[FI_T] * S0[FI_T] * S2[FI_T] + C2[FI_T] * S0[FI_T] * S1[FI_T]) - _link[FI_T][1] * S0[FI_T] * S1[FI_T];
	_J[FI_T][0][1] =  _link[FI_T][2] *(C0[FI_T] * C1[FI_T] * C2[FI_T] - C0[FI_T] * S1[FI_T] * S2[FI_T]) + _link[FI_T][1] * C0[FI_T] * C1[FI_T];
	_J[FI_T][0][2] =  _link[FI_T][2] *(C0[FI_T] * C1[FI_T] * C2[FI_T] - C0[FI_T] * S1[FI_T] * S2[FI_T]);

	_J[FI_T][1][0] =  _link[FI_T][2] *(C0[FI_T] * C1[FI_T] * S2[FI_T] + C0[FI_T] * C2[FI_T] * S1[FI_T]) + _link[FI_T][1] * C0[FI_T] * S1[FI_T];
	_J[FI_T][1][1] = -_link[FI_T][2] *(S0[FI_T] * S1[FI_T] * S2[FI_T] - C1[FI_T] * C2[FI_T] * S0[FI_T]) + _link[FI_T][1] * C1[FI_T] * S0[FI_T];
	_J[FI_T][1][2] = -_link[FI_T][2] *(S0[FI_T] * S1[FI_T] * S2[FI_T] - C1[FI_T] * C2[FI_T] * S0[FI_T]);

	_J[FI_T][2][0] =  0.0f;
	_J[FI_T][2][1] = -_link[FI_T][2] *(C1[FI_T] * S2[FI_T] + C2[FI_T] * S1[FI_T]) - _link[FI_T][1] * S1[FI_T];
	_J[FI_T][2][2] = -_link[FI_T][2] *(C1[FI_T] * S2[FI_T] + C2[FI_T] * S1[FI_T]);
}

void BGrip9DOF::CalculateGravity()
{
	//double _G[NOF][NOJ];
	double g_ = 9.81;

	for (int fi = 0; fi < NOF; fi++)
	{
		_G[fi][0] = 0.0;
		_G[fi][1] = -(g_ * _mass[fi][1] * _link[fi][1] * 0.5 * S1[fi]) 
			        -(g_ * _mass[fi][2] * (_link[fi][2] * 0.5 * S12[fi] + _link[fi][1] * S1[fi]));
		_G[fi][2] = -(g_ * _mass[fi][2] * _link[fi][2] * 0.5 * S12[fi]);
	}
}

void BGrip9DOF::CalculateGravityEx()
{
}

void BGrip9DOF::Motion_Ready()
{
	switch (_graspMode)
	{
	case eGraspMode_GROPED:

		//index
		_q_des[FI_I][0] = -30 * DEG2RAD;
		_q_des[FI_I][1] = (-30 - _openSize) * DEG2RAD;
		_q_des[FI_I][2] = (60 + 0.5*_openSize) * DEG2RAD;
		//middle
		_q_des[FI_M][0] = 30 * DEG2RAD;
		_q_des[FI_M][1] = (-30 - _openSize) * DEG2RAD;
		_q_des[FI_M][2] = (60 + 0.5*_openSize) * DEG2RAD;
		//Thumb
		_q_des[FI_T][0] = 0 * DEG2RAD;
		_q_des[FI_T][1] = (-30 - _openSize) * DEG2RAD;
		_q_des[FI_T][2] = (60 + 0.5*_openSize) * DEG2RAD;
		break;

	case eGraspMode_PARALLEL:

		//index
		_q_des[FI_I][0] = 0 * DEG2RAD;
		_q_des[FI_I][1] = (-30 - _openSize) * DEG2RAD;
		_q_des[FI_I][2] = (60 + 0.5*_openSize) * DEG2RAD;
		//middle
		_q_des[FI_M][0] = 0 * DEG2RAD;
		_q_des[FI_M][1] = (-30 - _openSize) * DEG2RAD;
		_q_des[FI_M][2] = (60 + 0.5*_openSize) * DEG2RAD;
		//Thumb
		_q_des[FI_T][0] = 0 * DEG2RAD;
		_q_des[FI_T][1] = (-30 - _openSize) * DEG2RAD;
		_q_des[FI_T][2] = (60 + 0.5*_openSize) * DEG2RAD;
		break;

	case eGraspMode_ENVELOP:

		//index
		_q_des[FI_I][0] = 0 * DEG2RAD;
		_q_des[FI_I][1] = -30 * DEG2RAD;
		_q_des[FI_I][2] = 60 * DEG2RAD;
		//middle
		_q_des[FI_M][0] = 0 * DEG2RAD;
		_q_des[FI_M][1] = -30 * DEG2RAD;
		_q_des[FI_M][2] = 60 * DEG2RAD;
		//Thumb
		_q_des[FI_T][0] = 0 * DEG2RAD;
		_q_des[FI_T][1] = -30 * DEG2RAD;
		_q_des[FI_T][2] = 60 * DEG2RAD;
		break;

	case eGraspMode_PINCH_INDEX:

		//index
		_q_des[FI_I][0] = -30 * DEG2RAD;
		_q_des[FI_I][1] = (-20 - _openSize) * DEG2RAD;
		_q_des[FI_I][2] = (55 + 0.5*_openSize) * DEG2RAD;
		//middle
		_q_des[FI_M][0] = 30 * DEG2RAD;
		_q_des[FI_M][1] = -30 * DEG2RAD;
		_q_des[FI_M][2] = 0 * DEG2RAD;
		//Thumb
		_q_des[FI_T][0] = -30 * DEG2RAD;
		_q_des[FI_T][1] = (-20 - _openSize) * DEG2RAD;
		_q_des[FI_T][2] = (55 + 0.5*_openSize) * DEG2RAD;
		break;

	case eGraspMode_PINCH_MIDDLE:

		//index
		_q_des[FI_I][0] = -30 * DEG2RAD;
		_q_des[FI_I][1] = -30 * DEG2RAD;
		_q_des[FI_I][2] = 0 * DEG2RAD;
		//middle
		_q_des[FI_M][0] = 30 * DEG2RAD;
		_q_des[FI_M][1] = (-20 - _openSize) * DEG2RAD;
		_q_des[FI_M][2] = (55 + 0.5*_openSize) * DEG2RAD; 
		//Thumb
		_q_des[FI_T][0] = 30 * DEG2RAD;
		_q_des[FI_T][1] = (-20 - _openSize) * DEG2RAD;
		_q_des[FI_T][2] = (55 + 0.5*_openSize) * DEG2RAD;
		break;

	case eGraspMode_PINCH_PEG:

		//index
		_q_des[FI_I][0] = -92 * DEG2RAD;
		_q_des[FI_I][1] = (-30 - _openSize) * DEG2RAD;
		_q_des[FI_I][2] = 60 * DEG2RAD;
		//middle
		_q_des[FI_M][0] = 91 * DEG2RAD;
		_q_des[FI_M][1] = (-30 - _openSize) * DEG2RAD;
		_q_des[FI_M][2] = 60 * DEG2RAD;
		//Thumb
		_q_des[FI_T][0] = 0 * DEG2RAD;
		_q_des[FI_T][1] = -60 * DEG2RAD;
		_q_des[FI_T][2] = 85 * DEG2RAD;
		break;

	default:
		break;
	}

	Motion_JointPD();
}

void BGrip9DOF::Motion_GravityComp()
{
	//memcpy(_tau_des, _G, SIZEOF_VARRAY);
	memset(_tau_task, 0, SIZEOF_VARRAY);
	
	/*for (int fi = 0; fi < NOF; fi++)
	{
		_x_des[fi] = _x[fi];
		_y_des[fi] = _y[fi];
		_z_des[fi] = _z[fi];
	}*/
}

void BGrip9DOF::Motion_Grasp()
{
	double t_TaskDamping[NOF][NOJ];
	double t_Position[NOF][NOJ];
	double t_Pinching[NOF][NOJ];
	double t_Position_Object[NOF][NOJ];

	double KT[3];
	double Cg[3];
	double Cgd[3];
	
	double F[NOF], Fx[NOF], Fy[NOF], Fz[NOF], Fdx[NOF], Fdy[NOF], Fdz[NOF];
	double ag[3];

	/*double IndexCg[3];
	double MiddleCg[3];
	double PegCg[3];
	double F1[3], F2[3], F3[3];
	double Fd1[3], Fd2[3], Fd3[3];
	double Index_F1[3];
	double Middle_F1[3];
	double Peg_F1[3];*/

	int i, j;

	KT[0] = 200;
	KT[1] = 200;
	KT[2] = 200;


	



	switch (_graspMode)
	{
	case eGraspMode_GROPED:

		if (_curT < 0.5)
		{
			//index
			_q_des[FI_I][0] = -30 * DEG2RAD;
			_q_des[FI_I][1] = -20 * DEG2RAD;
			_q_des[FI_I][2] =  65 * DEG2RAD;
			//middle
			_q_des[FI_M][0] =  30 * DEG2RAD;
			_q_des[FI_M][1] = -20 * DEG2RAD;
			_q_des[FI_M][2] =  65 * DEG2RAD;
			//Thumb
			_q_des[FI_T][0] =   0 * DEG2RAD;
			_q_des[FI_T][1] = -20 * DEG2RAD;
			_q_des[FI_T][2] =  65 * DEG2RAD;

			Motion_JointPD();
		}
		else
		{
			Cg[0] = (_x[0] + _x[1] + _x[2]) / 3;
			Cg[1] = (_y[0] + _y[1] + _y[2]) / 3;
			Cg[2] = (_z[0] + _z[1] + _z[2]) / 3;

			Cgd[0] = Cg[0];
			Cgd[1] = Cg[1];
			Cgd[2] = Cg[2];

			for (int fi = 0; fi < 3; fi++) 
			{
				F[fi] = sqrt((Cg[0] - _x[fi])*(Cg[0] - _x[fi]) + (Cg[1] - _y[fi])*(Cg[1] - _y[fi]) + (Cg[2] - _z[fi])*(Cg[2] - _z[fi]));
				Fx[fi] = (Cg[0] - _x[fi]) / F[fi];
				Fy[fi] = (Cg[1] - _y[fi]) / F[fi];
				Fz[fi] = (Cg[2] - _z[fi]) / F[fi];

				Fdx[fi] = Fx[fi] + KT[0] * (Cgd[0] - Cg[0]);
				Fdy[fi] = Fy[fi] + KT[1] * (Cgd[1] - Cg[1]);
				Fdz[fi] = Fz[fi] + KT[2] * (Cgd[2] - Cg[2]);
			}
						
			ag[FI_I] = _f_des[FI_I];
			ag[FI_M] = ag[FI_I] * sqrt((Cgd[0] - _x[FI_I])*(Cgd[0] - _x[FI_I]) + (Cgd[1] - _y[FI_I])*(Cgd[1] - _y[FI_I]) + (Cgd[2] - _z[FI_I])*(Cgd[2] - _z[FI_I])) / sqrt((Cgd[0] - _x[FI_M])*(Cgd[0] - _x[FI_M]) + (Cgd[1] - _y[FI_M])*(Cgd[1] - _y[FI_M]) + (Cgd[2] - _z[FI_M])*(Cgd[2] - _z[FI_M]));
			ag[FI_T] = sqrt(((ag[FI_I] * Fx[FI_I] + ag[FI_I] * Fy[FI_I] + ag[FI_I] * Fz[FI_I]) + (ag[FI_M] * Fx[FI_M] + ag[FI_M] * Fy[FI_M] + ag[FI_M] * Fz[FI_M]))*((ag[FI_I] * Fx[FI_I] + ag[FI_I] * Fy[FI_I] + ag[FI_I] * Fz[FI_I]) + (ag[FI_M] * Fx[FI_M] + ag[FI_M] * Fy[FI_M] + ag[FI_M] * Fz[FI_M])));

			for (int fi = 0; fi < 3; fi++)
			{
				for (int ji = 0; ji < 3; ji++)
				{
					_tau_task[fi][ji] = ag[fi] * (_J[fi][0][ji] * Fdx[fi] + _J[fi][1][ji] * Fdy[fi] + _J[fi][2][ji] * Fdz[fi]);
				}
			}
		}
		break;

	case eGraspMode_PARALLEL:

		//index
		_q_des[FI_I][0] =   0 * DEG2RAD;
		_q_des[FI_I][1] = -20 * DEG2RAD;
		_q_des[FI_I][2] =  65 * DEG2RAD;
		//middle
		_q_des[FI_M][0] =   0 * DEG2RAD;
		_q_des[FI_M][1] = -20 * DEG2RAD;
		_q_des[FI_M][2] =  65 * DEG2RAD;
		//Thumb
		_q_des[FI_T][0] =   0 * DEG2RAD;
		_q_des[FI_T][1] = -20 * DEG2RAD;
		_q_des[FI_T][2] =  65 * DEG2RAD;

		Motion_JointPD();
		
		if (_curT > 0.5)
		{
			Cg[0] = (_x[0] + _x[1] + _x[2]) / 3;
			Cg[1] = (_y[0] + _y[1] + _y[2]) / 3;
			Cg[2] = (_z[0] + _z[1] + _z[2]) / 3;

			Cgd[0] = Cg[0];
			Cgd[1] = Cg[1];
			Cgd[2] = Cg[2];

			for (int fi = 0; fi < 3; fi++)
			{
				F[fi] = sqrt((Cg[0] - _x[fi])*(Cg[0] - _x[fi]) + (Cg[1] - _y[fi])*(Cg[1] - _y[fi]) + (Cg[2] - _z[fi])*(Cg[2] - _z[fi]));
				Fx[fi] = (Cg[0] - _x[fi]) / F[fi];
				Fy[fi] = 0;// (Cg[1] - _y[fi]) / F[fi];
				Fz[fi] = (Cg[2] - _z[fi]) / F[fi];

				Fdx[fi] = Fx[fi] + KT[0] * (Cgd[0] - Cg[0]);
				Fdy[fi] = 0;// Fy[fi] + KT[1] * (Cgd[1] - Cg[1]);
				Fdz[fi] = Fz[fi] + KT[2] * (Cgd[2] - Cg[2]);
			}

			ag[FI_I] = _f_des[FI_I];
			ag[FI_M] = _f_des[FI_M]; //ag[FI_I] * sqrt((Cgd[0] - _x[FI_I])*(Cgd[0] - _x[FI_I]) + (Cgd[1] - _y[FI_I])*(Cgd[1] - _y[FI_I]) + (Cgd[2] - _z[FI_I])*(Cgd[2] - _z[FI_I])) / sqrt((Cgd[0] - _x[FI_M])*(Cgd[0] - _x[FI_M]) + (Cgd[1] - _y[FI_M])*(Cgd[1] - _y[FI_M]) + (Cgd[2] - _z[FI_M])*(Cgd[2] - _z[FI_M]));
			ag[FI_T] = _f_des[FI_T]; //sqrt(((ag[FI_I] * Fx[FI_I] + ag[FI_I] * Fy[FI_I] + ag[FI_I] * Fz[FI_I]) + (ag[FI_M] * Fx[FI_M] + ag[FI_M] * Fy[FI_M] + ag[FI_M] * Fz[FI_M]))*((ag[FI_I] * Fx[FI_I] + ag[FI_I] * Fy[FI_I] + ag[FI_I] * Fz[FI_I]) + (ag[FI_M] * Fx[FI_M] + ag[FI_M] * Fy[FI_M] + ag[FI_M] * Fz[FI_M])));

			for (int fi = 0; fi < 3; fi++)
			{
				for (int ji = 1; ji < 3; ji++)
				{
					_tau_task[fi][ji] = ag[fi] * (_J[fi][0][ji] * Fdx[fi] + _J[fi][1][ji] * Fdy[fi] + _J[fi][2][ji] * Fdz[fi]);
				}
			}
		}
		break;

	case eGraspMode_PINCH_INDEX:

		//index
		_q_des[FI_I][0] = -30 * DEG2RAD;
		_q_des[FI_I][1] = -20 * DEG2RAD;
		_q_des[FI_I][2] = 60 * DEG2RAD;
		//middle
		_q_des[FI_M][0] = _q[FI_M][0];
		_q_des[FI_M][1] = _q[FI_M][1];
		_q_des[FI_M][2] = _q[FI_M][2];
		//Thumb
		_q_des[FI_T][0] = -30 * DEG2RAD;
		_q_des[FI_T][1] = -20 * DEG2RAD;
		_q_des[FI_T][2] = 75 * DEG2RAD;

		Motion_JointPD();

		if (_curT > 0.5)
		{
			Cg[0] = (_x[FI_I] + _x[FI_T]) / 2;
			Cg[1] = (_y[FI_I] + _y[FI_T]) / 2;
			Cg[2] = (_z[FI_I] + _z[FI_T]) / 2;

			Cgd[0] = Cg[0];
			Cgd[1] = Cg[1];
			Cgd[2] = Cg[2];

			for (int fi = 0; fi < 3; fi++)
			{
				F[fi] = sqrt((Cg[0] - _x[fi])*(Cg[0] - _x[fi]) + (Cg[1] - _y[fi])*(Cg[1] - _y[fi]) + (Cg[2] - _z[fi])*(Cg[2] - _z[fi]));
				Fx[fi] = (Cg[0] - _x[fi]) / F[fi];
				Fy[fi] = (Cg[1] - _y[fi]) / F[fi];
				Fz[fi] = (Cg[2] - _z[fi]) / F[fi];

				Fdx[fi] = Fx[fi] + KT[0] * (Cgd[0] - Cg[0]);
				Fdy[fi] = Fy[fi] + KT[1] * (Cgd[1] - Cg[1]);
				Fdz[fi] = Fz[fi] + KT[2] * (Cgd[2] - Cg[2]);
			}
			
			ag[FI_I] = _f_des[FI_I];
			ag[FI_M] = _f_des[FI_M]; //ag[FI_I] * sqrt((Cgd[0] - _x[FI_I])*(Cgd[0] - _x[FI_I]) + (Cgd[1] - _y[FI_I])*(Cgd[1] - _y[FI_I]) + (Cgd[2] - _z[FI_I])*(Cgd[2] - _z[FI_I])) / sqrt((Cgd[0] - _x[FI_M])*(Cgd[0] - _x[FI_M]) + (Cgd[1] - _y[FI_M])*(Cgd[1] - _y[FI_M]) + (Cgd[2] - _z[FI_M])*(Cgd[2] - _z[FI_M]));
			ag[FI_T] = _f_des[FI_T]; //sqrt(((ag[FI_I] * Fx[FI_I] + ag[FI_I] * Fy[FI_I] + ag[FI_I] * Fz[FI_I]) + (ag[FI_M] * Fx[FI_M] + ag[FI_M] * Fy[FI_M] + ag[FI_M] * Fz[FI_M]))*((ag[FI_I] * Fx[FI_I] + ag[FI_I] * Fy[FI_I] + ag[FI_I] * Fz[FI_I]) + (ag[FI_M] * Fx[FI_M] + ag[FI_M] * Fy[FI_M] + ag[FI_M] * Fz[FI_M])));

			for (int fi = 0; fi < 3; fi++)
			{
				if (fi == FI_M)
					continue;

				for (int ji = 1; ji < 3; ji++)
				{
					_tau_task[fi][ji] = ag[fi] * (_J[fi][0][ji] * Fdx[fi] + _J[fi][1][ji] * Fdy[fi] + _J[fi][2][ji] * Fdz[fi]);
				}
			}
		}
		break;

	case eGraspMode_PINCH_MIDDLE:

		//index
		_q_des[FI_I][0] = _q[FI_I][0];
		_q_des[FI_I][1] = _q[FI_I][1];
		_q_des[FI_I][2] = _q[FI_I][2];
		//middle
		_q_des[FI_M][0] = 30 * DEG2RAD;
		_q_des[FI_M][1] = -20 * DEG2RAD;
		_q_des[FI_M][2] = 60 * DEG2RAD;
		//Thumb
		_q_des[FI_T][0] = 30 * DEG2RAD;
		_q_des[FI_T][1] = -20 * DEG2RAD;
		_q_des[FI_T][2] = 60 * DEG2RAD;

		Motion_JointPD();

		if (_curT > 0.5)
		{
			Cg[0] = (_x[FI_M] + _x[FI_T]) / 2;
			Cg[1] = (_y[FI_M] + _y[FI_T]) / 2;
			Cg[2] = (_z[FI_M] + _z[FI_T]) / 2;

			Cgd[0] = Cg[0];
			Cgd[1] = Cg[1];
			Cgd[2] = Cg[2];

			for (int fi = 0; fi < 3; fi++)
			{
				F[fi] = sqrt((Cg[0] - _x[fi])*(Cg[0] - _x[fi]) + (Cg[1] - _y[fi])*(Cg[1] - _y[fi]) + (Cg[2] - _z[fi])*(Cg[2] - _z[fi]));
				Fx[fi] = (Cg[0] - _x[fi]) / F[fi];
				Fy[fi] = (Cg[1] - _y[fi]) / F[fi];
				Fz[fi] = (Cg[2] - _z[fi]) / F[fi];

				Fdx[fi] = Fx[fi] + KT[0] * (Cgd[0] - Cg[0]);
				Fdy[fi] = Fy[fi] + KT[1] * (Cgd[1] - Cg[1]);
				Fdz[fi] = Fz[fi] + KT[2] * (Cgd[2] - Cg[2]);
			}

			ag[FI_I] = _f_des[FI_I];
			ag[FI_M] = _f_des[FI_M]; //ag[FI_I] * sqrt((Cgd[0] - _x[FI_I])*(Cgd[0] - _x[FI_I]) + (Cgd[1] - _y[FI_I])*(Cgd[1] - _y[FI_I]) + (Cgd[2] - _z[FI_I])*(Cgd[2] - _z[FI_I])) / sqrt((Cgd[0] - _x[FI_M])*(Cgd[0] - _x[FI_M]) + (Cgd[1] - _y[FI_M])*(Cgd[1] - _y[FI_M]) + (Cgd[2] - _z[FI_M])*(Cgd[2] - _z[FI_M]));
			ag[FI_T] = _f_des[FI_T]; //sqrt(((ag[FI_I] * Fx[FI_I] + ag[FI_I] * Fy[FI_I] + ag[FI_I] * Fz[FI_I]) + (ag[FI_M] * Fx[FI_M] + ag[FI_M] * Fy[FI_M] + ag[FI_M] * Fz[FI_M]))*((ag[FI_I] * Fx[FI_I] + ag[FI_I] * Fy[FI_I] + ag[FI_I] * Fz[FI_I]) + (ag[FI_M] * Fx[FI_M] + ag[FI_M] * Fy[FI_M] + ag[FI_M] * Fz[FI_M])));

			for (int fi = 0; fi < 3; fi++)
			{
				if (fi == FI_I)
					continue;

				for (int ji = 1; ji < 3; ji++)
				{
					_tau_task[fi][ji] = ag[fi] * (_J[fi][0][ji] * Fdx[fi] + _J[fi][1][ji] * Fdy[fi] + _J[fi][2][ji] * Fdz[fi]);
				}
			}
		}
		break;

	case eGraspMode_PINCH_PEG:

		//index
		_q_des[FI_I][0] = -90 * DEG2RAD;
		_q_des[FI_I][1] = -20 * DEG2RAD;
		_q_des[FI_I][2] = 70 * DEG2RAD;
		//middle
		_q_des[FI_M][0] = 90 * DEG2RAD;
		_q_des[FI_M][1] = -20 * DEG2RAD;
		_q_des[FI_M][2] = 70 * DEG2RAD;
		//Thumb
		_q_des[FI_T][0] = 0 * DEG2RAD;
		_q_des[FI_T][1] = -60 * DEG2RAD;
		_q_des[FI_T][2] = 95 * DEG2RAD;

		Motion_JointPD();

		if (_curT > 0.5)
		{
			Cg[0] = (_x[FI_I] + _x[FI_M]) / 2;
			Cg[1] = (_y[FI_I] + _y[FI_M]) / 2;
			Cg[2] = (_z[FI_I] + _z[FI_M]) / 2;

			Cgd[0] = Cg[0];
			Cgd[1] = Cg[1];
			Cgd[2] = Cg[2];

			for (int fi = 0; fi < 3; fi++)
			{
				F[fi] = sqrt((Cg[0] - _x[fi])*(Cg[0] - _x[fi]) + (Cg[1] - _y[fi])*(Cg[1] - _y[fi]) + (Cg[2] - _z[fi])*(Cg[2] - _z[fi]));
				Fx[fi] = (Cg[0] - _x[fi]) / F[fi];
				Fy[fi] = (Cg[1] - _y[fi]) / F[fi];
				Fz[fi] = (Cg[2] - _z[fi]) / F[fi];

				Fdx[fi] = Fx[fi] + KT[0] * (Cgd[0] - Cg[0]);
				Fdy[fi] = Fy[fi] + KT[1] * (Cgd[1] - Cg[1]);
				Fdz[fi] = Fz[fi] + KT[2] * (Cgd[2] - Cg[2]);
			}

			ag[FI_I] = _f_des[FI_I];
			ag[FI_M] = _f_des[FI_M]; //ag[FI_I] * sqrt((Cgd[0] - _x[FI_I])*(Cgd[0] - _x[FI_I]) + (Cgd[1] - _y[FI_I])*(Cgd[1] - _y[FI_I]) + (Cgd[2] - _z[FI_I])*(Cgd[2] - _z[FI_I])) / sqrt((Cgd[0] - _x[FI_M])*(Cgd[0] - _x[FI_M]) + (Cgd[1] - _y[FI_M])*(Cgd[1] - _y[FI_M]) + (Cgd[2] - _z[FI_M])*(Cgd[2] - _z[FI_M]));
			ag[FI_T] = _f_des[FI_T]; //sqrt(((ag[FI_I] * Fx[FI_I] + ag[FI_I] * Fy[FI_I] + ag[FI_I] * Fz[FI_I]) + (ag[FI_M] * Fx[FI_M] + ag[FI_M] * Fy[FI_M] + ag[FI_M] * Fz[FI_M]))*((ag[FI_I] * Fx[FI_I] + ag[FI_I] * Fy[FI_I] + ag[FI_I] * Fz[FI_I]) + (ag[FI_M] * Fx[FI_M] + ag[FI_M] * Fy[FI_M] + ag[FI_M] * Fz[FI_M])));

			for (int fi = 0; fi < 3; fi++)
			{
				if (fi == FI_T)
					continue;

				for (int ji = 1; ji < 3; ji++)
				{
					_tau_task[fi][ji] = ag[fi] * (_J[fi][0][ji] * Fdx[fi] + _J[fi][1][ji] * Fdy[fi] + _J[fi][2][ji] * Fdz[fi]);
				}
			}
		}
		break;

	case eGraspMode_ENVELOP:
		
		if (_curT < 0.5)
		{
			//index
			_q_des[FI_I][0] = 0 * DEG2RAD;
			_q_des[FI_I][1] = 30 * DEG2RAD;
			_q_des[FI_I][2] = 60 * DEG2RAD;
			//middle
			_q_des[FI_M][0] = 0 * DEG2RAD;
			_q_des[FI_M][1] = 30 * DEG2RAD;
			_q_des[FI_M][2] = 60 * DEG2RAD;
			//Thumb
			_q_des[FI_T][0] = 0 * DEG2RAD;
			_q_des[FI_T][1] = 30 * DEG2RAD;
			_q_des[FI_T][2] = 60 * DEG2RAD;

			Motion_JointPD();
		}
		else
		{
			for (int fi = 0; fi < 3; fi++)
			{
				for (int ji = 1; ji < 3; ji++)
				{
					_tau_task[fi][ji] = (_envelop_torque_scalar) * 0.05;
				}
			}
		}
		break;

	default:
		break;
	}






	/*


	float e_x[4];
	float e_y[4];
	float e_z[4];

	float e_x_object;
	float e_y_object;
	float e_z_object;

	float xc_object;
	float yc_object;
	float zc_object;

	//float x_d_object;
	//float y_d_object;
	//float z_d_object;


	// center coord of an object graspped
	xc_object = (_x[0] + _x[1] + _x[3]) / 3;
	yc_object = (_y[0] + _y[1] + _y[3]) / 3;
	zc_object = (_z[0] + _z[1] + _z[3]) / 3;

	x_d_object = 0.0f;
	y_d_object = 0.0f; // 10.0f*sinf(5*_curT);
	z_d_object = 0.0f;

	////printf("Disp. x: %f\t y: %f\t z: %f\t\n", x_d_object, y_d_object, z_d_object);

	e_x_object = x_d_object - xc_object;
	e_y_object = y_d_object - yc_object;
	e_z_object = z_d_object - zc_object;

	////printf("Obj. x: %f\t y: %f\t z: %f\t\n\n", e_x_object, e_y_object, e_z_object);

	for (int i = 0; i < NOF; i++)
	{
		e_x[i] = (_x_des[i] - _x[i]);
		e_y[i] = (_y_des[i] - _y[i]);
		e_z[i] = (_z_des[i] - _z[i]);

		//printf("e_xyz:\t%f \t %f \t %f\n", e_x[i],e_y[i],e_z[i]);

		///////////////////////////////////////////
		// task damping
		t_TaskDamping[i][0] = 0;
		t_TaskDamping[i][1] = 0;
		t_TaskDamping[i][2] = 0;
		t_TaskDamping[i][3] = 0;

		///////////////////////////////////////////
		// task position
		t_Position[i][0] = (_J[i][0][0] * _kp[i][0] * (e_x[i]) + _J[i][1][0] * _kp[i][0] * (e_y[i]) + _J[i][2][0] * _kp[i][0] * (e_z[i]));
		t_Position[i][1] = (_J[i][0][1] * _kp[i][1] * (e_x[i]) + _J[i][1][1] * _kp[i][1] * (e_y[i]) + _J[i][2][1] * _kp[i][1] * (e_z[i]));
		t_Position[i][2] = (_J[i][0][2] * _kp[i][2] * (e_x[i]) + _J[i][1][2] * _kp[i][2] * (e_y[i]) + _J[i][2][2] * _kp[i][2] * (e_z[i]));
		t_Position[i][3] = (_J[i][0][3] * _kp[i][3] * (e_x[i]) + _J[i][1][3] * _kp[i][3] * (e_y[i]) + _J[i][2][3] * _kp[i][3] * (e_z[i]));

		///////////////////////////////////////////
		// desired pinching force
		t_Pinching[i][0] = (_J[i][0][0] * _f_des[i] * (e_x[i]) + _J[i][1][0] * _f_des[i] * (e_y[i]) + _J[i][2][0] * _f_des[i] * (e_z[i]));
		t_Pinching[i][1] = (_J[i][0][1] * _f_des[i] * (e_x[i]) + _J[i][1][1] * _f_des[i] * (e_y[i]) + _J[i][2][1] * _f_des[i] * (e_z[i]));
		t_Pinching[i][2] = (_J[i][0][2] * _f_des[i] * (e_x[i]) + _J[i][1][2] * _f_des[i] * (e_y[i]) + _J[i][2][2] * _f_des[i] * (e_z[i]));
		t_Pinching[i][3] = (_J[i][0][3] * _f_des[i] * (e_x[i]) + _J[i][1][3] * _f_des[i] * (e_y[i]) + _J[i][2][3] * _f_des[i] * (e_z[i]));

		///////////////////////////////////////////
		// object position
		t_Position_Object[i][0] = (_J[i][0][0] * _kp_task[i][0] * (e_x_object)+_J[i][1][0] * _kp_task[i][0] * (e_y_object)+_J[i][2][0] * _kp_task[i][0] * (e_z_object));
		t_Position_Object[i][1] = (_J[i][0][1] * _kp_task[i][1] * (e_x_object)+_J[i][1][1] * _kp_task[i][1] * (e_y_object)+_J[i][2][1] * _kp_task[i][1] * (e_z_object));
		t_Position_Object[i][2] = (_J[i][0][2] * _kp_task[i][2] * (e_x_object)+_J[i][1][2] * _kp_task[i][2] * (e_y_object)+_J[i][2][2] * _kp_task[i][2] * (e_z_object));
		t_Position_Object[i][3] = (_J[i][0][3] * _kp_task[i][3] * (e_x_object)+_J[i][1][3] * _kp_task[i][3] * (e_y_object)+_J[i][2][3] * _kp_task[i][3] * (e_z_object));
	
		///////////////////////////////////////////
		// calculated torque
		_tau_task[i][0] = ((t_Position[i][0] + t_Pinching[i][0] + t_Position_Object[i][0] - t_TaskDamping[i][0])*(1));
		_tau_task[i][1] = ((t_Position[i][1] + t_Pinching[i][1] + t_Position_Object[i][1] - t_TaskDamping[i][1])*(1));
		_tau_task[i][2] = ((t_Position[i][2] + t_Pinching[i][2] + t_Position_Object[i][2] - t_TaskDamping[i][2])*(1));
		_tau_task[i][3] = ((t_Position[i][3] + t_Pinching[i][3] + t_Position_Object[i][3] - t_TaskDamping[i][3])*(1));
	}
	*/
}

void BGrip9DOF::Motion_JointPD()
{
	for (int fi=0; fi<NOF; fi++) 
	{
		for (int ji = 0; ji<NOJ; ji++) 
		{
			_tau_task[fi][ji] = _kp[fi][ji] * (_q_des[fi][ji] - _q_filtered[fi][ji]) - _kd[fi][ji] * _qdot_filtered[fi][ji];
			_tau_task[fi][ji] /= 800.0; // pwm to torque
		}
	}
}
