#include "BGrip.h"
#include "memory.h"
#include "math.h"
#include "stdio.h"

BGrip11DOF::BGrip11DOF()
	: BGrip(eGripperType_11DOF)
{
	_mass[FI_I][0] = 0.056;
	_mass[FI_I][1] = 0.018;
	_mass[FI_I][2] = 0.061;
	_mass[FI_I][3] = 0.036;
	_mass[FI_M][0] = 0.056;
	_mass[FI_M][1] = 0.018;
	_mass[FI_M][2] = 0.061;
	_mass[FI_M][3] = 0.036;
	_mass[FI_T][0] = 0.007;
	_mass[FI_T][1] = 0.061;
	_mass[FI_T][2] = 0.036;

	_link[FI_I][0] = 0.0;
	_link[FI_I][1] = 0.0;
	_link[FI_I][2] = 0.054;
	_link[FI_I][3] = 0.068;
	_link[FI_M][0] = 0.0;
	_link[FI_M][1] = 0.0;
	_link[FI_M][2] = 0.054;
	_link[FI_M][3] = 0.068;
	_link[FI_T][0] = 0.0;
	_link[FI_T][1] = 0.054;
	_link[FI_T][2] = 0.068;

	_baseX = 0.03;
	_baseY = 0.035;
}

BGrip11DOF::~BGrip11DOF(void)
{
}

void BGrip11DOF::SetGains(int motionType)
{
	switch (motionType)
	{
	case eMotionType_READY:
		_kp[FI_I][0] = _kp[FI_M][0] = 500;
		_kp[FI_I][1] = _kp[FI_M][1] = 800;
		_kp[FI_I][2] = _kp[FI_M][2] = 800;
		_kp[FI_I][3] = _kp[FI_M][3] = 500;
		_kd[FI_I][0] = _kd[FI_M][0] = 25;
		_kd[FI_I][1] = _kd[FI_M][1] = 50;
		_kd[FI_I][2] = _kd[FI_M][2] = 50;
		_kd[FI_I][3] = _kd[FI_M][3] = 40;

		_kp[FI_T][0] = 500;
		_kp[FI_T][1] = 800;
		_kp[FI_T][2] = 500;
		_kp[FI_T][3] = 0;
		_kd[FI_T][0] = 25;
		_kd[FI_T][1] = 50;
		_kd[FI_T][2] = 40;
		_kd[FI_T][3] = 0;
		break;

	case eMotionType_GRASP:
		break;

	case eMotionType_JOINT_PD:
		_kp[FI_I][0] = _kp[FI_M][0] = 500;
		_kp[FI_I][1] = _kp[FI_M][1] = 800;
		_kp[FI_I][2] = _kp[FI_M][2] = 800;
		_kp[FI_I][3] = _kp[FI_M][3] = 500;
		_kd[FI_I][0] = _kd[FI_M][0] = 25;
		_kd[FI_I][1] = _kd[FI_M][1] = 50;
		_kd[FI_I][2] = _kd[FI_M][2] = 50;
		_kd[FI_I][3] = _kd[FI_M][3] = 40;

		_kp[FI_T][0] = 500;
		_kp[FI_T][1] = 800;
		_kp[FI_T][2] = 500;
		_kp[FI_T][3] = 0;
		_kd[FI_T][0] = 25;
		_kd[FI_T][1] = 50;
		_kd[FI_T][2] = 40;
		_kd[FI_T][3] = 0;
		break;

	case eMotionType_GRAVITY_COMP:
		_kp[FI_I][0] = _kp[FI_M][0] = 80; // was 120 orignially
		_kp[FI_I][1] = _kp[FI_M][1] = 90;
		_kp[FI_I][2] = _kp[FI_M][2] = 90;
		_kp[FI_I][3] = _kp[FI_M][3] = 80;
		_kd[FI_I][0] = _kd[FI_M][0] = sqrt(_kp[FI_I][0])*0.004*1.0;
		_kd[FI_I][1] = _kd[FI_M][1] = sqrt(_kp[FI_I][1])*0.004*2.0;
		_kd[FI_I][2] = _kd[FI_M][2] = sqrt(_kp[FI_I][2])*0.004*2.0;
		_kd[FI_I][3] = _kd[FI_M][3] = sqrt(_kp[FI_I][3])*0.004*1.0;

		_kp[FI_T][0] = 80; // was 120 orignially
		_kp[FI_T][1] = 90;
		_kp[FI_T][2] = 80;
		_kp[FI_T][3] = 0;
		_kd[FI_T][0] = sqrt(_kp[FI_T][0])*0.004*1.0;
		_kd[FI_T][1] = sqrt(_kp[FI_T][1])*0.004*2.0;
		_kd[FI_T][2] = sqrt(_kp[FI_T][2])*0.004*1.0;
		_kd[FI_T][3] = 0;
		break;

	default:
		BGrip::SetGains(motionType);
		return;
	}
}

void BGrip11DOF::SolveFK()
{
	// backup current solution for applying LPF later
	for (int i = 0; i<NOF; i++)
	{
		_x_pre[i] = _x[i];
		_y_pre[i] = _y[i];
		_z_pre[i] = _z[i];
		_x_filtered_pre[i] = _x_filtered[i];
		_y_filtered_pre[i] = _y_filtered[i];
		_z_filtered_pre[i] = _z_filtered[i];
	}

	// solve the forward kinematics of each finger.
	_x[FI_I] = _baseX - _link[FI_I][2] * (C0[FI_I] * S2[FI_I] + C2[FI_I] * S0[FI_I] * S1[FI_I]) - _link[FI_I][3] * (C3[FI_I] * (C0[FI_I] * S2[FI_I] + C2[FI_I] * S0[FI_I] * S1[FI_I]) + S3[FI_I] * (C0[FI_I] * C2[FI_I] - S0[FI_I] * S1[FI_I] * S2[FI_I]));
	_y[FI_I] = -_baseY - _link[FI_I][2]*(S0[FI_I] * S2[FI_I] - C0[FI_I] * C2[FI_I] * S1[FI_I]) - _link[FI_I][3] * (C3[FI_I] * (S0[FI_I] * S2[FI_I] - C0[FI_I] * C2[FI_I] * S1[FI_I]) + S3[FI_I] * (C2[FI_I] * S0[FI_I] + C0[FI_I] * S1[FI_I] * S2[FI_I]));
	_z[FI_I] = _link[FI_I][3] * (C1[FI_I] * C2[FI_I] * C3[FI_I] - C1[FI_I] * S2[FI_I] * S3[FI_I]) + _link[FI_I][2] * C1[FI_I] * C2[FI_I];

	_x[FI_M] = _baseX - _link[FI_M][2] * (C0[FI_M] * S2[FI_M] + C2[FI_M] * S0[FI_M] * S1[FI_M]) - _link[FI_M][3] * (C3[FI_M] * (C0[FI_M] * S2[FI_M] + C2[FI_M] * S0[FI_M] * S1[FI_M]) + S3[FI_M] * (C0[FI_M] * C2[FI_M] - S0[FI_M] * S1[FI_M] * S2[FI_M]));
	_y[FI_M] = _baseY - _link[FI_M][2] * (S0[FI_M] * S2[FI_M] - C0[FI_M] * C2[FI_M] * S1[FI_M]) - _link[FI_M][3] * (C3[FI_M] * (S0[FI_M] * S2[FI_M] - C0[FI_M] * C2[FI_M] * S1[FI_M]) + S3[FI_M] * (C2[FI_M] * S0[FI_M] + C0[FI_M] * S1[FI_M] * S2[FI_M]));
	_z[FI_M] = _link[FI_M][3] * (C1[FI_M] * C2[FI_M] * C3[FI_M] - C1[FI_M] * S2[FI_M] * S3[FI_M]) + _link[FI_M][2] * C1[FI_M] * C2[FI_M];

	_x[FI_T] = _link[FI_T][2] * (C0[FI_T] * C1[FI_T] * S2[FI_T] + C0[FI_T] * C2[FI_T] * S1[FI_T]) + _link[FI_T][1] * C0[FI_T] * S1[FI_T] - _baseX;
	_y[FI_T] = _link[FI_T][2] * (C1[FI_T] * S0[FI_T] * S2[FI_T] + C2[FI_T] * S0[FI_T] * S1[FI_T]) + _link[FI_T][1] * S0[FI_T] * S1[FI_T];
	_z[FI_T] = _link[FI_T][2] * (C1[FI_T] * C2[FI_T] - S1[FI_T] * S2[FI_T]) + _link[FI_T][1] * C1[FI_T];

	// apply LPF
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

void BGrip11DOF::CalculateJacobian()
{
	// jacobian
	// index finger
	_J[FI_I][0][0] = _link[FI_I][2]*(S0[FI_I] * S2[FI_I] - C0[FI_I] * C2[FI_I] * S1[FI_I]) + _link[FI_I][3]*(C3[FI_I] * (S0[FI_I] * S2[FI_I] - C0[FI_I] * C2[FI_I] * S1[FI_I]) + S3[FI_I] * (C2[FI_I] * S0[FI_I] + C0[FI_I] * S1[FI_I] * S2[FI_I]));
	_J[FI_I][0][1] = -_link[FI_I][3]*(C1[FI_I] * C2[FI_I] * C3[FI_I] * S0[FI_I] - C1[FI_I] * S0[FI_I] * S2[FI_I] * S3[FI_I]) - _link[FI_I][2]*C1[FI_I] * C2[FI_I] * S0[FI_I];
	_J[FI_I][0][2] = -_link[FI_I][2]*(C0[FI_I] * C2[FI_I] - S0[FI_I] * S1[FI_I] * S2[FI_I]) - _link[FI_I][3]*(C3[FI_I] * (C0[FI_I] * C2[FI_I] - S0[FI_I] * S1[FI_I] * S2[FI_I]) - S3[FI_I] * (C0[FI_I] * S2[FI_I] + C2[FI_I] * S0[FI_I] * S1[FI_I]));
	_J[FI_I][0][3] = -_link[FI_I][3]*(C3[FI_I] * (C0[FI_I] * C2[FI_I] - S0[FI_I] * S1[FI_I] * S2[FI_I]) - S3[FI_I] * (C0[FI_I] * S2[FI_I] + C2[FI_I] * S0[FI_I] * S1[FI_I]));

	_J[FI_I][1][0] = -_link[FI_I][2]*(C0[FI_I] * S2[FI_I] + C2[FI_I] * S0[FI_I] * S1[FI_I]) - _link[FI_I][3]*(C3[FI_I] * (C0[FI_I] * S2[FI_I] + C2[FI_I] * S0[FI_I] * S1[FI_I]) + S3[FI_I] * (C0[FI_I] * C2[FI_I] - S0[FI_I] * S1[FI_I] * S2[FI_I]));
	_J[FI_I][1][1] = _link[FI_I][3]*(C0[FI_I] * C1[FI_I] * C2[FI_I] * C3[FI_I] - C0[FI_I] * C1[FI_I] * S2[FI_I] * S3[FI_I]) + _link[FI_I][2]*C0[FI_I] * C1[FI_I] * C2[FI_I];
	_J[FI_I][1][2] = -_link[FI_I][2]*(C2[FI_I] * S0[FI_I] + C0[FI_I] * S1[FI_I] * S2[FI_I]) - _link[FI_I][3]*(C3[FI_I] * (C2[FI_I] * S0[FI_I] + C0[FI_I] * S1[FI_I] * S2[FI_I]) - S3[FI_I] * (S0[FI_I] * S2[FI_I] - C0[FI_I] * C2[FI_I] * S1[FI_I]));
	_J[FI_I][1][3] = -_link[FI_I][3]*(C3[FI_I] * (C2[FI_I] * S0[FI_I] + C0[FI_I] * S1[FI_I] * S2[FI_I]) - S3[FI_I] * (S0[FI_I] * S2[FI_I] - C0[FI_I] * C2[FI_I] * S1[FI_I]));

	_J[FI_I][2][0] = 0.0f;
	_J[FI_I][2][1] = _link[FI_I][3]*(S1[FI_I] * S2[FI_I] * S3[FI_I] - C2[FI_I] * C3[FI_I] * S1[FI_I]) - _link[FI_I][2]*C2[FI_I] * S1[FI_I];
	_J[FI_I][2][2] = -_link[FI_I][3]*(C1[FI_I] * C2[FI_I] * S3[FI_I] + C1[FI_I] * C3[FI_I] * S2[FI_I]) - _link[FI_I][2]*C1[FI_I] * S2[FI_I];
	_J[FI_I][2][3] = -_link[FI_I][3]*(C1[FI_I] * C2[FI_I] * S3[FI_I] + C1[FI_I] * C3[FI_I] * S2[FI_I]);

	// middle finger
	_J[FI_M][0][0] = _link[FI_M][2]*(S0[FI_M] * S2[FI_M] - C0[FI_M] * C2[FI_M] * S1[FI_M]) + _link[FI_M][3]*(C3[FI_M] * (S0[FI_M] * S2[FI_M] - C0[FI_M] * C2[FI_M] * S1[FI_M]) + S3[FI_M] * (C2[FI_M] * S0[FI_M] + C0[FI_M] * S1[FI_M] * S2[FI_M]));
	_J[FI_M][0][1] = -_link[FI_M][3]*(C1[FI_M] * C2[FI_M] * C3[FI_M] * S0[FI_M] - C1[FI_M] * S0[FI_M] * S2[FI_M] * S3[FI_M]) - _link[FI_M][2]*C1[FI_M] * C2[FI_M] * S0[FI_M];
	_J[FI_M][0][2] = -_link[FI_M][2]*(C0[FI_M] * C2[FI_M] - S0[FI_M] * S1[FI_M] * S2[FI_M]) - _link[FI_M][3]*(C3[FI_M] * (C0[FI_M] * C2[FI_M] - S0[FI_M] * S1[FI_M] * S2[FI_M]) - S3[FI_M] * (C0[FI_M] * S2[FI_M] + C2[FI_M] * S0[FI_M] * S1[FI_M]));
	_J[FI_M][0][3] = -_link[FI_M][3]*(C3[FI_M] * (C0[FI_M] * C2[FI_M] - S0[FI_M] * S1[FI_M] * S2[FI_M]) - S3[FI_M] * (C0[FI_M] * S2[FI_M] + C2[FI_M] * S0[FI_M] * S1[FI_M]));

	_J[FI_M][1][0] = -_link[FI_M][2]*(C0[FI_M] * S2[FI_M] + C2[FI_M] * S0[FI_M] * S1[FI_M]) - _link[FI_M][3]*(C3[FI_M] * (C0[FI_M] * S2[FI_M] + C2[FI_M] * S0[FI_M] * S1[FI_M]) + S3[FI_M] * (C0[FI_M] * C2[FI_M] - S0[FI_M] * S1[FI_M] * S2[FI_M]));
	_J[FI_M][1][1] = _link[FI_M][3]*(C0[FI_M] * C1[FI_M] * C2[FI_M] * C3[FI_M] - C0[FI_M] * C1[FI_M] * S2[FI_M] * S3[FI_M]) + _link[FI_M][2]*C0[FI_M] * C1[FI_M] * C2[FI_M];
	_J[FI_M][1][2] = -_link[FI_M][2]*(C2[FI_M] * S0[FI_M] + C0[FI_M] * S1[FI_M] * S2[FI_M]) - _link[FI_M][3]*(C3[FI_M] * (C2[FI_M] * S0[FI_M] + C0[FI_M] * S1[FI_M] * S2[FI_M]) - S3[FI_M] * (S0[FI_M] * S2[FI_M] - C0[FI_M] * C2[FI_M] * S1[FI_M]));
	_J[FI_M][1][3] = -_link[FI_M][3]*(C3[FI_M] * (C2[FI_M] * S0[FI_M] + C0[FI_M] * S1[FI_M] * S2[FI_M]) - S3[FI_M] * (S0[FI_M] * S2[FI_M] - C0[FI_M] * C2[FI_M] * S1[FI_M]));

	_J[FI_M][2][0] = 0.0f;
	_J[FI_M][2][1] = _link[FI_M][3]*(S1[FI_M] * S2[FI_M] * S3[FI_M] - C2[FI_M] * C3[FI_M] * S1[FI_M]) - _link[FI_M][2]*C2[FI_M] * S1[FI_M];
	_J[FI_M][2][2] = -_link[FI_M][3]*(C1[FI_M] * C2[FI_M] * S3[FI_M] + C1[FI_M] * C3[FI_M] * S2[FI_M]) - _link[FI_M][2]*C1[FI_M] * S2[FI_M];
	_J[FI_M][2][3] = -_link[FI_M][3]*(C1[FI_M] * C2[FI_M] * S3[FI_M] + C1[FI_M] * C3[FI_M] * S2[FI_M]);

	// thumb
	_J[FI_T][0][0] = -_link[FI_T][2]*(C1[FI_T] * S0[FI_T] * S2[FI_T] + C2[FI_T] * S0[FI_T] * S1[FI_T]) - _link[FI_T][1]*S0[FI_T] * S1[FI_T];
	_J[FI_T][0][1] = _link[FI_T][2]*(C0[FI_T] * C1[FI_T] * C2[FI_T] - C0[FI_T] * S1[FI_T] * S2[FI_T]) + _link[FI_T][1]*C0[FI_T] * C1[FI_T];
	_J[FI_T][0][2] = _link[FI_T][2]*(C0[FI_T] * C1[FI_T] * C2[FI_T] - C0[FI_T] * S1[FI_T] * S2[FI_T]);

	_J[FI_T][1][0] = _link[FI_T][2]*(C0[FI_T] * C1[FI_T] * S2[FI_T] + C0[FI_T] * C2[FI_T] * S1[FI_T]) + _link[FI_T][1]*C0[FI_T] * S1[FI_T];
	_J[FI_T][1][1] = _link[FI_T][1]*C1[FI_T] * S0[FI_T] - _link[FI_T][2]*(S0[FI_T] * S1[FI_T] * S2[FI_T] - C1[FI_T] * C2[FI_T] * S0[FI_T]);
	_J[FI_T][1][2] = -_link[FI_T][2]*(S0[FI_T] * S1[FI_T] * S2[FI_T] - C1[FI_T] * C2[FI_T] * S0[FI_T]);

	_J[FI_T][2][0] = 0.0f;
	_J[FI_T][2][1] = -_link[FI_T][2]*(C1[FI_T] * S2[FI_T] + C2[FI_T] * S1[FI_T]) - _link[FI_T][1]*S1[FI_T];
	_J[FI_T][2][2] = -_link[FI_T][2]*(C1[FI_T] * S2[FI_T] + C2[FI_T] * S1[FI_T]);
}

void BGrip11DOF::CalculateGravity()
{
	static double g_ = 9.81;

	_G[FI_I][0] = 0.0;
	_G[FI_I][1] = -(g_ * _mass[FI_I][2] * _link[FI_I][2] * 0.5* C2[FI_I] * S1[FI_I]) + (g_ * _mass[FI_I][3] * ((_link[FI_I][3] * (S2[FI_I] * S3[FI_I] - C2[FI_I] * C3[FI_I]) * S1[FI_I]) * 0.5 - _link[FI_I][2] * C2[FI_I] * S1[FI_I]));
	_G[FI_I][2] = -(g_ * _mass[FI_I][2] * _link[FI_I][2] * 0.5 * S2[FI_I] * C1[FI_I]) - (g_ * _mass[FI_I][3] * (_link[FI_I][3] * 0.5 * S23[FI_I] * C1[FI_I] + _link[FI_I][2] * S2[FI_I] * C1[FI_I]));
	_G[FI_I][3] = -(g_ * _mass[FI_I][3] * _link[FI_I][3] * 0.5 * S23[FI_I] * C1[FI_I]);

	_G[FI_M][0] = 0.0;
	_G[FI_M][1] = -(g_ * _mass[FI_M][2] * _link[FI_M][2] * 0.5* C2[FI_M] * S1[FI_M]) + (g_ * _mass[FI_M][3] * ((_link[FI_M][3] * (S2[FI_M] * S3[FI_M] - C2[FI_M] * C3[FI_M]) * S1[FI_M]) * 0.5 - _link[FI_M][2] * C2[FI_M] * S1[FI_M]));
	_G[FI_M][2] = -(g_ * _mass[FI_M][2] * _link[FI_M][2] * 0.5 * S2[FI_M] * C1[FI_M]) - (g_ * _mass[FI_M][3] * (_link[FI_M][3] * 0.5 * S23[FI_M] * C1[FI_M] + _link[FI_M][2] * S2[FI_M] * C1[FI_M]));
	_G[FI_M][3] = -(g_ * _mass[FI_M][3] * _link[FI_M][3] * 0.5 * S23[FI_M] * C1[FI_M]);

	_G[FI_T][0] = 0.0;
	_G[FI_T][1] = -(g_ * _mass[FI_T][1] * _link[FI_T][1] * 0.5 * S1[FI_T]) - (g_ * _mass[FI_T][2] * (_link[FI_T][2] * 0.5 * S12[FI_T] + _link[FI_T][1] * S1[FI_T]));
	_G[FI_T][2] = -(g_ * _mass[FI_T][2] * _link[FI_T][2] * 0.5 * S12[FI_T]);
}

void BGrip11DOF::CalculateGravityEx()
{
}

void BGrip11DOF::Motion_Ready()
{
	switch (_graspMode)
	{
	case eGraspMode_GROPED:

		//index
		_q_des[FI_I][0] = -30 * DEG2RAD;
		_q_des[FI_I][1] = 0 * DEG2RAD;
		_q_des[FI_I][2] = (-30 - _openSize) * DEG2RAD;
		_q_des[FI_I][3] = (60 + 0.5*_openSize) * DEG2RAD;
		//middle
		_q_des[FI_M][0] = 30 * DEG2RAD;
		_q_des[FI_M][1] = 0 * DEG2RAD;
		_q_des[FI_M][2] = (-30 - _openSize) * DEG2RAD;
		_q_des[FI_M][3] = (60 + 0.5*_openSize) * DEG2RAD;
		//Thumb
		_q_des[FI_T][0] = 0 * DEG2RAD;
		_q_des[FI_T][1] = (-30 - _openSize) * DEG2RAD;
		_q_des[FI_T][2] = (60 + 0.5*_openSize) * DEG2RAD;
		break;

	case eGraspMode_PARALLEL:

		//index
		_q_des[FI_I][0] = 0 * DEG2RAD;
		_q_des[FI_I][1] = 0 * DEG2RAD;
		_q_des[FI_I][2] = (-30 - _openSize) * DEG2RAD;
		_q_des[FI_I][3] = (60 + 0.5*_openSize) * DEG2RAD;
		//middle
		_q_des[FI_M][0] = 0 * DEG2RAD;
		_q_des[FI_M][1] = 0 * DEG2RAD;
		_q_des[FI_M][2] = (-30 - _openSize) * DEG2RAD;
		_q_des[FI_M][3] = (60 + 0.5*_openSize) * DEG2RAD;
		//Thumb
		_q_des[FI_T][0] = 0 * DEG2RAD;
		_q_des[FI_T][1] = (-30 - _openSize) * DEG2RAD;
		_q_des[FI_T][2] = (60 + 0.5*_openSize) * DEG2RAD;
		break;

	case eGraspMode_ENVELOP:

		//index
		_q_des[FI_I][0] = 0 * DEG2RAD;
		_q_des[FI_I][1] = 0 * DEG2RAD;
		_q_des[FI_I][2] = -30 * DEG2RAD;
		_q_des[FI_I][3] = 60 * DEG2RAD;
		//middle
		_q_des[FI_M][0] = 0 * DEG2RAD;
		_q_des[FI_M][1] = 0 * DEG2RAD;
		_q_des[FI_M][2] = -30 * DEG2RAD;
		_q_des[FI_M][3] = 60 * DEG2RAD;
		//Thumb
		_q_des[FI_T][0] = 0 * DEG2RAD;
		_q_des[FI_T][1] = -30 * DEG2RAD;
		_q_des[FI_T][2] = 60 * DEG2RAD;
		break;

	case eGraspMode_PINCH_INDEX:

		//index
		_q_des[FI_I][0] = -30 * DEG2RAD;
		_q_des[FI_I][1] = 0 * DEG2RAD;
		_q_des[FI_I][2] = (-20 - _openSize) * DEG2RAD;
		_q_des[FI_I][3] = (55 + 0.5*_openSize) * DEG2RAD;
		//middle
		_q_des[FI_M][0] = 30 * DEG2RAD;
		_q_des[FI_M][1] = 0 * DEG2RAD;
		_q_des[FI_M][2] = -30 * DEG2RAD;
		_q_des[FI_M][3] = 0 * DEG2RAD;
		//Thumb
		_q_des[FI_T][0] = -30 * DEG2RAD;
		_q_des[FI_T][1] = (-20 - _openSize) * DEG2RAD;
		_q_des[FI_T][2] = (55 + 0.5*_openSize) * DEG2RAD;
		break;

	case eGraspMode_PINCH_MIDDLE:

		//index
		_q_des[FI_I][0] = -30 * DEG2RAD;
		_q_des[FI_I][1] = 0 * DEG2RAD;
		_q_des[FI_I][2] = -30 * DEG2RAD;
		_q_des[FI_I][3] = 0 * DEG2RAD;
		//middle
		_q_des[FI_M][0] = 30 * DEG2RAD;
		_q_des[FI_M][1] = 0 * DEG2RAD;
		_q_des[FI_M][2] = (-20 - _openSize) * DEG2RAD;
		_q_des[FI_M][3] = (55 + 0.5*_openSize) * DEG2RAD;
		//Thumb
		_q_des[FI_T][0] = 30 * DEG2RAD;
		_q_des[FI_T][1] = (-20 - _openSize) * DEG2RAD;
		_q_des[FI_T][2] = (55 + 0.5*_openSize) * DEG2RAD;
		break;

	case eGraspMode_PINCH_PEG:

		//index
		_q_des[FI_I][0] = -92 * DEG2RAD;
		_q_des[FI_I][1] = 0 * DEG2RAD;
		_q_des[FI_I][2] = (-30 - _openSize) * DEG2RAD;
		_q_des[FI_I][3] = 60 * DEG2RAD;
		//middle
		_q_des[FI_M][0] = 91 * DEG2RAD;
		_q_des[FI_M][1] = 0 * DEG2RAD;
		_q_des[FI_M][2] = (-30 - _openSize) * DEG2RAD;
		_q_des[FI_M][3] = 60 * DEG2RAD;
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

void BGrip11DOF::Motion_GravityComp()
{
	memset(_tau_task, 0, SIZEOF_VARRAY);
}

void BGrip11DOF::Motion_Grasp()
{
	double KT[3];
	double Cg[3];
	double Cgd[3];

	double F[NOF], Fx[NOF], Fy[NOF], Fz[NOF], Fdx[NOF], Fdy[NOF], Fdz[NOF];
	double ag[3];

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
			_q_des[FI_I][1] = 0 * DEG2RAD;
			_q_des[FI_I][2] = -20 * DEG2RAD;
			_q_des[FI_I][3] = 65 * DEG2RAD;
			//middle
			_q_des[FI_M][0] = 30 * DEG2RAD;
			_q_des[FI_M][1] = 0 * DEG2RAD;
			_q_des[FI_M][2] = -20 * DEG2RAD;
			_q_des[FI_M][3] = 65 * DEG2RAD;
			//Thumb
			_q_des[FI_T][0] = 0 * DEG2RAD;
			_q_des[FI_T][1] = -20 * DEG2RAD;
			_q_des[FI_T][2] = 65 * DEG2RAD;

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
				for (int ji = 0; ji < 4; ji++)
				{
					_tau_task[fi][ji] = ag[fi] * (_J[fi][0][ji] * Fdx[fi] + _J[fi][1][ji] * Fdy[fi] + _J[fi][2][ji] * Fdz[fi]);
				}
			}
		}
		break;

	case eGraspMode_PARALLEL:

		//index
		_q_des[FI_I][0] = 0 * DEG2RAD;
		_q_des[FI_I][1] = 0 * DEG2RAD;
		_q_des[FI_I][2] = -20 * DEG2RAD;
		_q_des[FI_I][3] = 65 * DEG2RAD;
		//middle
		_q_des[FI_M][0] = 0 * DEG2RAD;
		_q_des[FI_M][1] = 0 * DEG2RAD;
		_q_des[FI_M][2] = -20 * DEG2RAD;
		_q_des[FI_M][3] = 65 * DEG2RAD;
		//Thumb
		_q_des[FI_T][0] = 0 * DEG2RAD;
		_q_des[FI_T][1] = -20 * DEG2RAD;
		_q_des[FI_T][2] = 65 * DEG2RAD;

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
				for (int ji = 1; ji < 4; ji++)
				{
					_tau_task[fi][ji] = ag[fi] * (_J[fi][0][ji] * Fdx[fi] + _J[fi][1][ji] * Fdy[fi] + _J[fi][2][ji] * Fdz[fi]);
				}
			}
		}
		break;

	case eGraspMode_PINCH_INDEX:

		//index
		_q_des[FI_I][0] = -30 * DEG2RAD;
		_q_des[FI_I][1] = 0 * DEG2RAD;
		_q_des[FI_I][2] = -20 * DEG2RAD;
		_q_des[FI_I][3] = 60 * DEG2RAD;
		//middle
		_q_des[FI_M][0] = _q[FI_M][0];
		_q_des[FI_M][1] = _q[FI_M][1];
		_q_des[FI_M][2] = _q[FI_M][2];
		_q_des[FI_M][3] = _q[FI_M][3];
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

				for (int ji = 1; ji < 4; ji++)
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
		_q_des[FI_I][3] = _q[FI_I][3];
		//middle
		_q_des[FI_M][0] = 30 * DEG2RAD;
		_q_des[FI_M][1] = 0 * DEG2RAD;
		_q_des[FI_M][2] = -20 * DEG2RAD;
		_q_des[FI_M][3] = 60 * DEG2RAD;
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

				for (int ji = 1; ji < 4; ji++)
				{
					_tau_task[fi][ji] = ag[fi] * (_J[fi][0][ji] * Fdx[fi] + _J[fi][1][ji] * Fdy[fi] + _J[fi][2][ji] * Fdz[fi]);
				}
			}
		}
		break;

	case eGraspMode_PINCH_PEG:

		//index
		_q_des[FI_I][0] = -90 * DEG2RAD;
		_q_des[FI_I][1] = 0 * DEG2RAD;
		_q_des[FI_I][2] = -20 * DEG2RAD;
		_q_des[FI_I][3] = 70 * DEG2RAD;
		//middle
		_q_des[FI_M][0] = 90 * DEG2RAD;
		_q_des[FI_M][1] = 0 * DEG2RAD;
		_q_des[FI_M][2] = -20 * DEG2RAD;
		_q_des[FI_M][3] = 70 * DEG2RAD;
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

				for (int ji = 1; ji < 4; ji++)
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
			_q_des[FI_I][1] = 0 * DEG2RAD;
			_q_des[FI_I][2] = 30 * DEG2RAD;
			_q_des[FI_I][3] = 60 * DEG2RAD;
			//middle
			_q_des[FI_M][0] = 0 * DEG2RAD;
			_q_des[FI_M][1] = 0 * DEG2RAD;
			_q_des[FI_M][2] = 30 * DEG2RAD;
			_q_des[FI_M][3] = 60 * DEG2RAD;
			//Thumb
			_q_des[FI_T][0] = 0 * DEG2RAD;
			_q_des[FI_T][1] = 30 * DEG2RAD;
			_q_des[FI_T][2] = 60 * DEG2RAD;

			Motion_JointPD();
		}
		else
		{
			_tau_task[FI_I][2] = (_envelop_torque_scalar) * 0.05;
			_tau_task[FI_I][3] = (_envelop_torque_scalar) * 0.05;
			_tau_task[FI_M][2] = (_envelop_torque_scalar) * 0.05;
			_tau_task[FI_M][3] = (_envelop_torque_scalar) * 0.05;
			_tau_task[FI_T][1] = (_envelop_torque_scalar) * 0.05;
			_tau_task[FI_T][2] = (_envelop_torque_scalar) * 0.05;
		}
		break;

	default:
		break;
	}
}

void BGrip11DOF::Motion_JointPD()
{
	for (int fi = 0; fi<NOF; fi++)
	{
		for (int ji = 0; ji<NOJ; ji++)
		{
			_tau_task[fi][ji] = _kp[fi][ji] * (_q_des[fi][ji] - _q_filtered[fi][ji]) - _kd[fi][ji] * _qdot_filtered[fi][ji];
			_tau_task[fi][ji] /= 800.0; // pwm to torque
		}
	}
}
