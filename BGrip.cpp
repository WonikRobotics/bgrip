#include "BGrip.h"
#include "memory.h"
#include "math.h"
#include "stdio.h"

BGrip::BGrip(eGripperType gt)
{
	_dT = 0.003;
	_gripperType = gt;
	_motionType = eMotionType_NONE; 
	_graspMode = eGraspMode_NONE;
	_openSize = 0;
	
	memset(_q, 0, sizeof(_q));
	memset(_q_filtered, 0, sizeof(_q_filtered));
	memset(_q_pre, 0, sizeof(_q_pre));
	memset(_q_filtered_pre, 0, sizeof(_q_filtered_pre));
	memset(_qdot, 0, sizeof(_qdot));
	memset(_qdot_filtered, 0, sizeof(_qdot_filtered));
	memset(_qdot_pre, 0, sizeof(_qdot_pre));
	memset(_tau_des, 0, sizeof(_tau_des));
	memset(_q_des, 0, sizeof(_q_des));

	_envelop_torque_scalar = 1.0;

	memset(_x, 0, sizeof(_x));
	memset(_y, 0, sizeof(_y));
	memset(_z, 0, sizeof(_z));
	memset(_x_filtered, 0, sizeof(_x_filtered));
	memset(_y_filtered, 0, sizeof(_y_filtered));
	memset(_z_filtered, 0, sizeof(_z_filtered));
	memset(_x_pre, 0, sizeof(_x_pre));
	memset(_y_pre, 0, sizeof(_y_pre));
	memset(_z_pre, 0, sizeof(_z_pre));
	memset(_x_filtered_pre, 0, sizeof(_x_filtered_pre));
	memset(_y_filtered_pre, 0, sizeof(_y_filtered_pre));
	memset(_z_filtered_pre, 0, sizeof(_z_filtered_pre));

	memset(_xdot, 0, sizeof(_xdot));
	memset(_ydot, 0, sizeof(_ydot));
	memset(_zdot, 0, sizeof(_zdot));
	memset(_xdot_filtered, 0, sizeof(_xdot_filtered));
	memset(_ydot_filtered, 0, sizeof(_ydot_filtered));
	memset(_zdot_filtered, 0, sizeof(_zdot_filtered));
	memset(_xdot_pre, 0, sizeof(_xdot_pre));
	memset(_ydot_pre, 0, sizeof(_ydot_pre));
	memset(_zdot_pre, 0, sizeof(_zdot_pre));
	
	memset(_kp, 0, sizeof(_kp));
	memset(_kd, 0, sizeof(_kd));
	memset(_kp_task, 0, sizeof(_kp_task));
	memset(_kd_task, 0, sizeof(_kd_task));

	memset(_tau_task, 0, sizeof(_tau_task));

	memset(_J, 0, sizeof(_J));
	
	memset(_G, 0, sizeof(_G));
	_grav_comp_enabled = false;

	memset(_f_des, 0, sizeof(_f_des));

	memset(_x_des, 0, sizeof(_x_des));
	memset(_y_des, 0, sizeof(_y_des));
	memset(_z_des, 0, sizeof(_z_des));

	memset(_mass, 0, SIZEOF_VARRAY);
	memset(_link, 0, SIZEOF_VARRAY);
	_baseX = 0.0;
	_baseY = 0.0;

	// initialize body orientation
	memset(_R, 0, sizeof(_R[0])*9);
	_R[0] = _R[4] =_R[8] = 1.0;
}

BGrip::~BGrip(void)
{
}

eGripperType BGrip::GetType()
{
	return _gripperType;
}

void BGrip::SetTimeInterval(double dT)
{
	_dT = dT;
}

double BGrip::GetTimeInterval()
{
	return _dT;
}

void BGrip::SetGraspMode(int graspMode)
{
	_curT = 0;
	_graspMode = (eGraspMode)graspMode;
	//SetGains(graspMode);
}

void BGrip::SetMotionType(int motionType)
{
	_curT = 0;
	_motionType = (eMotionType)motionType;
	SetGains(motionType);
}

void BGrip::EnableGravCompensation(bool on)
{
	_grav_comp_enabled = on;
}


void BGrip::SetOpenSize(double openSize)
{
	if (openSize < 0) openSize = 0;
	else if (openSize > 100) openSize = 100;
	_openSize = openSize;
}

void BGrip::SetJointPosition(double* q)
{
	memcpy(_q_pre, _q, SIZEOF_VARRAY);
	memcpy(_q, q, SIZEOF_VARRAY);
	memcpy(_q_filtered_pre, _q_filtered, SIZEOF_VARRAY);
	memcpy(_qdot_pre, _qdot, SIZEOF_VARRAY);
	for (int i=0; i<NOF; i++) 
	{
		for (int j=0; j<NOJ; j++) 
		{
			_q_filtered[i][j] = (0.6*_q_filtered[i][j]) + (0.198*_q_pre[i][j]) + (0.198*_q[i][j]);
			_qdot[i][j] = (_q_filtered[i][j]-_q_filtered_pre[i][j]) / _dT;
			_qdot_filtered[i][j] = (0.6*_qdot_filtered[i][j]) + (0.198*_qdot_pre[i][j]) + (0.198*_qdot[i][j]);
		}

		S0[i] = sin(_q[i][0]);
		C0[i] = cos(_q[i][0]);
		S1[i] = sin(_q[i][1]);
		C1[i] = cos(_q[i][1]);
		S2[i] = sin(_q[i][2]);
		C2[i] = cos(_q[i][2]);
		S3[i] = sin(_q[i][3]);
		C3[i] = cos(_q[i][3]);
		S12[i] = sin(_q[i][1] + _q[i][2]);
		C12[i] = cos(_q[i][1] + _q[i][2]);
		S23[i] = sin(_q[i][2] + _q[i][3]);
		C23[i] = cos(_q[i][2] + _q[i][3]);
		S123[i] = sin(_q[i][1] + _q[i][2] + _q[i][3]);
		C123[i] = cos(_q[i][1] + _q[i][2] + _q[i][3]);
	}

	SolveFK();
	CalculateJacobian();
	CalculateGravity();
	//CalculateGravityEx();
}

void BGrip::UpdateControl(double time)
{
	int i = 0;
	double o = 0.5;
	double c = 1.0;

	double fv[NOF][NOJ];	// viscous friction coefficient
	double fc[NOF][NOJ]; // columb friction coefficient

	double Scalef = 800.0;

	double t_JointDamping[NOF][NOJ];
	double t_JointFriction[NOF][NOJ];
	double t_Task[NOF][NOJ];
	double t_Gravity[NOF][NOJ];


	_curT += _dT;

	switch (_motionType)
	{
	case eMotionType_READY:
		Motion_Ready();
		break;

	case eMotionType_GRASP:
		Motion_Grasp();
		break;

	case eMotionType_GRAVITY_COMP:
		Motion_GravityComp();
		break;

	case eMotionType_JOINT_PD:
		Motion_JointPD();
		break;

	default:
		memset(_tau_des, 0, SIZEOF_VARRAY);
		return;
	}



	// coulomb friction coefficient
	// Tune these for gravity compensation
	fc[0][0] = fc[1][0] = fc[2][0] =			2.0f / Scalef * o;
	fc[0][1] = fc[1][1] = fc[2][1] =			30.0f / Scalef * o;
	fc[0][2] = fc[1][2] = fc[2][2] =			20.0f / Scalef * o;
	fc[0][3] = fc[1][3] = fc[2][3] =			10.0f / Scalef * o; // maybe make this higher for all?

	fc[3][0] = 0.0f / Scalef * o;
	fc[3][1] = 0.0f / Scalef * o;
	fc[3][2] = 0.0f / Scalef * o;
	fc[3][3] = 0.0f / Scalef * o;

	// viscous friction coefficient
	fv[0][0] = fv[1][0] = fv[2][0] =				2.0f / Scalef * c;//20.0f / Scalef * c;
	fv[0][1] = fv[1][1] = fv[2][1] =				3.0f / Scalef * c;
	fv[0][2] = fv[1][2] = fv[2][2] =				3.0f / Scalef * c;
	fv[0][3] = fv[1][3] = fv[2][3] =				0.5f / Scalef * c;

	fv[3][0] = 0.0f / Scalef * c;
	fv[3][1] = 0.0f / Scalef * c;
	fv[3][2] = 0.0f / Scalef * c;
	fv[3][3] = 0.0f / Scalef * c;
	


	for (i=0; i<NOF; i++)
	{
		///////////////////////////////////////////
		// joint damping
		t_JointDamping[i][0] = 0;//0.01f*_kd[i][0] * _qdot_filtered[i][0];
		t_JointDamping[i][1] = 0;//1.0f*_kd[i][1] * _qdot_filtered[i][1];
		t_JointDamping[i][2] = 0;//1.0f*_kd[i][2] * _qdot_filtered[i][2];
		t_JointDamping[i][3] = 0;//1.0f*_kd[i][3] * _qdot_filtered[i][3];

		///////////////////////////////////////////
		// friction
		t_JointFriction[i][0] = (fv[i][0] * _qdot_filtered[i][0]) + (fc[i][0] * (float)tanh(_qdot_filtered[i][0]*50.0f));
		t_JointFriction[i][1] = (fv[i][1] * _qdot_filtered[i][1]) + (fc[i][1] * (float)tanh(_qdot_filtered[i][1]*40.0f));
		t_JointFriction[i][2] = (fv[i][2] * _qdot_filtered[i][2]) + (fc[i][2] * (float)tanh(_qdot_filtered[i][2]*50.0f));
		t_JointFriction[i][3] = (fv[i][3] * _qdot_filtered[i][3]) + (fc[i][3] * (float)tanh(_qdot_filtered[i][3]*10.0f));

		///////////////////////////////////////////
		// task 
		t_Task[i][0] = _tau_task[i][0];
		t_Task[i][1] = _tau_task[i][1];
		t_Task[i][2] = _tau_task[i][2];
		t_Task[i][3] = _tau_task[i][3];
		
		///////////////////////////////////////////
		// gravity
		if (_grav_comp_enabled)
		{
			t_Gravity[i][0] = _G[i][0];
			t_Gravity[i][1] = _G[i][1];
			t_Gravity[i][2] = _G[i][2];
			t_Gravity[i][3] = _G[i][3];
		}
		else
		{
			t_Gravity[i][0] = 0;
			t_Gravity[i][1] = 0;
			t_Gravity[i][2] = 0;
			t_Gravity[i][3] = 0;
		}

		///////////////////////////////////////////
		// total
		_tau_des[i][0] = ((t_Task[i][0] - t_JointDamping[i][0] + t_Gravity[i][0] + t_JointFriction[i][0])*(1));
		_tau_des[i][1] = ((t_Task[i][1] - t_JointDamping[i][1] + t_Gravity[i][1] + t_JointFriction[i][1])*(1));
		_tau_des[i][2] = ((t_Task[i][2] - t_JointDamping[i][2] + t_Gravity[i][2] + t_JointFriction[i][2])*(1));
		_tau_des[i][3] = ((t_Task[i][3] - t_JointDamping[i][3] + t_Gravity[i][3] + t_JointFriction[i][3])*(1));
	}
}

void BGrip::GetJointTorque(double* tau)
{
	memcpy(tau, _tau_des, SIZEOF_VARRAY);
}

void BGrip::GetFKResult(double x[NOF], double y[NOF], double z[NOF])
{
	memcpy(x, _x, sizeof(_x[0])*NOF);
	memcpy(y, _y, sizeof(_y[0])*NOF);
	memcpy(z, _z, sizeof(_z[0])*NOF);
}

void BGrip::SetJointDesiredPosition(double* q)
{
	memcpy(_q_des, q, SIZEOF_VARRAY);
}

void BGrip::SetGainsEx(double* kp, double* kd)
{
	memcpy(_kp, kp, SIZEOF_VARRAY);
	memcpy(_kd, kd, SIZEOF_VARRAY);
}

void BGrip::SetGraspingForce(double f[NOF])
{
	memcpy(_f_des, f, sizeof(_f_des));
}

void BGrip::SetEnvelopTorqueScalar(double set_scalar)
{
	//set_scalar default is 1.0. Set in the header.
	_envelop_torque_scalar = set_scalar;
}

void BGrip::GetGraspingForce(double fx[NOF], double fy[NOF], double fz[NOF])
{
	int i;
	double e_x[NOF];
	double e_y[NOF];
	double e_z[NOF];
	double e_norm = 1.0;

	for (i=0; i<NOF; i++)
	{
		e_x[i] = (_x_des[i]-_x[i]);
		e_y[i] = (_y_des[i]-_y[i]);
		e_z[i] = (_z_des[i]-_z[i]);
		e_norm = sqrt(e_x[i]*e_x[i] + e_y[i]*e_y[i] + e_z[i]*e_z[i]);

		fx[i] = _f_des[i]*e_x[i]/e_norm;
		fy[i] = _f_des[i]*e_y[i]/e_norm;
		fz[i] = _f_des[i]*e_z[i]/e_norm;
	}
}

void BGrip::SetOrientation(double roll, double pitch, double yaw)
{
	memset(_R, 0, sizeof(_R[0])*9);
	_R[0] = _R[4] =_R[8] = 1.0;
}

void BGrip::SetOrientation(double R[9])
{
	memcpy(_R, R, sizeof(_R[0])*9);
}

void BGrip::SetGains(int motionType)
{
	switch (motionType)
	{
	case eMotionType_NONE:
	default:
	{
		memset(_kp, 0, sizeof(_kp));
		memset(_kd, 0, sizeof(_kd));
		memset(_kp_task, 0, sizeof(_kp_task));
		memset(_kd_task, 0, sizeof(_kd_task));
	}
	break;
	}
}

BGRIP_EXTERN_C_BEGIN

BGrip* bgCreateGripper9DOF()
{
	return new BGrip9DOF();
}

BGrip* bgCreateGripper11DOF()
{
	return new BGrip11DOF();
}

BGRIP_EXTERN_C_END
