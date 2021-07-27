#include "BGrip.h"
#include "memory.h"
#include "math.h"
#include "stdio.h"

BGrip11DOF::BGrip11DOF()
	: BGrip(eGripperType_11DOF)
{
}

BGrip11DOF::~BGrip11DOF(void)
{
}

void BGrip11DOF::SetGains(int motionType)
{
}

void BGrip11DOF::SolveFK()
{
}

void BGrip11DOF::CalculateJacobian()
{
}

void BGrip11DOF::CalculateGravity()
{
}

void BGrip11DOF::CalculateGravityEx()
{
}

void BGrip11DOF::Motion_Ready()
{
}

void BGrip11DOF::Motion_GravityComp()
{
}

void BGrip11DOF::Motion_Grasp()
{
}

void BGrip11DOF::Motion_JointPD()
{
}
