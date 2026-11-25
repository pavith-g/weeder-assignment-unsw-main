#pragma once

#include "UGVModule.h"
#include "SMObjects.h"

ref class CrashAvoidance : public UGVModule
{
public:
	CrashAvoidance(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC, SM_Laser^ SM_LiDAR);

	error_state	SetupSharedMemory() override;
	error_state processHeartBeats() override;
	error_state processSharedMemory() override;
	bool getShutdownFlag() override;
	void selfShutDown() override;
	void threadFunction() override;

private:
	SM_VehicleControl^ SM_VC_;
	SM_Laser^ SM_LiDAR_;
	Stopwatch^ CrashAvoidance_Stopwatch;

	array<double>^ localX;
	array<double>^ localY;

	bool wasInhibited;
}; 