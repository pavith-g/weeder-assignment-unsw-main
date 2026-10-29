#pragma once

#include "UGVModule.h"
#include "SMObjects.h"

ref class CrashAvoidance : public UGVModule
{
public:
	CrashAvoidance(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC, SM_Laser^ SM_LiDAR);

	error_state	setupSharedMemory() override;
	error_state processHeartBeats() override;
	error_state processSharedMemory() override;
	bool getShudownFlag() override;
	void threadFunction() override;

private:
	SM_VehicleControl^ SM_VC;
	SM_Laser^ SM_LiDAR;
	Stopwatch^ CrashAvoidance_Stopwatch;
};