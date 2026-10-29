#pragma once

#include "UGVModule.h"
#include "SMObjects.h"

ref class Controller : public UGVModule
{
public:
	Controller(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC);

	error_state	setupSharedMemory() override;
	error_state processHeartBeats() override;
	error_state processSharedMemory() override;
	bool getShudownFlag() override;
	void threadFunction() override;

private:
	SM_VehicleControl^ SM_VC;
	Stopwatch^ Controller_Stopwatch;
};