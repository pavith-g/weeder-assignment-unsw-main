#pragma once

#include "NetworkedModule.h";
#include "SMObjects.h";

ref class VC : public NetworkedModule
{
public:
	VC(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC);

	error_state	setupSharedMemory() override;
	error_state processHeartBeats() override;
	error_state processSharedMemory() override;
	bool getShudownFlag() override;
	void threadFunction() override;

private:
	SM_VehicleControl^ SM_VC;
	Stopwatch^ VC_Stopwatch;
};