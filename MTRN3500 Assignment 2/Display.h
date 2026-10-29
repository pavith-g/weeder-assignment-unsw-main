#pragma once

#include "NetworkedModule.h";
#include "SMObjects.h";

ref class Display : public NetworkedModule
{
public:
	Display(SM_ThreadManagement^ SM_TM, SM_Display^ SM_Display);

	error_state	setupSharedMemory() override;
	error_state processHeartBeats() override;
	error_state processSharedMemory() override;
	bool getShudownFlag() override;
	void threadFunction() override;

private:
	SM_Display^ SM_Display;
	SM_GPS^ SM_GNSS;
	SM_Laser^ SM_LiDAR;
	SM_VehicleControl^ SM_VC;
	Stopwatch^ Display_Stopwatch;

};