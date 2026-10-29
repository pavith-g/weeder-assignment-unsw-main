#pragma once

#include "NetworkedModule.h";
#include "SMObjects.h"


ref class LiDAR : public NetworkedModule 
{
public:
	LiDAR(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_LiDAR);


	error_state	setupSharedMemory() override;
	error_state processHeartBeats() override;
	error_state processSharedMemory() override;
	bool getShudownFlag() override;
	void threadFunction() override;

private:
	SM_Laser^ SM_LiDAR_;
	Stopwatch^ LiDAR_Stopwatch;
};