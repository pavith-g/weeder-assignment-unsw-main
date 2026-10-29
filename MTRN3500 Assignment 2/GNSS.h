#pragma once

#include "NetworkedModule.h";
#include "SMObjects.h"

ref class GNSS : public NetworkedModule
{
public:
	GNSS(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_LiDAR);

	error_state setupSharedMemory() override;
	error_state processHeartBeats() override;
	error_state processSharedMemory() override;
	bool getShutdownFlag() override;
	void threadFunction() override;

private:
	SM_ThreadManagement^ SM_GNSS;
	Stopwatch^ GNSS_Stopwatch;
};
		