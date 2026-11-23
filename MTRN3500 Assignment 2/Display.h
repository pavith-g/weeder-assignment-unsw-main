#pragma once

#include "NetworkedModule.h";
#include "SMObjects.h";

ref class Display : public NetworkedModule
{
public:
	Display(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_LiDAR, SM_GPS^ SM_GNSS);

	error_state connect(String^ hostName, int portNumber) override;
	error_state communicate() override;
	void sendDisplayData();

	error_state	SetupSharedMemory() override;
	error_state processHeartBeats() override;
	error_state processSharedMemory() override;
	bool getShutdownFlag() override;
	void selfShutDown() override;
	void threadFunction() override;
	
private:
	SM_ThreadManagement^ SM_TM_;
	SM_GPS^ SM_GNSS_;
	SM_Laser^ SM_LiDAR_;
	SM_VehicleControl^ SM_VC;
	Stopwatch^ Display_Stopwatch;
};