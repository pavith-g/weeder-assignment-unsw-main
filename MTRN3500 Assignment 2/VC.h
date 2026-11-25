#pragma once

#include "NetworkedModule.h";
#include "SMObjects.h";

ref class VC : public NetworkedModule
{
public:
	VC(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC);

	error_state	SetupSharedMemory() override;
	error_state processHeartBeats() override;
	error_state processSharedMemory() override;
	error_state sendCommand(double steer, double speed, int wdog, bool inhibit);
	error_state communicate() override;
	bool getShutdownFlag() override;
	void threadFunction() override;
	void selfShutDown() override;



private:
	SM_VehicleControl^ SM_VC_;
	Stopwatch^ VC_Stopwatch;
};