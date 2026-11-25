#pragma once


#include "ControllerInterface.h"
#include "UGVModule.h"
#include "SMObjects.h"


ref class Controller : public UGVModule
{
public:
	Controller(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC);
	~Controller();
	!Controller();

	error_state	SetupSharedMemory() override;
	error_state processHeartBeats() override;
	error_state processSharedMemory() override;
	bool getShutdownFlag() override;
	void selfShutDown() override;
	void threadFunction() override;
	error_state checkConnection();
	error_state updateState();
	void zeroControl();

	double MapValue(double value, double fromLow, double fromHigh, double toLow, double toHigh);

private:
	SM_VehicleControl^ SM_VC_;
	Stopwatch^ Controller_Stopwatch;
	
	System::IntPtr controllerIntfPtr;
	
	double leftTriggerVal;
	double rightTriggerVal;
	
	bool buttonX;

	double rightThumbX;

	const double maxSpeed = 1.0;
	const double maxSteer = 40.0;
};