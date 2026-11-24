#pragma once

#include "NetworkedModule.h";
#include "SMObjects.h"

ref class GNSS : public NetworkedModule
{
public:
	GNSS(SM_ThreadManagement^ SM_TM, SM_GPS^ SM_GNSS);

	error_state communicate() override;
	error_state	SetupSharedMemory() override;
	error_state processHeartBeats() override;
	error_state processSharedMemory() override;
	void disconnect() override;
	bool getShutdownFlag() override;
	void threadFunction() override;
	void selfShutDown() override;

private:
	SM_GPS^ SM_GNSS_;
	Stopwatch^ GNSS_Stopwatch;

	System::Collections::Generic::List<unsigned char>^ rxBuffer_;
};
		