#pragma once

#include "NetworkedModule.h";
#include "SMObjects.h";




ref class LiDAR : public NetworkedModule 
{
public:
	LiDAR(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_LiDAR);


	error_state connect(String^ hostName, int portNumber) override;
	error_state communicate() override;
	error_state authenticate(String^ id) override;
	void sendCommand(String^ command);
	void outputDataToTerminal() override;
	error_state	SetupSharedMemory() override;
	error_state processHeartBeats() override;
	error_state processSharedMemory() override;
	bool getShutdownFlag() override;
	void threadFunction() override;
	void selfShutDown() override;



private:
	SM_Laser^ SM_LiDAR_;
	Stopwatch^ LiDAR_Stopwatch;
};