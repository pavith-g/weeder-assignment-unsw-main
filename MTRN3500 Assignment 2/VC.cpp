#include "VC.h"


VC::VC(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC) {
	SM_TM_ = SM_TM;
	SM_VC_ = SM_VC;
	VC_Stopwatch = gcnew Stopwatch;
}

error_state	VC::SetupSharedMemory() {
	return error_state::SUCCESS;
}

error_state VC::communicate() {
	return error_state::SUCCESS;
}

error_state VC::processHeartBeats() {
	if ((SM_TM_->heartbeat & bit_VC) == 0) {
		SM_TM_->heartbeat |= bit_VC;
		VC_Stopwatch->Restart();
	}
	return error_state::SUCCESS;
}

error_state VC::sendCommand(double steer, double speed, int wdog, bool inhibit) {

	if (verifyConnection() != error_state::SUCCESS) {
		return error_state::ERR_CONNECTION;
	}

	double tempSteer = steer;
	double tempSpeed = speed;

	// Check if vehicle is inhibited by crash avoidance. 
	// Allow reverse motion
	if (inhibit && tempSpeed > 0.0) {
		// tempSteer = 0;
		tempSpeed = 0;
	}
	

	String^ command = "# " + tempSteer.ToString() + " " + tempSpeed.ToString() + " " + wdog.ToString() + " #";

	// Console::WriteLine(command);

	array<unsigned char>^ packet = System::Text::Encoding::ASCII->GetBytes(command);

	try {
		Stream->Write(packet, 0, packet->Length);
	}
	catch (System::IO::IOException^) {
		disconnect();
		return error_state::ERR_CONNECTION;
	}
	catch (System::Net::Sockets::SocketException^) {
		disconnect();
		return error_state::ERR_CONNECTION;
	}
	catch (Exception^) {
		disconnect();
		return error_state::ERR_CONNECTION;
	}

	return error_state::SUCCESS;
}

error_state VC::processSharedMemory() {
	if (SM_VC_ == nullptr || SM_VC_->lockObject == nullptr) {
		return error_state::ERR_SM;
	}

	double steer = 0.0;
	double speed = 0.0;
	bool wdog = false;
	bool inhibit = false;
	bool useCrashAvoidance = false;

	bool lockTaken = false;

	try {
		Monitor::Enter(SM_VC_->lockObject, lockTaken);
		steer = SM_VC_->Steering;
		speed = SM_VC_->Speed;
		wdog = SM_VC_->wdog;
		useCrashAvoidance = SM_VC_->useCrashAvoidance;
		inhibit = SM_VC_->inhibit;
		SM_VC_->wdog = !wdog;
	}

	catch (Exception^) {
		return error_state::ERR_SM;
	}

	finally {
		if (lockTaken) {
			Monitor::Exit(SM_VC_->lockObject);
		}
	}

	if (!useCrashAvoidance) {
		inhibit = false;
	}

	return sendCommand(steer, speed, wdog ? 1 : 0, inhibit);
}

bool VC::getShutdownFlag() {
	return SM_TM_->shutdown & bit_VC;
}

void VC::selfShutDown() {
	SM_TM_->shutdown |= bit_VC;
}

void VC::threadFunction() {
	Console::WriteLine("Vehicle Control Thread Starting");

	error_state result = connect(WEEDER_ADDRESS, ports::VC);
	if (result != error_state::SUCCESS) {
		Console::WriteLine("VC Connection Error");
		selfShutDown();
	}

	else if (authenticate("5480558") != error_state::SUCCESS) {
		Console::WriteLine("VC Authentication Failure");
		selfShutDown();
	}

	SM_TM_->ThreadBarrier->SignalAndWait();
	VC_Stopwatch->Start();
	
	while (!getShutdownFlag()) {
		error_state response;
		try {
			response = processSharedMemory();
		}
		catch (Exception^) {
			selfShutDown();
		}

		if (response == error_state::SUCCESS) {
			processHeartBeats();
		}
		Thread::Sleep(25);
	}


	// Final Zero Command on Shutdown //

	error_state status = error_state::ERR_CONNECTION;

	if (Stream != nullptr && Client != nullptr && Client->Connected) {
		bool nextWdog = false;
		if (SM_VC_ != nullptr && SM_VC_->lockObject != nullptr) {
			bool lockTaken = false;
		
			try {
				Monitor::Enter(SM_VC_->lockObject, lockTaken);
				nextWdog = SM_VC_->wdog;
				SM_VC_->Speed = 0.0;
				SM_VC_->Steering = 0.0;
			}
			finally {
				if (lockTaken) {
					Monitor::Exit(SM_VC_->lockObject);
				}
			}
		}

		Thread::Sleep(50);

		for (int i = 0; i < 5; i++) {
			// Send multiple zero commands for redundancy
			status = sendCommand(0.0, 0.0, nextWdog ? 1: 0, false); 
			if (status != error_state::SUCCESS) {
				break;
			}
			nextWdog = !nextWdog;
			Thread::Sleep(50);
		}

		if (verifyConnection() == error_state::SUCCESS) {
			try {
				Client->Client->Shutdown(System::Net::Sockets::SocketShutdown::Send);
				Thread::Sleep(50);
			}
			catch (Exception^) {}
		}
	}

	Console::Write("Zeroed Weeder attempt status: ");
	printError(status);

	disconnect();
	Console::WriteLine("VC Thread Terminated");
}