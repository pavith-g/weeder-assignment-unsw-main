#include "Controller.h"

using namespace System::Threading;

Controller::Controller(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC) {
	SM_TM_ = SM_TM;
	SM_VC_ = SM_VC;
	Controller_Stopwatch = gcnew Stopwatch;

	ControllerInterface* rawIntf = new ControllerInterface(1, 0);
	controllerIntfPtr = System::IntPtr(rawIntf);
}

Controller::~Controller() {
	this->!Controller();
}

Controller::!Controller() {
	if (controllerIntfPtr != System::IntPtr::Zero) {
		ControllerInterface* rawIntf = static_cast<ControllerInterface*>(controllerIntfPtr.ToPointer());
		delete rawIntf;
		controllerIntfPtr = System::IntPtr::Zero;
	}
}

error_state	Controller::SetupSharedMemory() {
	return error_state::SUCCESS;
}

error_state Controller::processHeartBeats() {
	if ((SM_TM_->heartbeat & bit_CONTROLLER) == 0) {
		SM_TM_->heartbeat |= bit_CONTROLLER;
		Controller_Stopwatch->Restart();
	}

	return error_state::SUCCESS;
}

double Controller::MapValue(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
	double output = (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
	return output;
}

error_state Controller::updateState() {
	if (checkConnection() != error_state::SUCCESS) {
		return error_state::ERR_CONNECTION;
	}
	ControllerInterface* intf = static_cast<ControllerInterface*>(controllerIntfPtr.ToPointer());
	controllerState state = intf->GetState();
	leftTriggerVal = state.leftTrigger;
	rightTriggerVal = state.rightTrigger;
	buttonX = state.buttonX;
	rightThumbX = state.rightThumbX;

	return error_state::SUCCESS;
}

error_state Controller::processSharedMemory() {
	double Speed = rightTriggerVal - leftTriggerVal;
	double tempSteer = -rightThumbX;
	double mappedSteer = MapValue(tempSteer, -1, 1, -40, 40);

	bool lockTaken = false;
	try {
		Monitor::Enter(SM_VC_->lockObject, lockTaken);

		SM_VC_->Speed = Speed;
		SM_VC_->Steering = mappedSteer;
	}
	finally {
		if (lockTaken) {
			Monitor::Exit(SM_VC_->lockObject);
		}
	}
	
	// Console::WriteLine("Written Speed: {0:F3}, Steer: {1:F3}", Speed, mappedSteer);
	return error_state::SUCCESS;
}

void Controller::zeroControl() {
	if (SM_VC_ == nullptr || SM_VC_->lockObject == nullptr) {
		return;
	}
	bool lockTaken = false;
	try {
		Monitor::Enter(SM_VC_->lockObject, lockTaken);
		SM_VC_->Speed = 0.0;
		SM_VC_->Steering = 0.0;
	}
	finally {
		if (lockTaken) {
			Monitor::Exit(SM_VC_->lockObject);
		}
	}
}

bool Controller::getShutdownFlag() {
	return SM_TM_->shutdown & bit_CONTROLLER;
}

void Controller::selfShutDown() {
	SM_TM_->shutdown |= bit_CONTROLLER;
}

error_state Controller::checkConnection() {
	ControllerInterface* intf = static_cast<ControllerInterface*>(controllerIntfPtr.ToPointer());
	if (intf == nullptr || !intf->IsConnected()) {
		Console::WriteLine("Controller Disconnected");
		return error_state::ERR_CONNECTION;
	}
	return error_state::SUCCESS;
	
}

void Controller::threadFunction() {
	Console::WriteLine("Controller Thread Starting");

	if (checkConnection() != error_state::SUCCESS) {
		zeroControl();
		selfShutDown();
	}

	SM_TM_->ThreadBarrier->SignalAndWait();
	Controller_Stopwatch->Start();

	try {
		while (!getShutdownFlag()) {
			error_state response;
			try {
				response = updateState();
			}

			catch (Exception^) {
				zeroControl();
				selfShutDown();
				break;
			}

			if (response == error_state::SUCCESS) {
				if (buttonX) {
					Console::WriteLine("Controller Shutdown Button Pressed");
					zeroControl();
					SM_TM_->shutdown = 0xFF;
					break;
				}

				processSharedMemory();
				processHeartBeats();

			}
			else {
				// Disconnect, set to zero
				zeroControl();

			}
			Thread::Sleep(10);
		}
	}

	finally {
		zeroControl();
		Console::WriteLine("Controller Thread Terminated");
	}
	
}