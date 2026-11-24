#include "Display.h"

Display::Display(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_LiDAR, SM_GPS^ SM_GNSS) {
	SM_TM_ = SM_TM;
	SM_LiDAR_ = SM_LiDAR;
	SM_GNSS_ = SM_GNSS;

	ReadData = gcnew array<unsigned char>(256);
	Display_Stopwatch = gcnew Stopwatch;
}

error_state Display::communicate() {
	if (verifyConnection() != error_state::SUCCESS) {
		return error_state::ERR_CONNECTION;
	}
	try {
		sendDisplayData();
		return error_state::SUCCESS;
	}
	catch (System::IO::IOException^) {
		disconnect();
		return error_state::ERR_CONNECTION;
	}

	catch (System::Net::Sockets::SocketException^) {
		disconnect();
		return error_state::ERR_CONNECTION;
	}
}


void Display::sendDisplayData() {

	if (Stream == nullptr) {
		throw gcnew System::IO::IOException("Display Stream Null");
	}

	Monitor::Enter(SM_LiDAR_->lockObject);

	array<Byte>^ dataX = gcnew array<Byte>(SM_LiDAR_->x->Length * sizeof(double));
	Buffer::BlockCopy(SM_LiDAR_->x, 0, dataX, 0, dataX->Length);

	array<Byte>^ dataY = gcnew array<Byte>(SM_LiDAR_->y->Length * sizeof(double));
	Buffer::BlockCopy(SM_LiDAR_->y, 0, dataY, 0, dataY->Length);

	Monitor::Exit(SM_LiDAR_->lockObject);

	Stream->Write(dataX, 0, dataX->Length);
	Thread::Sleep(10);
	Stream->Write(dataY, 0, dataY->Length);
}

error_state Display::processHeartBeats() {
	if ((SM_TM_->heartbeat & bit_DISPLAY) == 0) {
		SM_TM_->heartbeat |= bit_DISPLAY;
		Display_Stopwatch->Restart();
	}

	return error_state::SUCCESS;
}

error_state Display::SetupSharedMemory() {
	return error_state::SUCCESS;
}

error_state Display::processSharedMemory() {
	return error_state::SUCCESS;
}

bool Display::getShutdownFlag() {
	return SM_TM_->shutdown & bit_DISPLAY;
}

void Display::selfShutDown() {
	SM_TM_->shutdown |= bit_DISPLAY;
}

void Display::threadFunction() {
	Console::WriteLine("Display Thread Starting");

	error_state result = connect(DISPLAY_ADDRESS, ports::DISPLAY);

	if (result != error_state::SUCCESS) {
		Console::Write("Display Connection Error: ");
		printError(result);
		selfShutDown();
	}

	SM_TM_->ThreadBarrier->SignalAndWait();
	Display_Stopwatch->Start();
	
	while (!getShutdownFlag()) {
		error_state response;
		try {
			response = communicate();
		}
		catch (Exception^ ) {
			selfShutDown();
		}

		if (response == error_state::SUCCESS) {
			processHeartBeats();
		}
		Thread::Sleep(10);
	}


	Console::WriteLine("Display Thread Terminated");
}
