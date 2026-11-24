#include "Display.h"

Display::Display(SM_ThreadManagement^ SM_TM, SM_Laser^ SM_LiDAR, SM_GPS^ SM_GNSS) {
	SM_TM_ = SM_TM;
	SM_LiDAR_ = SM_LiDAR;
	SM_GNSS_ = SM_GNSS;

	Display_Stopwatch = gcnew Stopwatch;
}


// TODO: Move Connect function to parent
error_state Display::connect(String^ hostName, int portNumber) {
	Console::WriteLine("Attempting to connect to Display");
	disconnect();

	ReadData = gcnew array<unsigned char>(2048);

	try
	{
		Client = gcnew TcpClient();

		Client->ReceiveBufferSize = 2048;
		Client->ReceiveTimeout = 500;
		Client->NoDelay = true;

		IAsyncResult^ asyncResult = Client->BeginConnect(hostName, portNumber, nullptr, nullptr);
		bool connected = asyncResult->AsyncWaitHandle->WaitOne(1500, false);

		if (!connected)
		{
			disconnect();
			return error_state::ERR_CONNECTION;
		}

		Client->EndConnect(asyncResult);


		if (!Client->Connected)
		{
			disconnect();
			return error_state::ERR_STARTUP;
		}

		Stream = Client->GetStream();
		ReadData = gcnew array<unsigned char>(Client->ReceiveBufferSize);
		return error_state::SUCCESS;
	}

	catch (Exception^)
	{
		disconnect();
		return error_state::ERR_STARTUP;
	}
}

error_state Display::communicate() {
	sendDisplayData();
	return error_state::SUCCESS;
}


void Display::sendDisplayData() {
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
		sendDisplayData();
		processHeartBeats();
		Thread::Sleep(10);
	}
	Console::WriteLine("Display Thread Terminated");
}
