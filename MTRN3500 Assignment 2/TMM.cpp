#include "TMM.h"
#include "LiDAR.h"



error_state ThreadManagement::SetupSharedMemory() {
	// Create a thread for each module
	SM_TM_ = gcnew SM_ThreadManagement;
	SM_LiDAR_ = gcnew SM_Laser;
	SM_GNSS_ = gcnew SM_GPS;
	SM_Display_ = gcnew SM_Display;
	SM_VC_ = gcnew SM_VehicleControl;

	return error_state::SUCCESS;
}

void ThreadManagement::outputDataToTerminal() {

}
	
error_state ThreadManagement::processSharedMemory() {
	return error_state::SUCCESS;
}

void ThreadManagement::shutdownModules() {
	SM_TM_->shutdown = 0xFF;
}

bool ThreadManagement::getShutdownFlag() {
	return SM_TM_->shutdown & bit_TM;
}

void ThreadManagement::selfShutDown() {
	return;
}

error_state ThreadManagement::processHeartBeats() {
	for (int i = 0; i < ThreadList->Length; i++)
	{
		if (SM_TM_->heartbeat & ThreadPropertiesList[i]->BitID)
		{
			// High, put to low
			SM_TM_->heartbeat &= ~ThreadPropertiesList[i]->BitID;
			// Restart stopwatch
			SM_TM_->WatchList[i]->Restart();
		}

		else
		{
			if (SM_TM_->WatchList[i]->ElapsedMilliseconds > CRASH_LIMIT)
			{
				// Check if critical
				if (ThreadPropertiesList[i]->BitID & CRITICALMASK)
				{
					Console::WriteLine(ThreadPropertiesList[i]->ThreadName + " (critical)failure.Shutting down all threads");
					shutdownModules();
					return error_state::ERR_CRITICAL_THREAD_FAILURE;
				}
				else
				{
					Console::WriteLine(ThreadPropertiesList[i]->ThreadName + " failed...Restarting");
					ThreadList[i]->Abort();
					ThreadList[i] = gcnew Thread(ThreadPropertiesList[i]->ThreadStart_);
					SM_TM_->ThreadBarrier = gcnew Barrier(1);
					ThreadList[i]->Start();
				}
			}
		}
	}
	return error_state::SUCCESS;
}

void ThreadManagement::threadFunction() {
	// Main Thread Function

	Console::WriteLine("Thread Management Starting");

	// Create Thread Properties List

	ThreadPropertiesList = gcnew array<ThreadProperties^>
	{
		gcnew ThreadProperties
		(
			gcnew ThreadStart(gcnew LiDAR(SM_TM_, SM_LiDAR_),
				&LiDAR::threadFunction),
			"LiDAR Thread", bit_LASER
		),
			// Add Other Threads Later
	};

	// Create Thread List
	ThreadList = gcnew array<Thread^>(ThreadPropertiesList->Length);

	// Create Stopwatch List
	SM_TM_->WatchList = gcnew array<Stopwatch^>(ThreadPropertiesList->Length);

	// Create Barrier List
	SM_TM_->ThreadBarrier = gcnew Barrier(ThreadPropertiesList->Length + 1);


	// Start Threads
	for (int i = 0; i < ThreadPropertiesList->Length; i++)
	{
		SM_TM_->WatchList[i] = gcnew Stopwatch;
		ThreadList[i] = gcnew Thread(ThreadPropertiesList[i]->ThreadStart_);
		ThreadList[i]->Start();
	}

	// wait at TMM barrier

	SM_TM_->ThreadBarrier->SignalAndWait();

	// Start Stopwatches
	for (int i = 0; i < ThreadList->Length; i++)
	{
		SM_TM_->WatchList[i]->Start();
	}


	// Start thread loop. Check Heartbeats
	while (!getShutdownFlag()) {
		if (Console::KeyAvailable) {
			ConsoleKeyInfo^ keyInfo = Console::ReadKey();

			if (keyInfo->Key == ConsoleKey::Q) {
				shutdownModules();
			}
		}
		// Console::WriteLine("TMM thread Running");
		processHeartBeats();
		Thread::Sleep(50);

	}


	shutdownModules();

	// Join all threads
	for (int i = 0; i < ThreadPropertiesList->Length; i++)
	{
		ThreadList[i]->Join();
	}

	Console::WriteLine("TMM Terminated");
	while (1);
}



