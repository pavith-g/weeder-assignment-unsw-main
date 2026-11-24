#include "Controller.h"
#include "TMM.h"
#include "LiDAR.h"
#include "Display.h"
#include "GNSS.h"

error_state ThreadManagement::SetupSharedMemory() {
	// Create a thread for each module
	SM_TM_ = gcnew SM_ThreadManagement;
	SM_LiDAR_ = gcnew SM_Laser;
	SM_GNSS_ = gcnew SM_GPS;
	SM_VC_ = gcnew SM_VehicleControl;

	return error_state::SUCCESS;
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
	if (ThreadList == nullptr || ThreadPropertiesList == nullptr ||
		SM_TM_ == nullptr || SM_TM_->WatchList == nullptr) {
		return error_state::ERR_SM;
	}

	int count = ThreadList->Length;
	if (ThreadPropertiesList->Length < count || SM_TM_->WatchList->Length < count) {
		return error_state::ERR_SM;
	}

	for (int i = 0; i < count; i++) {
		if (ThreadPropertiesList[i] == nullptr || SM_TM_->WatchList[i] == nullptr) {
			continue;
		}

		unsigned int bitId = ThreadPropertiesList[i]->BitID;

		if ((SM_TM_->heartbeat & bitId) != 0) {
			// Heartbeat Received
			SM_TM_->heartbeat &= ~bitId;
			SM_TM_->WatchList[i]->Restart();
		}

		else if (SM_TM_->WatchList[i]->ElapsedMilliseconds > CRASH_LIMIT) {
			if ((bitId & CRITICALMASK) != 0) {
				Console::WriteLine(
					"{0} Critical Failure. Shutting down all threads",
					ThreadPropertiesList[i]->ThreadName
				);
				shutdownModules();
				return error_state::ERR_CRITICAL_THREAD_FAILURE;
			}
			else {
				Console::WriteLine(
					"{0} Failed. Restarting.",
					ThreadPropertiesList[i]->ThreadName
				);

				if (ThreadList[i] != nullptr && ThreadList[i]->IsAlive) {
					try {
						ThreadList[i]->Abort();
					}
					catch (Exception^) {}
				}

				SM_TM_->shutdown &= ~bitId;
				SM_TM_->WatchList[i]->Restart();

				ThreadList[i] = gcnew Thread(ThreadPropertiesList[i]->ThreadStart_);
				ThreadList[i]->Start();
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

			gcnew ThreadProperties
			(
				gcnew ThreadStart(gcnew Display(SM_TM_, SM_LiDAR_, SM_GNSS_),
					&Display::threadFunction),
				"Display Thread", bit_DISPLAY
			),
			gcnew ThreadProperties
			(
				gcnew ThreadStart(gcnew GNSS(SM_TM_, SM_GNSS_),
					&GNSS::threadFunction),
				"GNSS Thread", bit_GPS
			),

			gcnew ThreadProperties
			(
				gcnew ThreadStart(gcnew Controller(SM_TM_, SM_VC_),
					&Controller::threadFunction),
				"Controller Thread", bit_CONTROLLER
			)
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
				Console::WriteLine("Quit Key Pressed. Shutting down all modules");
				shutdownModules();
				break;
			}
		}
		// Console::WriteLine("TMM thread Running");
		processHeartBeats();
		Thread::Sleep(50);

	}
		
	Console::WriteLine("Joining Threads");

	// Join all threads
	for (int i = 0; i < ThreadPropertiesList->Length; i++)
	{
		if (ThreadList[i] != nullptr) {
			Console::WriteLine("Joining {0}...", ThreadPropertiesList[i]->ThreadName);
			if (!ThreadList[i]->Join(3000)) {
				Console::WriteLine("Warning: {0} did not exit within 3000 ms", ThreadPropertiesList[i]->ThreadName);
			}

		}
	}

	Console::WriteLine("TMM Terminated");
}



