#include "CrashAvoidance.h"

CrashAvoidance::CrashAvoidance(SM_ThreadManagement^ SM_TM, SM_VehicleControl^ SM_VC, SM_Laser^ SM_LiDAR) {
	SM_TM_ = SM_TM;
	SM_VC_ = SM_VC;
	SM_LiDAR_ = SM_LiDAR;
	CrashAvoidance_Stopwatch = gcnew Stopwatch;

	localX = gcnew array<double>(STANDARD_LASER_LENGTH);
	localY = gcnew array<double>(STANDARD_LASER_LENGTH);

	wasInhibited = false;
}

error_state CrashAvoidance::SetupSharedMemory() {
	return error_state::SUCCESS;
}

error_state CrashAvoidance::processHeartBeats() {
	if ((SM_TM_->heartbeat & bit_CRASHAVOIDANCE) == 0) {
		SM_TM_->heartbeat |= bit_CRASHAVOIDANCE;
		CrashAvoidance_Stopwatch->Restart();
	}

	return error_state::SUCCESS;
}

error_state CrashAvoidance::processSharedMemory() {
	if (SM_LiDAR_ == nullptr || SM_LiDAR_->lockObject == nullptr) {
		return error_state::ERR_SM;
	}

	if (SM_VC_ == nullptr || SM_VC_->lockObject == nullptr) {
		return error_state::ERR_SM;
	}

	// Update local lidar data
	bool lockTaken = false;
	try {
		Monitor::Enter(SM_LiDAR_->lockObject, lockTaken);
		Array::Copy(SM_LiDAR_->x, localX, STANDARD_LASER_LENGTH);
		Array::Copy(SM_LiDAR_->y, localY, STANDARD_LASER_LENGTH);
	}
	finally {
		if (lockTaken) {
			Monitor::Exit(SM_LiDAR_->lockObject);
		}
	}

	double localSpeed = 0.0;
	double localSteerDeg = 0.0;
	
	// Update local speed and steer data
	lockTaken = false;
	try {
		Monitor::Enter(SM_VC_->lockObject, lockTaken);
		localSpeed = SM_VC_->Speed;
		localSteerDeg = SM_VC_->Steering;
	}
	finally {
		if (lockTaken) {
			Monitor::Exit(SM_VC_->lockObject);
		}
	}

	bool inhibit = false;


	if (localSpeed > 0) {
		constexpr double ugvWidth = 0.70;
		constexpr double sideMargin = 0.10;
		constexpr double halfWidth = (ugvWidth * 0.5) + sideMargin;
		constexpr double wheelbase = 0.85; // Estimate 

		double baseDist = 1.0 + 0.2 * localSpeed;
		double stopDist = wasInhibited ? (baseDist + 0.05) : baseDist;

		constexpr double minRangeSq = 0.05 * 0.05;
		constexpr int minObstaclePoints = 3;

		double deltaRad = -localSteerDeg * (Math::PI / 180.0);
		bool isTurning = Math::Abs(localSteerDeg) >= 2;

		double R = isTurning ? (wheelbase / Math::Tan(deltaRad)) : 0.0;
		double absR = Math::Abs(R);
		double signR = (R >= 0.0) ? 1.0 : -1.0;

		int hitCount = 0;

		for (int i = 0; i < STANDARD_LASER_LENGTH; i++) {
			double x = localX[i] * 0.001; // mm to m
			double y = localY[i] * 0.001;

			if (Double::IsNaN(x) || Double::IsNaN(y) || x <= 0.0) {
				continue;
			}
			if ((x * x + y * y) < minRangeSq) {
				continue;
			}

			bool inHazard = false;

			if (!isTurning) {
				inHazard = (x <= stopDist) && (Math::Abs(y) <= halfWidth);
			}
			else {
				double dy = y - R;
				double rFromICR = Math::Sqrt(x * x + dy * dy);

				if (Math::Abs(rFromICR - absR) <= halfWidth) {
					double theta = Math::Atan2(x, absR - signR * y);
					double arcDist = absR * theta;

					if (theta > 0.0 && arcDist <= stopDist) {
						inHazard = true;
					}
				}
			}

			if (inHazard && ++hitCount >= minObstaclePoints) {
				inhibit = true;
				break;
			}
		}
	}

	wasInhibited = inhibit;
	lockTaken = false;
	try {
		Monitor::Enter(SM_VC_->lockObject, lockTaken);
		SM_VC_->inhibit = inhibit;
	}
	finally {
		if (lockTaken) {
			Monitor::Exit(SM_VC_->lockObject);
		}
	}

}

bool CrashAvoidance::getShutdownFlag() {
	return SM_TM_->shutdown & bit_CRASHAVOIDANCE;
}

void CrashAvoidance::selfShutDown() {
	SM_TM_->shutdown |= bit_CRASHAVOIDANCE;
}

void CrashAvoidance::threadFunction() {
	Console::WriteLine("Crash Avoidance Thread Starting");

	SM_TM_->ThreadBarrier->SignalAndWait();
	CrashAvoidance_Stopwatch->Start();

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
		Thread::Sleep(10);
	}
	Console::WriteLine("Crash Avoidance Thread Terminated");
}

