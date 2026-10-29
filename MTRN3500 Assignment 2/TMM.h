#pragma once

#include <UGVModule.h>
#include "SMObjects.h"



ref class ThreadManagement : public UGVModule {
public:
    // Create shared memory objects
    error_state setupSharedMemory();

    // Send/Recieve data from shared memory structures
    error_state processSharedMemory() override;

    // Shutdown all modules in the software
    void shutdownModules();

    // Get Shutdown signal for module, from Thread Management SM
    bool getShutdownFlag() override;
    
    error_state processHeartBeats() override;

    // Thread function for TMM
    void threadFunction() override;

private:
    SM_Laser^ SM_LiDAR_;
    SM_GPS^ SM_GNSS_;
    SM_VehicleControl^ SM_VC_;
    SM_Display^ SM_Display_;

    array<Stopwatch^>^ StopwatchList;
    array<Thread^>^ ThreadList;
};
