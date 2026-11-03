#pragma once

#include <UGVModule.h>
#include "SMObjects.h"


ref struct ThreadProperties
{
    ThreadStart^ ThreadStart_;
    // bool Critical;
    String^ ThreadName;
    uint8_t BitID;

    ThreadProperties(ThreadStart^ start, String^ name, uint8_t bitid)
    {
        ThreadStart_ = start;
        // Critical = critical;
        ThreadName = name;
        BitID = bitid;
    }
};

ref class ThreadManagement : public UGVModule {
public:
    // Create shared memory objects
    error_state SetupSharedMemory() override;

    // Send/Recieve data from shared memory structures
    error_state processSharedMemory() override;

    // Shutdown all modules in the software
    void shutdownModules();

    // Get Shutdown signal for module, from Thread Management SM
    bool getShutdownFlag() override;
    
    error_state processHeartBeats() override;

    void outputDataToTerminal() override;

    // Thread function for TMM
    void threadFunction() override;

    void selfShutDown() override;

private:
    SM_Laser^ SM_LiDAR_;
    SM_GPS^ SM_GNSS_;
    SM_VehicleControl^ SM_VC_;
    SM_Display^ SM_Display_;

    array<Thread^>^ ThreadList;
    array<ThreadProperties^>^ ThreadPropertiesList;
};
