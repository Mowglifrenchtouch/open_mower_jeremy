#pragma once

#include <memory>
#include "MowerServiceInterfaceMAVROS.h"

class EmergencyServiceInterfaceMAVROS {
public:
    explicit EmergencyServiceInterfaceMAVROS(std::shared_ptr<MowerServiceInterfaceMAVROS> mower);

    void updateEmergencyStatus(bool emergency);

private:
    std::shared_ptr<MowerServiceInterfaceMAVROS> mower_;
};
