#pragma once

#include <memory>
#include "MowerServiceInterfaceMAVROS.h"

class EmergencyServiceInterface {
public:
    EmergencyServiceInterface(std::shared_ptr<MowerServiceInterface> mower);

    void updateEmergencyStatus(bool emergency);

private:
    std::shared_ptr<MowerServiceInterface> mower_;
};
