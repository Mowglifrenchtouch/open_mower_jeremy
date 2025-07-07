#include "EmergencyServiceInterfaceMAVROS.h"
#include "MowerServiceInterfaceMAVROS.h"

EmergencyServiceInterface::EmergencyServiceInterface(std::shared_ptr<MowerServiceInterface> mower)
    : mower_(std::move(mower)) {}

void EmergencyServiceInterface::updateEmergencyStatus(bool emergency) {
    if (mower_) {
        mower_->setEmergency(emergency);
    }
}
