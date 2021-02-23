#include <class_loader/class_loader.hpp>
#include <canopen_401/inclinometer.h>

canopen::GenericDeviceBaseSharedPtr canopen::Inclinometer::Allocator::allocate(const std::string &name, canopen::ObjectStorageSharedPtr storage, const canopen::Settings &settings) {
    return std::make_shared<canopen::Inclinometer>(name, storage, settings);
}

CLASS_LOADER_REGISTER_CLASS(canopen::Inclinometer::Allocator, canopen::GenericDeviceBase::Allocator);
