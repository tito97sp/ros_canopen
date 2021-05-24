#ifndef CANOPEN_402_BASE_H
#define CANOPEN_402_BASE_H

#include <canopen_master/canopen.h>

namespace canopen
{

class GenericDeviceBase : public canopen::Layer {
protected:
    GenericDeviceBase(const std::string &name) : Layer(name), device_name_(name) {}
public:
    std::string device_name_;

    typedef std::shared_ptr<GenericDeviceBase> GenericDeviceBaseSharedPtr;

    class Allocator {
    public:
        virtual GenericDeviceBaseSharedPtr allocate(const std::string &name, ObjectStorageSharedPtr storage, const canopen::Settings &settings) = 0;
        virtual ~Allocator() {}
    };
};

typedef GenericDeviceBase::GenericDeviceBaseSharedPtr GenericDeviceBaseSharedPtr;

}

#endif
