#ifndef CANOPEN_402_MOTOR_H
#define CANOPEN_402_MOTOR_H

#include <canopen_410/base.h>
#include <canopen_master/canopen.h>
#include <functional>
#include <boost/container/flat_map.hpp>

#include <boost/numeric/conversion/cast.hpp>
#include <limits>
#include <algorithm>

#include <realtime_tools/realtime_publisher.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/Vector3.h>


namespace canopen
{

class Inclinometer : public GenericDeviceBase
{
    ros::NodeHandle nh_;

public:

    Inclinometer(const std::string &name, ObjectStorageSharedPtr storage, const canopen::Settings &settings)
    : GenericDeviceBase(name),
    publish_rate_(settings.get_optional<double>("publish_rate_", 100))
    {
        storage->entry(resolution_, 0x6000);
        storage->entry(slope_long_, 0x6010);
        storage->entry(slope_long_op_, 0x6011);
        storage->entry(slope_lont_preset_, 0x6012);
        storage->entry(slope_late_, 0x6020);
        storage->entry(slope_late_op_, 0x6021);
        storage->entry(slope_late_preset_, 0x6022);

        quat_msg = std::shared_ptr<geometry_msgs::QuaternionStamped>( new geometry_msgs::QuaternionStamped );
        deg_msg = std::shared_ptr<geometry_msgs::Vector3>( new geometry_msgs::Vector3 );
    }

    class Allocator : public GenericDeviceBase::Allocator{
    public:
        virtual GenericDeviceBaseSharedPtr allocate(const std::string &name, ObjectStorageSharedPtr storage, const canopen::Settings &settings);
    };

protected:
    virtual void handleRead(LayerStatus &status, const LayerState &current_state);
    virtual void handleWrite(LayerStatus &status, const LayerState &current_state){/* Unused */}
    virtual void handleDiag(LayerReport &report){/* Unused */}
    virtual void handleInit(LayerStatus &status);
    virtual void handleShutdown(LayerStatus &status){/* Unused */}
    virtual void handleHalt(LayerStatus &status){/* Unused */}
    virtual void handleRecover(LayerStatus &status){/* Unused */}

private:
    bool readState(LayerStatus &status, const LayerState &current_state);

    void init();
    void starting();
    void update();
    void stopping();

    double slope_long, slope_late;
    double roll, pitch, yaw;

    std::shared_ptr<ros::Publisher> inclinometer_pub_shptr;
    std::shared_ptr<geometry_msgs::QuaternionStamped> quat_msg;
    std::shared_ptr<geometry_msgs::Vector3> deg_msg;

    ros::Publisher inclinometer_quat_pub_;
    ros::Publisher inclinometer_deg_pub_;

    ros::Time last_publish_time_;
    double publish_rate_;

    boost::mutex map_mutex_;

    canopen::ObjectStorage::Entry<uint16_t>  resolution_;
    canopen::ObjectStorage::Entry<int16_t>   slope_long_;
    canopen::ObjectStorage::Entry<uint8_t>   slope_long_op_;
    canopen::ObjectStorage::Entry<int16_t>   slope_lont_preset_;
    canopen::ObjectStorage::Entry<int16_t>   slope_late_;
    canopen::ObjectStorage::Entry<uint8_t>   slope_late_op_;
    canopen::ObjectStorage::Entry<int16_t>   slope_late_preset_; 
};

}

#endif
