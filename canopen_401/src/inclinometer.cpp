#include <canopen_401/inclinometer.h>
#include <boost/thread/reverse_lock.hpp>
#include <tf2/LinearMath/Quaternion.h>

namespace canopen
{

void Inclinometer::handleInit(LayerStatus &status){
    
    if(!readState(status, Init)){
        status.error("Could not read inclinometer state");
        return;
    }

    //Configure canopen inclinometer objects. From EDS?


    init();
}

void Inclinometer::handleRead(LayerStatus &status, const LayerState &current_state)
{
    if(current_state < Shutdown)
    {
        update();
    }
}

bool Inclinometer::readState(LayerStatus &status, const LayerState &current_state){
    //TODO: Include error cases and internal limits.
    return true;
}

void Inclinometer::init()
{
    // realtime publisher
    std::string inclinometer_topic_name = "inclinometer_" + device_name_ + "quaternion";
    inclinometer_pub_ = nh_.advertise<geometry_msgs::QuaternionStamped>(inclinometer_topic_name,4);

    ros::Time time = ros::Time::now();
    tf2::Quaternion quat;
    quat.setRPY(0,0,0);
    
    quat_msg->header.stamp = time;
    quat_msg->quaternion.x = quat.getX();
    quat_msg->quaternion.y = quat.getY();
    quat_msg->quaternion.z = quat.getZ();
    quat_msg->quaternion.w = quat.getW();

    last_publish_time_ = time;

}

void Inclinometer::update()
{
    // limit rate of publishing
    ros::Time time = ros::Time::now();

    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time){

        // try to publish
        //if (inclinometer_pub_->trylock()){
            // we're actually publishing, so increment time
            last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);

            //get roll, pitch & yaw from canopen node.
            double slope_long, slope_late;
            slope_long = slope_long_.get();
            slope_late = slope_late_.get();

            //TODO: Set the inclination according to the orientation.
            double roll, pitch, yaw;
            roll = slope_long;
            pitch = slope_late;
            yaw = 0;

            tf2::Quaternion quat;
            quat.setRPY(roll,pitch,yaw);

            // populate inclinator message:
            quat_msg->header.stamp = time;
            quat_msg->quaternion.x = quat.getX();
            quat_msg->quaternion.y = quat.getY();
            quat_msg->quaternion.z = quat.getZ();
            quat_msg->quaternion.w = quat.getW();
            
            inclinometer_pub_.publish((*quat_msg));
        //}
    }
}

} // namespace
