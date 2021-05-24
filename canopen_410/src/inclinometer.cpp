#include <canopen_410/inclinometer.h>
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
    if(current_state == Ready)
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
    std::string inclinometer_quat_name = "inclinometer_quat";
    std::string inclinometer_deg_name = "inclinometer_deg";
    
    inclinometer_quat_pub_ = nh_.advertise<geometry_msgs::QuaternionStamped>(inclinometer_quat_name,4);
    inclinometer_deg_pub_ = nh_.advertise<geometry_msgs::Vector3>(inclinometer_deg_name,4);

    ros::Time time = ros::Time::now();
    tf2::Quaternion quat;
    quat.setRPY(0,0,0);

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
            volatile int16_t slope_long, slope_late;
            volatile uint16_t resolution;
            resolution = resolution_.get();
            slope_long = slope_long_.get();
            slope_late = slope_late_.get();

            //TODO: Set the inclination according to the orientation.
            double roll_deg, pitch_deg, yaw_deg;
            roll_deg = slope_long/(resolution*10.0);                
            pitch_deg = slope_late/(resolution*10.0);               
            yaw_deg = 0;         

            double roll_rad, pitch_rad, yaw_rad;
            roll_rad = roll_deg*2*M_PI/180.0;                
            pitch_rad = pitch_deg*2*M_PI/180.0;            
            yaw_rad = 0;  

            tf2::Quaternion quat;
            quat.setRPY(roll_rad, pitch_rad, yaw_rad);

            // populate inclinator message:
            quat_msg->header.stamp = time;
            quat_msg->quaternion.x = quat.getX();
            quat_msg->quaternion.y = quat.getY();
            quat_msg->quaternion.z = quat.getZ();
            quat_msg->quaternion.w = quat.getW();

            deg_msg->x = (float)roll_deg;
            deg_msg->y = (float)pitch_deg;
            deg_msg->z = (float)yaw_deg;
            
            inclinometer_quat_pub_.publish((*quat_msg));
            inclinometer_deg_pub_.publish((*deg_msg));
        //}
    }
}

} // namespace
