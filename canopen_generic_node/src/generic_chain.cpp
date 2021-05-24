
#include <canopen_generic_node/generic_chain.h>
#include <socketcan_interface/xmlrpc_settings.h>

using namespace canopen;

GenericDeviceChain::GenericDeviceChain(const ros::NodeHandle &nh, const ros::NodeHandle &nh_priv) :
        RosChain(nh, nh_priv),
        generic_devices_allocator_("canopen_410", "canopen::GenericDeviceBase::Allocator"){}

bool GenericDeviceChain::nodeAdded(XmlRpc::XmlRpcValue &params, const canopen::NodeSharedPtr &node, const LoggerSharedPtr &logger)
{
    std::string name = params["name"];

    std::string device_type;
    if(params.hasMember("device_type")){
        device_type.assign(params["device_type"]);
    }
    else{
        ROS_ERROR_STREAM("device_param not defined");
        return 1;
    }

    if(device_type.compare("inclinometer") == 0){

        std::string alloc_name = "canopen::Inclinometer::Allocator";

        GenericDeviceBaseSharedPtr inclinometer;

        XmlRpcSettings settings;
        if(params.hasMember("inclinometer_layer")) settings = params["inclinometer_layer"];

        try{
            inclinometer = generic_devices_allocator_.allocateInstance(alloc_name, name + "_inclinometer", node->getStorage(), settings);
        }
        catch( const std::exception &e){
            std::string info = boost::diagnostic_information(e);
            ROS_ERROR_STREAM(info);
            return false;
        }

        if(!inclinometer){
            std::string error_msg = "Could not allocate inclinometer " + name;
            ROS_ERROR_STREAM(error_msg);
            return false;
        }

        generic_devices_->add(inclinometer);
        logger->add(inclinometer);
    }
    return true;
}

bool GenericDeviceChain::setup_chain() {
    printf("GenericDeviceChain setup_chain\n");

    generic_devices_.reset(new LayerGroupNoDiag<GenericDeviceBase>("410 Layer"));

    ros::Duration dur(0.0) ;

    if(RosChain::setup_chain()){
        add(generic_devices_);

        // if(!nh_.param("use_realtime_period", false)){
        //     dur.fromSec(boost::chrono::duration<double>(update_duration_).count());
        //     ROS_INFO_STREAM("Using fixed control period: " << dur);
        // }else{
        //     ROS_INFO("Using real-time control period");
        // }
        //cm_.reset(new ControllerManagerLayer(robot_layer_, nh_, dur));
        //add(cm_);

        return true;
    }

    return false;
}
