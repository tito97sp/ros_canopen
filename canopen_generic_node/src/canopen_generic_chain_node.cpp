#include <canopen_generic_node/generic_chain.h>

using namespace canopen;


int main(int argc, char** argv){
  ros::init(argc, argv, "canopen_generic_chain_node");
  ros::AsyncSpinner spinner(0);
  spinner.start();

  ros::NodeHandle nh;
  ros::NodeHandle nh_priv("~");

  GenericDeviceChain chain(nh, nh_priv);

  if(!chain.setup()){
      return 1;
  }

  ros::waitForShutdown();
  return 0;
}
