#include <ros/ros.h>
#include <signal.h>
#include <ros/xmlrpc_manager.h>

#include <toposens_driver/sensor.h>

// Signal-safe flag for whether shutdown is requested
volatile sig_atomic_t g_shutdown = 0;

// Replacement SIGINT handler. Called on Ctrl-C
void onSigint(int sig)
{
  g_shutdown = 1;
}

// Replacement "shutdown" XMLRPC callback. Called on rosnode kill.
void onShutdown(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
  int num_params = 0;
  if (params.getType() == XmlRpc::XmlRpcValue::TypeArray) num_params = params.size();
  if (num_params > 1)
  {
    std::string reason = params[1];
    ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
    g_shutdown = 1; // Set flag
  }
  result = ros::xmlrpc::responseInt(1, "", 0);
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "ts_driver_node", ros::init_options::NoSigintHandler);
  signal(SIGINT, onSigint); // Override SIGINT handler

  // Override XMLRPC shutdown
  ros::XMLRPCManager::instance()->unbind("shutdown");
  ros::XMLRPCManager::instance()->bind("shutdown", onShutdown);

  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  ros::Rate loop_rate(10); // 10 Hz


  // @todo: Exception should raise better message
  try {
    toposens_driver::Sensor d(nh, private_nh);

//    d.calibrate(t_celsius);

    while (!g_shutdown) {
      d.poll();
      ros::spinOnce();
      loop_rate.sleep();
    }

    d.shutdown();
  } catch (const char *msg) {
    ROS_ERROR("%s", msg);
  }
  
  ros::shutdown();
  return 0;
}
