#include <ros/ros.h>
#include <nodelet/loader.h>
#include <blob_tracker/advertisement_checker.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "blob_tracker");

  // Check for common user errors
  if (ros::names::remap("camera") != "camera")
  {
    ROS_WARN("Remapping 'camera' has no effect! Start blob_tracker in the "
             "camera namespace instead.\nExample command-line usage:\n"
             "\t$ ROS_NAMESPACE=%s rosrun blob_tracker blob_tracker",
             ros::names::remap("camera").c_str());
  }
  if (ros::this_node::getNamespace() == "/")
  {
    ROS_WARN("Started in the global namespace! This is probably wrong. Start blob_tracker "
             "in the camera namespace.\nExample command-line usage:\n"
             "\t$ ROS_NAMESPACE=my_camera rosrun blob_tracker blob_tracker");
  }

  // Shared parameters to be propagated to nodelet private namespaces
  ros::NodeHandle private_nh("~");
  XmlRpc::XmlRpcValue shared_params;
  int queue_size;
  if (private_nh.getParam("queue_size", queue_size))
    shared_params["queue_size"] = queue_size;

  nodelet::Loader manager(false); // Don't bring up the manager ROS API
  nodelet::M_string remappings;
  nodelet::V_string my_argv;

  // Debayer nodelet, image_raw -> image_mono, image_color
  std::string debayer_name = ros::this_node::getName() + "_debayer";
  manager.load(debayer_name, "blob_tracker/debayer", remappings, my_argv);

  // Rectify nodelet, image_mono -> image_rect
  std::string rectify_mono_name = ros::this_node::getName() + "_rectify_mono";
  if (shared_params.valid())
    ros::param::set(rectify_mono_name, shared_params);
  manager.load(rectify_mono_name, "blob_tracker/rectify", remappings, my_argv);

  // Rectify nodelet, image_color -> image_rect_color
  // NOTE: Explicitly resolve any global remappings here, so they don't get hidden.
  remappings["image_mono"] = ros::names::resolve("image_color");
  remappings["image_rect"] = ros::names::resolve("image_rect_color");
  std::string rectify_color_name = ros::this_node::getName() + "_rectify_color";
  if (shared_params.valid())
    ros::param::set(rectify_color_name, shared_params);
  manager.load(rectify_color_name, "blob_tracker/rectify", remappings, my_argv);

  // Check for only the original camera topics
  ros::V_string topics;
  topics.push_back(ros::names::resolve("image_raw"));
  topics.push_back(ros::names::resolve("camera_info"));
  blob_tracker::AdvertisementChecker check_inputs(ros::NodeHandle(), ros::this_node::getName());
  check_inputs.start(topics, 60.0);
  
  ros::spin();
  return 0;
}
