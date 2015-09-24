#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <cv_bridge/cv_bridge.h>
#include <blob_tracker/ProcessImageConfig.h>
#include <opencv2/imgproc/imgproc.hpp>

namespace blob_tracker {

class ProcessImageNodelet : public nodelet::Nodelet
{
  // ROS communication
  boost::shared_ptr<image_transport::ImageTransport> it_in_, it_out_;
  image_transport::CameraSubscriber sub_;
  int queue_size_;

  boost::mutex connect_mutex_;
  image_transport::CameraPublisher pub_;

  // Dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef blob_tracker::ProcessImageConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

  virtual void onInit();

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);

  void configCb(Config &config, uint32_t level);
};

void ProcessImageNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  ros::NodeHandle nh_in (nh, "camera");
  ros::NodeHandle nh_out(nh, "blob_out");
  it_in_ .reset(new image_transport::ImageTransport(nh_in));
  it_out_.reset(new image_transport::ImageTransport(nh_out));

  // Read parameters
  private_nh.param("queue_size", queue_size_, 5);

  // Set up dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_, private_nh));
  ReconfigureServer::CallbackType f = boost::bind(&ProcessImageNodelet::configCb, this, _1, _2);
  reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&ProcessImageNodelet::connectCb, this);
  ros::SubscriberStatusCallback connect_cb_info = boost::bind(&ProcessImageNodelet::connectCb, this);
  // Make sure we don't enter connectCb() between advertising and assigning to pub_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  pub_ = it_out_->advertiseCamera("image_raw",  1, connect_cb, connect_cb, connect_cb_info, connect_cb_info);
}

// Handles (un)subscribing when clients (un)subscribe
void ProcessImageNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if (pub_.getNumSubscribers() == 0)
    sub_.shutdown();
  else if (!sub_)
  {
    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
    sub_ = it_in_->subscribeCamera("image_raw", queue_size_, &ProcessImageNodelet::imageCb, this, hints);
  }
}

void ProcessImageNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
                                  const sensor_msgs::CameraInfoConstPtr& info_msg)
{
  Config config;
  {
    boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
    config = config_;
  }
  int threshold = config.threshold;

  // Get a cv::Mat view of the source data
  cv_bridge::CvImageConstPtr source_ptr = cv_bridge::toCvShare(image_msg, sensor_msgs::image_encodings::MONO8);

  // Threshold
  cv::Mat image_threshold = cv::Mat::zeros(source_ptr->image.size(), source_ptr->image.type());
  cv::threshold(source_ptr->image,image_threshold,threshold,255,cv::THRESH_BINARY);

  // Output Image
  cv::Mat image_output;
  cv::cvtColor(image_threshold,image_output,CV_GRAY2BGR);

  // Draw test circle
  cv::circle(image_output, cv::Point(512, 512), 50, cv::Scalar(255,255,0), -1);

  cv_bridge::CvImage output;
  output.image = image_output;
  output.header = source_ptr->header;
  output.encoding = sensor_msgs::image_encodings::BGR8;

  sensor_msgs::ImagePtr out_image = output.toImageMsg();

  // Create updated CameraInfo message
  sensor_msgs::CameraInfoPtr out_info = boost::make_shared<sensor_msgs::CameraInfo>(*info_msg);

  pub_.publish(out_image, out_info);
}

void ProcessImageNodelet::configCb(Config &config, uint32_t level)
{
  config_ = config;
}

} // namespace blob_tracker

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( blob_tracker::ProcessImageNodelet, nodelet::Nodelet)
