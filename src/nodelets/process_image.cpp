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
  /// @todo Check image dimensions match info_msg
  /// @todo Publish tweaks to config_ so they appear in reconfigure_gui

  Config config;
  {
    boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
    config = config_;
  }
  // int decimation_x = config.decimation_x;
  // int decimation_y = config.decimation_y;

  // // Compute the ROI we'll actually use
  // bool is_bayer = sensor_msgs::image_encodings::isBayer(image_msg->encoding);
  // if (is_bayer)
  // {
  //   // Odd offsets for Bayer images basically change the Bayer pattern, but that's
  //   // unnecessarily complicated to support
  //   config.x_offset &= ~0x1;
  //   config.y_offset &= ~0x1;
  //   config.width &= ~0x1;
  //   config.height &= ~0x1;
  // }

  // int max_width = image_msg->width - config.x_offset;
  // int max_height = image_msg->height - config.y_offset;
  // int width = config.width;
  // int height = config.height;
  // if (width == 0 || width > max_width)
  //   width = max_width;
  // if (height == 0 || height > max_height)
  //   height = max_height;

  // // On no-op, just pass the messages along
  // if (decimation_x == 1               &&
  //     decimation_y == 1               &&
  //     config.x_offset == 0            &&
  //     config.y_offset == 0            &&
  //     width  == (int)image_msg->width &&
  //     height == (int)image_msg->height)
  // {
  //   pub_.publish(image_msg, info_msg);
  //   return;
  // }

  // Get a cv::Mat view of the source data
  cv_bridge::CvImageConstPtr source = cv_bridge::toCvShare(image_msg);

  // We won't modify the data. We can safely share the data owned by the ROS message instead of copying.
  // cv_bridge::CvImage output(source->header, source->encoding);
  // output.image = source->image;
  // // Apply ROI (no copy, still a view of the image_msg data)
  // output.image = source->image(cv::Rect(config.x_offset, config.y_offset, width, height));

  // We want to modify the data in-place. We have to make a copy of the ROS message data.
  cv_bridge::CvImagePtr output;
  try
  {
    output = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // Draw an example circle on the video stream
  if (output->image.rows > 60 && output->image.cols > 60)
    cv::circle(output->image, cv::Point(512, 512), 20, cv::Scalar(255,0,0), -1);

  // Create output Image message
  /// @todo Could save copies by allocating this above and having output.image alias it
  // sensor_msgs::ImagePtr out_image = output.toImageMsg();
  sensor_msgs::ImagePtr out_image = output->toImageMsg();

  // Create updated CameraInfo message
  sensor_msgs::CameraInfoPtr out_info = boost::make_shared<sensor_msgs::CameraInfo>(*info_msg);
  // int binning_x = std::max((int)info_msg->binning_x, 1);
  // int binning_y = std::max((int)info_msg->binning_y, 1);
  // out_info->binning_x = binning_x * config.decimation_x;
  // out_info->binning_y = binning_y * config.decimation_y;
  // out_info->roi.x_offset += config.x_offset * binning_x;
  // out_info->roi.y_offset += config.y_offset * binning_y;
  // out_info->roi.height = height * binning_y;
  // out_info->roi.width = width * binning_x;
  // // If no ROI specified, leave do_rectify as-is. If ROI specified, set do_rectify = true.
  // if (width != (int)image_msg->width || height != (int)image_msg->height)
  //   out_info->roi.do_rectify = true;

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
