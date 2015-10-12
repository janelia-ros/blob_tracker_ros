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

#include "blob_tracker/Ellipse.h"
#include "blob_tracker/Blob.h"
#include "blob_tracker/Blobs.h"

namespace blob_tracker {

class ProcessImageNodelet : public nodelet::Nodelet
{
  // ROS communication
  boost::shared_ptr<image_transport::ImageTransport> it_in_,it_out_;
  image_transport::CameraSubscriber sub_;
  int queue_size_;

  boost::mutex connect_mutex_;
  image_transport::CameraPublisher image_pub_;
  ros::Publisher blobs_pub_;

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

  void configCb(Config &config,uint32_t level);
};

void ProcessImageNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  ros::NodeHandle nh_in (nh,"camera");
  ros::NodeHandle nh_out(nh,"blob_out");
  it_in_ .reset(new image_transport::ImageTransport(nh_in));
  it_out_.reset(new image_transport::ImageTransport(nh_out));

  // Read parameters
  private_nh.param("queue_size",queue_size_,5);

  std::string background_image_path;
  private_nh.getParam("background_image_path",background_image_path);
  ROS_WARN_STREAM("background_image_path " << background_image_path);

  // Set up dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_,private_nh));
  ReconfigureServer::CallbackType f = boost::bind(&ProcessImageNodelet::configCb,this,_1,_2);
  reconfigure_server_->setCallback(f);

  // Monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&ProcessImageNodelet::connectCb,this);
  ros::SubscriberStatusCallback connect_cb_info = boost::bind(&ProcessImageNodelet::connectCb,this);
  ros::SubscriberStatusCallback connect_cb_blobs = boost::bind(&ProcessImageNodelet::connectCb,this);
  // Make sure we don't enter connectCb() between advertising and assigning to image_pub_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  image_pub_ = it_out_->advertiseCamera("image_raw",1,connect_cb,connect_cb,connect_cb_info,connect_cb_info);
  blobs_pub_ = nh.advertise<Blobs>("blobs",1000,connect_cb_blobs,connect_cb_blobs);
}

// Handles (un)subscribing when clients (un)subscribe
void ProcessImageNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if ((image_pub_.getNumSubscribers() == 0) && (blobs_pub_.getNumSubscribers() == 0))
  {
    sub_.shutdown();
  }
  else if (!sub_)
  {
    image_transport::TransportHints hints("raw",ros::TransportHints(),getPrivateNodeHandle());
    sub_ = it_in_->subscribeCamera("image_raw",queue_size_,&ProcessImageNodelet::imageCb,this,hints);
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
  int morph_kernel_size = config.morph_kernel_size;
  int center_marker_radius = config.center_marker_radius;
  int drawn_line_thickness = config.drawn_line_thickness;
  bool draw_crosshairs = config.draw_crosshairs;

  // Get a cv::Mat view of the source data
  cv_bridge::CvImageConstPtr source_ptr = cv_bridge::toCvShare(image_msg,sensor_msgs::image_encodings::MONO8);

  // Threshold
  cv::Mat image_threshold;
  cv::threshold(source_ptr->image,image_threshold,threshold,255,cv::THRESH_BINARY);

  // Morphological Opening
  cv::Mat image_morph;
  cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph_kernel_size,morph_kernel_size));
  cv::morphologyEx(image_threshold,image_morph,cv::MORPH_OPEN,morph_kernel);


  // Find Contours
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(image_morph,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

  // Output Image
  cv::Mat image_output;
  cv::cvtColor(image_morph,image_output,CV_GRAY2BGR);

  // Draw Crosshairs
  if (draw_crosshairs)
  {
    int h = image_msg->height;
    int w = image_msg->width;
    cv::Scalar yellow(0,255,255);
    cv::line(image_output,cv::Point(0,h/2),cv::Point(w,h/2),yellow,drawn_line_thickness);
    cv::line(image_output,cv::Point(w/2,0),cv::Point(w/2,h),yellow,drawn_line_thickness);
  }

  // Draw Contours
  cv::Scalar blue(255,0,0);
  drawContours(image_output,contours,-1,blue,-1,8);

  // Calculate Blob Data
  Blobs blobs;
  blobs.header = image_msg->header;
  blobs.image_height = image_msg->height;
  blobs.image_width = image_msg->width;
  cv::Scalar red(0,0,255);
  cv::Scalar green(0,255,0);
  for(int i=0;i<contours.size();++i)
  {
    Blob blob;
    cv::Moments moments = cv::moments(contours[i]);
    if (moments.m00 > 0)
    {
      blob.x = moments.m10/moments.m00;
      blob.y = moments.m01/moments.m00;
      blob.area = moments.m00;
      cv::circle(image_output,cv::Point(blob.x,blob.y),center_marker_radius,red,-1);

      size_t count = contours[i].size();
      if(count < 6)
        continue;
      cv::Mat pointsf;
      cv::Mat(contours[i]).convertTo(pointsf, CV_32F);
      cv::RotatedRect box = cv::fitEllipse(pointsf);
      Ellipse ellipse;
      ellipse.x = box.center.x;
      ellipse.y = box.center.y;
      ellipse.width = box.size.width;
      ellipse.height = box.size.height;
      ellipse.angle = box.angle;
      blob.ellipse = ellipse;
      cv::ellipse(image_output,box,green,drawn_line_thickness);
      cv::Point2f vtx[4];
      box.points(vtx);
      for(int j=0; j<4; ++j)
      {
        cv::line(image_output,vtx[j],vtx[(j+1)%4],cv::Scalar(255,255,0),drawn_line_thickness);
      }
    }
    blobs.blobs.push_back(blob);
  }
  blobs_pub_.publish(blobs);

  cv_bridge::CvImage output;
  output.image = image_output;
  output.header = source_ptr->header;
  output.encoding = sensor_msgs::image_encodings::BGR8;

  sensor_msgs::ImagePtr out_image = output.toImageMsg();

  // Create updated CameraInfo message
  sensor_msgs::CameraInfoPtr out_info = boost::make_shared<sensor_msgs::CameraInfo>(*info_msg);

  image_pub_.publish(out_image,out_info);
}

void ProcessImageNodelet::configCb(Config &config,uint32_t level)
{
  config_ = config;
}

} // namespace blob_tracker

// Register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( blob_tracker::ProcessImageNodelet,nodelet::Nodelet)
