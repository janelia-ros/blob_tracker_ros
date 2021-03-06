#include <boost/version.hpp>
#if ((BOOST_VERSION / 100) % 1000) >= 53
#include <boost/thread/mutex.hpp>
#include <boost/thread/lock_guard.hpp>
#endif

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <cv_bridge/cv_bridge.h>
#include <blob_tracker/ProcessImageConfig.h>

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/videoio.hpp"
#include <opencv2/highgui.hpp>
#include <opencv2/video.hpp>

#include <ros/console.h>

#include "std_msgs/Empty.h"

#include "blob_tracker/Ellipse.h"
#include "blob_tracker/Blob.h"
#include "blob_tracker/Blobs.h"


namespace blob_tracker {

class ProcessImageNodelet : public nodelet::Nodelet
{
  // ROS communication
  boost::shared_ptr<image_transport::ImageTransport> it_in_,it_out_;
  image_transport::Subscriber image_sub_;
  int queue_size_;
  cv_bridge::CvImageConstPtr source_ptr_;

  // boost::mutex connect_mutex_;
  boost::mutex callback_mutex_;
  image_transport::Publisher image_pub_;
  ros::Publisher blobs_pub_;

  // background subtraction
  cv::Ptr<cv::BackgroundSubtractor> bg_subtr_ptr_;
  cv::Mat image_foreground_;
  image_transport::Publisher foreground_pub_;

  // threshold
  double threshold_;
  ros::Subscriber find_otsu_threshold_sub_;

  // dynamic reconfigure
  boost::recursive_mutex config_mutex_;
  typedef blob_tracker::ProcessImageConfig Config;
  typedef dynamic_reconfigure::Server<Config> ReconfigureServer;
  boost::shared_ptr<ReconfigureServer> reconfigure_server_;
  Config config_;

  virtual void onInit();

  // void connectCb();

  // void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
  //              const sensor_msgs::CameraInfoConstPtr& info_msg);

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg);

  void configCb(Config &config,uint32_t level);

  void findOtsuThresholdCallback(const std_msgs::EmptyConstPtr& message);

};

void ProcessImageNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  // ros::NodeHandle nh_in (nh,"camera");
  // ros::NodeHandle nh_in (nh);
  ros::NodeHandle nh_out(nh,"blob_out");
  // it_in_.reset(new image_transport::ImageTransport(nh_in));
  it_in_.reset(new image_transport::ImageTransport(nh));
  it_out_.reset(new image_transport::ImageTransport(nh_out));

  // read parameters
  private_nh.param("queue_size",queue_size_,5);

  // set up dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_,private_nh));
  ReconfigureServer::CallbackType f = boost::bind(&ProcessImageNodelet::configCb,this,_1,_2);
  reconfigure_server_->setCallback(f);

  bg_subtr_ptr_ = cv::createBackgroundSubtractorMOG2();

  // threshold default
  threshold_ = 20;

  // monitor whether anyone is subscribed to the output
  // image_transport::SubscriberStatusCallback connect_cb = boost::bind(&ProcessImageNodelet::connectCb,this);
  // ros::SubscriberStatusCallback connect_cb_info = boost::bind(&ProcessImageNodelet::connectCb,this);
  // ros::SubscriberStatusCallback connect_cb_blobs = boost::bind(&ProcessImageNodelet::connectCb,this);
  // make sure we don't enter connectCb() between advertising and assigning to image_pub_
  // boost::lock_guard<boost::mutex> lock(connect_mutex_);
  image_pub_ = it_out_->advertise("image_raw",1);
  foreground_pub_ = it_out_->advertise("foreground_raw",1);
  // blobs_pub_ = nh.advertise<Blobs>("blobs",1000,connect_cb_blobs,connect_cb_blobs);
  blobs_pub_ = nh.advertise<Blobs>("blobs",1000);
  find_otsu_threshold_sub_ = nh.subscribe("find_otsu_threshold",1,&ProcessImageNodelet::findOtsuThresholdCallback,this);

  image_sub_ = it_in_->subscribe("image_raw",queue_size_,&ProcessImageNodelet::imageCb,this);
}

// handles (un)subscribing when clients (un)subscribe
// void ProcessImageNodelet::connectCb()
// {
//   boost::lock_guard<boost::mutex> lock(connect_mutex_);
//   if ((image_pub_.getNumSubscribers() == 0) && (blobs_pub_.getNumSubscribers() == 0))
//   {
//     ROS_WARN_STREAM("shutting down subscription to image_raw");
//     image_sub_.shutdown();
//   }
//   else if (!image_sub_)
//   {
//     // read background image if one exists
//     // image_background_ = cv::imread(background_image_path_,CV_LOAD_IMAGE_GRAYSCALE);

//     // image_transport::TransportHints hints("raw",ros::TransportHints(),getPrivateNodeHandle());
//     // image_sub_ = it_in_->subscribeCamera("image_raw",queue_size_,&ProcessImageNodelet::imageCb,this,hints);
//     // image_sub_ = it_in_->subscribe("image_raw",queue_size_,&ProcessImageNodelet::imageCb,this);
//   }
// }

// void ProcessImageNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg,
//                                   const sensor_msgs::CameraInfoConstPtr& info_msg)
void ProcessImageNodelet::imageCb(const sensor_msgs::ImageConstPtr& image_msg)
{
  Config config;
  {
    boost::lock_guard<boost::recursive_mutex> lock(config_mutex_);
    config = config_;
  }
  threshold_ = config.threshold;
  int morph_kernel_size = config.morph_kernel_size;

  int blob_count_max = config.blob_count_max;

  bool draw_blobs = config.draw_blobs;
  bool draw_contours = config.draw_contours;

  int drawn_line_thickness = config.drawn_line_thickness;
  bool draw_crosshairs = config.draw_crosshairs;
  bool draw_ellipse = config.draw_ellipse;
  bool draw_box = config.draw_box;

  int center_marker_radius = config.center_marker_radius;
  bool draw_center_marker = config.draw_center_marker;

  // get a cv::Mat view of the source data
  {
    boost::lock_guard<boost::mutex> lock(callback_mutex_);
    source_ptr_ = cv_bridge::toCvShare(image_msg,sensor_msgs::image_encodings::MONO8);
  }

  // update background
  bg_subtr_ptr_->apply(source_ptr_->image,image_foreground_);

  // threshold
  cv::Mat image_threshold;
  cv::threshold(image_foreground_,image_threshold,threshold_,255,cv::THRESH_BINARY);

  // morphological opening
  cv::Mat image_morph;
  cv::Mat morph_kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(morph_kernel_size,morph_kernel_size));
  cv::morphologyEx(image_threshold,image_morph,cv::MORPH_OPEN,morph_kernel);


  // find contours
  std::vector<std::vector<cv::Point> > contours;
  cv::findContours(image_morph,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE);

  // output image
  cv::Mat image_output;
  if (draw_blobs)
  {
    cv::cvtColor(image_morph,image_output,CV_GRAY2BGR);
  }
  else
  {
    image_output = cv::Mat::zeros(image_morph.size(),CV_8UC3);
  }


  // colors
  cv::Scalar blue(255,0,0);
  cv::Scalar yellow(0,255,255);
  cv::Scalar red(0,0,255);
  cv::Scalar green(0,255,0);

  // draw crosshairs
  if (draw_crosshairs)
  {
    int h = image_msg->height;
    int w = image_msg->width;
    cv::line(image_output,cv::Point(0,h/2),cv::Point(w,h/2),yellow,drawn_line_thickness);
    cv::line(image_output,cv::Point(w/2,0),cv::Point(w/2,h),yellow,drawn_line_thickness);
  }

  // calculate blob data
  Blobs blobs;
  blobs.header = image_msg->header;
  blobs.image_height = image_msg->height;
  blobs.image_width = image_msg->width;

  if (blob_count_max > contours.size())
  {
    blob_count_max = contours.size();
  }
  for(int i=0; i<blob_count_max; ++i)
  {
    Blob blob;
    cv::Moments moments = cv::moments(contours[i]);
    if (moments.m00 > 0)
    {
      size_t count = contours[i].size();
      if(count < 6)
        continue;
      if (draw_contours)
      {
        drawContours(image_output,contours,i,blue,-1,8);
      }
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
      if (draw_ellipse)
      {
        cv::ellipse(image_output,box,green,drawn_line_thickness);
      }
      if (draw_box)
      {
        cv::Point2f vtx[4];
        box.points(vtx);
        for(int j=0; j<4; ++j)
        {
          cv::line(image_output,vtx[j],vtx[(j+1)%4],cv::Scalar(255,255,0),drawn_line_thickness);
        }
      }

      if (draw_center_marker)
      {
        blob.x = moments.m10/moments.m00;
        blob.y = moments.m01/moments.m00;
        blob.area = moments.m00;
        cv::circle(image_output,cv::Point(blob.x,blob.y),center_marker_radius,red,-1);
      }
    }
    blobs.blobs.push_back(blob);
  }
  blobs_pub_.publish(blobs);

  sensor_msgs::ImagePtr image_msg_ptr = cv_bridge::CvImage(std_msgs::Header(),"bgr8",image_output).toImageMsg();
  image_pub_.publish(image_msg_ptr);

  cv::Mat image_foreground;
  cv::cvtColor(image_foreground_,image_foreground,CV_GRAY2BGR);
  sensor_msgs::ImagePtr foreground_msg_ptr = cv_bridge::CvImage(std_msgs::Header(),"bgr8",image_foreground).toImageMsg();
  foreground_pub_.publish(foreground_msg_ptr);
}

void ProcessImageNodelet::configCb(Config &config,uint32_t level)
{
  config_ = config;
}

void ProcessImageNodelet::findOtsuThresholdCallback(const std_msgs::EmptyConstPtr& message)
{
  boost::lock_guard<boost::mutex> lock(callback_mutex_);
  cv::Mat image_threshold;
  threshold_ = cv::threshold(source_ptr_->image,image_threshold,0,255,cv::THRESH_BINARY+cv::THRESH_OTSU);
}

} // namespace blob_tracker

// register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( blob_tracker::ProcessImageNodelet,nodelet::Nodelet)
