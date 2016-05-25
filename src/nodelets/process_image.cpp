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
#include <opencv2/highgui/highgui.hpp>

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
  image_transport::CameraSubscriber cam_sub_;
  int queue_size_;
  cv_bridge::CvImageConstPtr source_ptr_;

  boost::mutex connect_mutex_;
  boost::mutex callback_mutex_;
  image_transport::CameraPublisher image_pub_;
  ros::Publisher blobs_pub_;

  // background image
  ros::Subscriber save_background_sub_;
  std::string background_image_path_;
  cv::Mat image_background_;
  cv::Mat image_foreground_;

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

  void connectCb();

  void imageCb(const sensor_msgs::ImageConstPtr& image_msg,
               const sensor_msgs::CameraInfoConstPtr& info_msg);

  void configCb(Config &config,uint32_t level);

  void saveBackgroundImageCb(const std_msgs::EmptyConstPtr& msg);

  void findOtsuThresholdCallback(const std_msgs::EmptyConstPtr& message);

};

void ProcessImageNodelet::onInit()
{
  ros::NodeHandle& nh = getNodeHandle();
  ros::NodeHandle& private_nh = getPrivateNodeHandle();
  ros::NodeHandle nh_in (nh,"camera");
  ros::NodeHandle nh_out(nh,"blob_out");
  it_in_.reset(new image_transport::ImageTransport(nh_in));
  it_out_.reset(new image_transport::ImageTransport(nh_out));

  // read parameters
  private_nh.param("queue_size",queue_size_,5);

  // set up dynamic reconfigure
  reconfigure_server_.reset(new ReconfigureServer(config_mutex_,private_nh));
  ReconfigureServer::CallbackType f = boost::bind(&ProcessImageNodelet::configCb,this,_1,_2);
  reconfigure_server_->setCallback(f);

  // background image
  ros::NodeHandle& private_nh_mt = getMTPrivateNodeHandle();
  private_nh.param<std::string>("background_image_path",background_image_path_,"background.png");

  // threshold default
  threshold_ = 20;

  // monitor whether anyone is subscribed to the output
  image_transport::SubscriberStatusCallback connect_cb = boost::bind(&ProcessImageNodelet::connectCb,this);
  ros::SubscriberStatusCallback connect_cb_info = boost::bind(&ProcessImageNodelet::connectCb,this);
  ros::SubscriberStatusCallback connect_cb_blobs = boost::bind(&ProcessImageNodelet::connectCb,this);
  // make sure we don't enter connectCb() between advertising and assigning to image_pub_
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  image_pub_ = it_out_->advertiseCamera("image_raw",1,connect_cb,connect_cb,connect_cb_info,connect_cb_info);
  blobs_pub_ = nh.advertise<Blobs>("blobs",1000,connect_cb_blobs,connect_cb_blobs);
  save_background_sub_ = private_nh.subscribe("save_background_image",2,&ProcessImageNodelet::saveBackgroundImageCb,this);
  find_otsu_threshold_sub_ = nh.subscribe("find_otsu_threshold",1,&ProcessImageNodelet::findOtsuThresholdCallback,this);
}

// handles (un)subscribing when clients (un)subscribe
void ProcessImageNodelet::connectCb()
{
  boost::lock_guard<boost::mutex> lock(connect_mutex_);
  if ((image_pub_.getNumSubscribers() == 0) && (blobs_pub_.getNumSubscribers() == 0))
  {
    cam_sub_.shutdown();
  }
  else if (!cam_sub_)
  {
    // read background image if one exists
    image_background_ = cv::imread(background_image_path_,CV_LOAD_IMAGE_GRAYSCALE);

    image_transport::TransportHints hints("raw",ros::TransportHints(),getPrivateNodeHandle());
    cam_sub_ = it_in_->subscribeCamera("image_raw",queue_size_,&ProcessImageNodelet::imageCb,this,hints);
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
  int morph_kernel_size = config.morph_kernel_size;
  int center_marker_radius = config.center_marker_radius;
  int drawn_line_thickness = config.drawn_line_thickness;
  bool draw_crosshairs = config.draw_crosshairs;

  // get a cv::Mat view of the source data
  {
    boost::lock_guard<boost::mutex> lock(callback_mutex_);
    source_ptr_ = cv_bridge::toCvShare(image_msg,sensor_msgs::image_encodings::MONO8);
  }

  // subtract background image if one exists
  if (image_background_.data)
  {
    boost::lock_guard<boost::mutex> lock(callback_mutex_);
    cv::absdiff(source_ptr_->image,image_background_,image_foreground_);
  }
  else
  {
    boost::lock_guard<boost::mutex> lock(callback_mutex_);
    image_foreground_ = source_ptr_->image;
  }

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
  cv::cvtColor(image_morph,image_output,CV_GRAY2BGR);

  // draw crosshairs
  if (draw_crosshairs)
  {
    int h = image_msg->height;
    int w = image_msg->width;
    cv::Scalar yellow(0,255,255);
    cv::line(image_output,cv::Point(0,h/2),cv::Point(w,h/2),yellow,drawn_line_thickness);
    cv::line(image_output,cv::Point(w/2,0),cv::Point(w/2,h),yellow,drawn_line_thickness);
  }

  // draw Contours
  cv::Scalar blue(255,0,0);
  drawContours(image_output,contours,-1,blue,-1,8);

  // calculate blob data
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
  output.header = source_ptr_->header;
  output.encoding = sensor_msgs::image_encodings::BGR8;

  sensor_msgs::ImagePtr out_image = output.toImageMsg();

  // create updated CameraInfo message
  sensor_msgs::CameraInfoPtr out_info = boost::make_shared<sensor_msgs::CameraInfo>(*info_msg);

  image_pub_.publish(out_image,out_info);
}

void ProcessImageNodelet::configCb(Config &config,uint32_t level)
{
  config_ = config;
}

void ProcessImageNodelet::saveBackgroundImageCb(const std_msgs::EmptyConstPtr& msg)
{
  boost::lock_guard<boost::mutex> lock(callback_mutex_);
  cv::imwrite(background_image_path_,source_ptr_->image);
  source_ptr_->image.copyTo(image_background_);
}

void ProcessImageNodelet::findOtsuThresholdCallback(const std_msgs::EmptyConstPtr& message)
{
  boost::lock_guard<boost::mutex> lock(callback_mutex_);
  cv::Mat image_threshold;
  threshold_ = cv::threshold(source_ptr_->image,image_threshold,0,255,cv::THRESH_BINARY+cv::THRESH_OTSU);
  ROS_INFO("Finding Otsu threshold.");
}

} // namespace blob_tracker

// register nodelet
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS( blob_tracker::ProcessImageNodelet,nodelet::Nodelet)
