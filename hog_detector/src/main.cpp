#include <ros/ros.h>
#include <iostream>
#include <iomanip>

#include <image_transport/image_transport.h>
#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <cv_bridge/cv_bridge.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h> 
#include <pcl/visualization/cloud_viewer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/subscriber.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Image.h>

using namespace cv;
using namespace std;
using namespace message_filters;
using namespace pcl;

Mat rgb_frame, dpt_frame;
ros::Publisher point_sub;

class Detector {
  enum Mode {Default, Daimler} m;
  HOGDescriptor hog, hog_d;
public:
  Detector() : m(Default), hog(), hog_d(Size(48, 96), Size(16, 16), Size(8, 8), Size(8, 8), 9) {
    hog.setSVMDetector(HOGDescriptor::getDefaultPeopleDetector());
    hog_d.setSVMDetector(HOGDescriptor::getDaimlerPeopleDetector());
  }
  void toggleMode() { m = (m == Default ? Daimler : Default); }
  string modeName() const { return (m == Default ? "Default" : "Daimler"); }
  vector<Rect> detect(InputArray img) {
    vector<Rect> found;
    if (m == Default)
      hog.detectMultiScale(img, found, 0, Size(8,8), Size(), 1.05, 2, false);
    else if (m == Daimler)
      hog_d.detectMultiScale(img, found, 0, Size(8,8), Size(), 1.05, 2, true);
    return found;
  }
  void adjustRect(Rect & r) const {
    r.x += cvRound(r.width*0.1);
    r.width = cvRound(r.width*0.8);
    r.y += cvRound(r.height*0.07);
    r.height = cvRound(r.height*0.8);
  }
};

void RGBDcallback(const sensor_msgs::ImageConstPtr& msg_rgb , const sensor_msgs::ImageConstPtr& msg_depth) {  
  cv_bridge::CvImagePtr rgb_ptr;
  cv_bridge::CvImagePtr dpt_ptr;
  try { 
    rgb_ptr = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert '%s' format", e.what());
  }
 
  try{
    dpt_ptr = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_8UC1);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("Could not convert '%s' format", e.what());
  }  

  rgb_frame = rgb_ptr->image;
  dpt_frame = dpt_ptr->image;
  Mat rgb_detected_frame;
  Mat dpt_detected_frame;
  Detector detector; 
  vector<Rect> found = detector.detect(rgb_frame);

  for (vector<Rect>::iterator i = found.begin(); i != found.end(); ++i) {
    Rect &r = *i;
    detector.adjustRect(r);
    Rect crop_region((int)r.tl().x, (int)r.tl().y, (int)(r.br().x - r.tl().x), (int)(r.br().y - r.tl().y));
    rgb_detected_frame = rgb_frame(crop_region);
    dpt_detected_frame = dpt_frame(crop_region);
    rectangle(rgb_frame, r.tl(), r.br(), Scalar(0, 255, 0), 2);
  }
  imshow("People detector", rgb_frame); 
  if (rgb_detected_frame.rows > 0 && rgb_detected_frame.cols > 0) {
    imshow("RGB detection", rgb_detected_frame);
    imshow("Depth detection", dpt_detected_frame);
  } 
  waitKey(30);
}

void PCLcallback(const sensor_msgs::CameraInfo& info_msg) {
  typedef PointXYZRGB PointT;
  typedef PointCloud<PointT> PointCloud;
  PointCloud::Ptr my_cloud(new PointCloud);
  for (int v=0; v<rgb_frame.rows; v++) {
    for (int u=0; u<rgb_frame.cols; u++) {
      unsigned int d = dpt_frame.ptr<unsigned short>(v)[u];
      if (d==0)
        continue;
      PointT p;
      p.z = double(d);
      p.x = (u-info_msg.K[2])*p.z/info_msg.K[0];
      p.y = (v-info_msg.K[5])*p.z/info_msg.K[4];
      p.b = rgb_frame.data[v*rgb_frame.step+u*rgb_frame.channels()];
      p.g = rgb_frame.data[v*rgb_frame.step+u*rgb_frame.channels()+1];
      p.r = rgb_frame.data[v*rgb_frame.step+u*rgb_frame.channels()+2];
      my_cloud->points.push_back(p);
    }
  }
  my_cloud->is_dense = false;
  point_sub.publish(my_cloud);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Human_dection");
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/camera/depth/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
  typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub, depth_sub);
  sync.registerCallback(boost::bind(&RGBDcallback, _1, _2));

  ros::Subscriber cam_info_sub = nh.subscribe("/camera/rgb/camera_info", 1, PCLcallback);
  ros::Publisher point_sub = nh.advertise<PointCloud> ("points2", 1);
  //pcl_pub.publish (*pointCloud);
  ros::spin();
  return 0;
}
