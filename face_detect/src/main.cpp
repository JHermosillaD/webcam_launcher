#include <ros/ros.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>

#include <opencv2/objdetect.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>


#define default_ID "camera"
#define default_topic "/image_raw"

namespace enc = sensor_msgs::image_encodings;
using namespace cv;
using namespace std;

CascadeClassifier face_cascade_("/usr/share/opencv4/haarcascades/haarcascade_frontalface_alt2.xml");
ros::Subscriber img_sub;

void faceDetectCallback(const sensor_msgs::ImageConstPtr& msg) {
  vector<Rect> faces;
  Mat img_rgb;
  Mat img_gray;

  cv_bridge::CvImageConstPtr cv_ptr;
  try { 
    cv_ptr = cv_bridge::toCvShare(msg,enc::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cvtColor(cv_ptr->image, img_gray, CV_BGR2GRAY);
  equalizeHist(img_gray, img_gray);
  img_rgb = cv_ptr->image;
  
  face_cascade_.detectMultiScale(img_gray, faces);
  if (faces.size() > 0) {
    rectangle(img_rgb, faces[0], Scalar(0,0,255));
    imshow("Face detection", img_rgb);
  }
  waitKey(30);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "face_detect");  
  ros::NodeHandle _nh;
  img_sub = _nh.subscribe(default_topic, 1, faceDetectCallback);
   
  while (ros::ok()) { 
      ros::spinOnce();
  }

  return 0;
}
