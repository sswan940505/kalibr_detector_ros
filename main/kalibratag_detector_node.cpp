#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>

#include "../camera_models/include/EquidistantCamera.h"
#include "../camera_models/include/PinholeCamera.h"
#include "config.h"
#include "utilities.h"
#include "calcCamPose.h"


template <typename T>
T readParam(ros::NodeHandle &n, std::string name)
{
  std::cout << name <<std::endl;
  T ans;
  if (n.getParam(name, ans))
  {
    ROS_INFO_STREAM("Loaded " << name << ": " << ans);
  }
  else
  {
    ROS_ERROR_STREAM("Failed to load " << name);
    n.shutdown();
  }
  return ans;
}

class kalibraTagDetector{
public:
    kalibraTagDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh);
    ~kalibraTagDetector();
//    bool initParam(std::string filename);
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    bool detect(const sensor_msgs::ImageConstPtr& msg, CamPose& T);

private:
    CamPoseEst camposecal_;
    CameraPtr cameraptr_;
    cv::Mat undist_map1_,undist_map2_;
    ros::NodeHandle it_;
    ros::Subscriber image_sub_;
    ros::Publisher image_pub_;
    ros::Publisher detections_pub_;
    ros::Publisher pose_pub_;
};

kalibraTagDetector::kalibraTagDetector(ros::NodeHandle& nh, ros::NodeHandle& pnh): it_(nh){

    std::string config_file;
    config_file = readParam<std::string>(pnh, "config_file");

    cv::FileStorage fsSettings(config_file,     cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
    std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    int tagtype;
    double tagsize = 0.055;  // default size
    double tagspace = 0.3;  // default
    int black_border;
    tagtype = static_cast<int>(fsSettings["tag_type"]);
    tagsize = static_cast<double>(fsSettings["tag_size"]);
    tagspace = static_cast<double>(fsSettings["tag_spacing"]);
    black_border = static_cast<double>(fsSettings["black_border"]);
    std::cout <<"tag_size: "<< tagsize <<" type: "<<tagtype<<std::endl;

    if( 1 == tagtype)
    {
        CalibrBoardInfo info(KALIBR_TAG_PATTERN,tagsize,tagspace,black_border);
        camposecal_ = CamPoseEst(info);
    }else if(2 == tagtype)
    {
        CalibrBoardInfo info(APRIL_TAG_ONE,tagsize,black_border);
        camposecal_ = CamPoseEst(info);
    }else if (3 == tagtype)
    {
        CalibrBoardInfo info(CHESS,tagsize);
        camposecal_ = CamPoseEst(info);
    }

    std::string sModelType;
    fsSettings["model_type"] >> sModelType;

    cameraptr_ = nullptr;
    if (sModelType.compare("KANNALA_BRANDT") == 0)
    {
        EquidistantCamera::Parameters paras;
        paras.readFromYamlFile(config_file);
        cameraptr_ = CameraPtr (new EquidistantCamera(paras));
        cameraptr_->initUndistortRectifyMap(undist_map1_,undist_map2_,paras.mu(),paras.mv(),cv::Size(paras.imageWidth(), paras.imageHeight()),paras.u0(),paras.v0());
        ROS_INFO("LOAD KANNALA BRANDT CAMERA!");
    }
    else if(sModelType.compare("PINHOLE") == 0)
    {
        PinholeCamera::Parameters paras;
        paras.readFromYamlFile(config_file);
        cameraptr_ = CameraPtr (new PinholeCamera(paras));
        cameraptr_->initUndistortRectifyMap(undist_map1_,undist_map2_,paras.fx(),paras.fy(),cv::Size(paras.imageWidth(), paras.imageHeight()),paras.cx(),paras.cy());
        ROS_INFO("LOAD PINHOLE CAMERA!"); 
   }

    std::string img_topic_name = "image";
    fsSettings["img_topic_name"] >> img_topic_name;

    fsSettings.release();

    image_pub_ = it_.advertise<sensor_msgs::Image>("/kalibr_detector/image",100);
    pose_pub_ = it_.advertise<geometry_msgs::PoseStamped>("/kalibr_detector/pose",100);


    image_sub_ = it_.subscribe(img_topic_name,10, &kalibraTagDetector::imageCb, this);
}

kalibraTagDetector::~kalibraTagDetector(){
  image_sub_.shutdown();
}

void kalibraTagDetector::imageCb(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception &e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  Eigen::Matrix4d Twc;
  if (cameraptr_ != nullptr)
  {
    cv::Mat rectified;
    cv::remap(cv_ptr->image, rectified, undist_map1_, undist_map2_, CV_INTER_LINEAR);
    cv::Mat resultImage;
    if(camposecal_.calcCamPoseRet(cv_ptr->header.stamp.toSec(), rectified, cameraptr_, Twc,resultImage))
    {
       cv_ptr->image = resultImage.clone();
       image_pub_.publish(cv_ptr->toImageMsg());
       geometry_msgs::PoseStamped poseOut;
       poseOut.header = cv_ptr->header;
       poseOut.pose.position.x = Twc(0,3);
       poseOut.pose.position.y = Twc(1,3);
       poseOut.pose.position.z = Twc(2,3);
       Eigen::Matrix3d R = Twc.block(0,0,3,3);
       Eigen::Quaterniond q(R);
       poseOut.pose.orientation.x = q.x();
       poseOut.pose.orientation.w = q.w();
       poseOut.pose.orientation.y = q.y();
       poseOut.pose.orientation.z = q.z();
       pose_pub_.publish(poseOut);
    }
  }
  else{
    ROS_ERROR("please select camera model and write in yaml file");
  }

}


int main(int argc, char **argv){
  ros::init(argc, argv, "CamPoseEstimation");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string config_file;
  config_file = readParam<std::string>(pnh, "config_file");
  readParameters(config_file);


  // 初始视觉 pose
  kalibraTagDetector detector(nh, pnh);
  ros::spin();
}
