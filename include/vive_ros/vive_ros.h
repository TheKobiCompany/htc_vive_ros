#include <cmath>
#include <iostream>
#include <ros/ros.h>
#include <vector>

#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/JoyFeedback.h>
#include <std_srvs/Empty.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <vive_ros/vr_interface.h>

class VIVEnode {
 public:
  VIVEnode();
  ~VIVEnode();
  bool            Init();
  void            Run();
  void            Shutdown();
  bool            setOriginCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
  void            set_feedback(sensor_msgs::JoyFeedbackConstPtr msg);
  ros::NodeHandle nh_;
  VRInterface     vr_;

#ifdef USE_IMAGE
  void                        imageCb_L(const sensor_msgs::ImageConstPtr& msg);
  void                        imageCb_R(const sensor_msgs::ImageConstPtr& msg);
  void                        infoCb_L(const sensor_msgs::CameraInfoConstPtr& msg);
  void                        infoCb_R(const sensor_msgs::CameraInfoConstPtr& msg);
  CMainApplicationMod*        pMainApplication;
  image_transport::Subscriber sub_L, sub_R;
  ros::Subscriber             sub_i_L, sub_i_R;
#endif

 private:
  double                                rate_;
  std::vector<double>                   world_offset_;
  double                                world_yaw_;
  tf::TransformBroadcaster              tf_broadcaster_;
  tf::TransformListener                 tf_listener_;
  ros::ServiceServer                    set_origin_server_;
  std::map<std::string, ros::Publisher> odometry_pubs_map_;
  ros::Subscriber                       feedback_sub_;
};
