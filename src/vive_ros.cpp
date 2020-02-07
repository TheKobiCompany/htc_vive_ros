#include <vive_ros/vive_ros.h>

void handleDebugMessages(const std::string& msg) { ROS_DEBUG("[VIVE] %s", msg.c_str()); }
void handleInfoMessages(const std::string& msg) { ROS_INFO("[VIVE] %s", msg.c_str()); }
void handleErrorMessages(const std::string& msg) { ROS_ERROR("[VIVE] %s", msg.c_str()); }

std::string GetTrackedDeviceString(vr::IVRSystem* pHmd, vr::TrackedDeviceIndex_t unDevice,
                                   vr::TrackedDeviceProperty prop,
                                   vr::TrackedPropertyError* peError = NULL) {
  uint32_t unRequiredBufferLen =
      pHmd->GetStringTrackedDeviceProperty(unDevice, prop, NULL, 0, peError);
  if (unRequiredBufferLen == 0) return "";

  char* pchBuffer = new char[unRequiredBufferLen];
  unRequiredBufferLen =
      pHmd->GetStringTrackedDeviceProperty(unDevice, prop, pchBuffer, unRequiredBufferLen, peError);
  std::string sResult = pchBuffer;
  delete[] pchBuffer;
  return sResult;
}

VIVEnode::VIVEnode()
    : nh_(),
      tf_broadcaster_(),
      tf_listener_(),
      vr_(),
      world_offset_({0, 0, 0}),
      world_yaw_(0),
      rate_(90.0) {
  nh_.param<std::vector<double>>("vive/world_offset", world_offset_, world_offset_);
  nh_.param<double>("vive/world_yaw", world_yaw_, world_yaw_);

  nh_.param<double>("vive/rate", rate_, rate_);

  if (world_offset_.size() < 3) {
    world_offset_ = {0., 0., 0.};
  }

  ROS_INFO("[VIVE] World offset: [%2.3f , %2.3f, %2.3f] %2.3f", world_offset_[0], world_offset_[1],
           world_offset_[2], world_yaw_);

  set_origin_server_ = nh_.advertiseService("vive/set_origin", &VIVEnode::setOriginCB, this);
  feedback_sub_      = nh_.subscribe("vive/set_feedback", 10, &VIVEnode::set_feedback, this);
  return;
}

VIVEnode::~VIVEnode() {}

bool VIVEnode::Init() {
  //  Set logging functions

  vr_.setDebugMsgCallback(handleDebugMessages);
  vr_.setInfoMsgCallback(handleInfoMessages);
  vr_.setErrorMsgCallback(handleErrorMessages);

  if (!vr_.Init()) {
    return false;
  }

  return true;
}

void VIVEnode::Shutdown() { vr_.Shutdown(); }

bool VIVEnode::setOriginCB(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  double tf_matrix[3][4];
  int    index = 1, dev_type;
  while (dev_type != 2) {
    dev_type = vr_.GetDeviceMatrix(index++, tf_matrix);
  }
  if (dev_type == 0) {
    ROS_WARN("[VIVE] Coulnd't find controller 1.");
    return false;
  }

  tf::Matrix3x3 rot_matrix(tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2], tf_matrix[1][0],
                           tf_matrix[1][1], tf_matrix[1][2], tf_matrix[2][0], tf_matrix[2][1],
                           tf_matrix[2][2]);
  tf::Vector3   c_z;
  c_z    = rot_matrix * tf::Vector3(0, 0, 1);
  c_z[1] = 0;
  c_z.normalize();
  double new_yaw = acos(tf::Vector3(0, 0, 1).dot(c_z)) + M_PI_2;
  if (c_z[0] < 0) new_yaw = -new_yaw;
  world_yaw_ = -new_yaw;

  tf::Vector3   new_offset;
  tf::Matrix3x3 new_rot;
  new_rot.setRPY(0, 0, world_yaw_);
  new_offset = new_rot * tf::Vector3(-tf_matrix[0][3], tf_matrix[2][3], -tf_matrix[1][3]);

  world_offset_[0] = new_offset[0];
  world_offset_[1] = new_offset[1];
  world_offset_[2] = new_offset[2];

  nh_.setParam("vive/world_offset", world_offset_);
  nh_.setParam("vive/world_yaw", world_yaw_);
  ROS_INFO("[VIVE] New world offset: [%2.3f , %2.3f, %2.3f] %2.3f", world_offset_[0],
           world_offset_[1], world_offset_[2], world_yaw_);

  return true;
}

void VIVEnode::set_feedback(sensor_msgs::JoyFeedbackConstPtr msg) {
  if (msg->type == 1 /* TYPE_RUMBLE */) {
    vr_.TriggerHapticPulse(msg->id, 0, (int)(msg->intensity));
    for (int i = 0; i < 16; i++) vr_.TriggerHapticPulse(i, 0, (int)(msg->intensity));
  }
}

void VIVEnode::Run() {
  ros::Rate loop_rate(rate_);
  double    tf_matrix[3][4];
  int       run_hz_count = 0;

  while (ros::ok()) {
    // do stuff
    const ros::Time now = ros::Time::now();
    vr_.Update();

    tf::Transform  transform;
    tf::Quaternion quat;

    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp    = now;
    odom_msg.header.frame_id = "world_vive";

    for (int i = 0; i < vr::k_unMaxTrackedDeviceCount; i++) {
      int dev_type = vr_.GetDeviceMatrix(i, tf_matrix);

      // No device
      if (dev_type == 0) continue;

      transform.setOrigin(tf::Vector3(tf_matrix[0][3], tf_matrix[1][3], tf_matrix[2][3]));

      tf::Matrix3x3 rot_matrix(tf_matrix[0][0], tf_matrix[0][1], tf_matrix[0][2], tf_matrix[1][0],
                               tf_matrix[1][1], tf_matrix[1][2], tf_matrix[2][0], tf_matrix[2][1],
                               tf_matrix[2][2]);

      rot_matrix.getRotation(quat);
      transform.setRotation(quat);

      std::string cur_sn = GetTrackedDeviceString(vr_.pHMD_, i, vr::Prop_SerialNumber_String);
      std::replace(cur_sn.begin(), cur_sn.end(), '-', '_');
      std::string type;
      // It's a HMD
      if (dev_type == 1) {
        type = "headset";
      } else if (dev_type == 2) {
        type = "controller";
      } else if (dev_type == 3) {
        type = "tracker";
      } else if (dev_type == 4) {
        type = "lighthouse";
      } else {
        type = "unknown";
      }

      tf_broadcaster_.sendTransform(
          tf::StampedTransform(transform, now, "world_vive", type + "_" + cur_sn));

      // Send odometry messages
      tf::poseTFToMsg(transform, odom_msg.pose.pose);
      odom_msg.twist.twist.linear.x  = vr_.device_poses_[i].vVelocity.v[0];
      odom_msg.twist.twist.linear.y  = vr_.device_poses_[i].vVelocity.v[1];
      odom_msg.twist.twist.linear.z  = vr_.device_poses_[i].vVelocity.v[2];
      odom_msg.twist.twist.angular.x = vr_.device_poses_[i].vAngularVelocity.v[0];
      odom_msg.twist.twist.angular.y = vr_.device_poses_[i].vAngularVelocity.v[1];
      odom_msg.twist.twist.angular.z = vr_.device_poses_[i].vAngularVelocity.v[2];

      if (odometry_pubs_map_.count(cur_sn) == 0) {
        odometry_pubs_map_[cur_sn] =
            nh_.advertise<nav_msgs::Odometry>("vive/" + type + "/" + cur_sn + "/odom", 10);
      }
      odometry_pubs_map_[cur_sn].publish(odom_msg);
    }

    // Publish corrective transform
    tf::Transform tf_world;
    tf_world.setOrigin(tf::Vector3(world_offset_[0], world_offset_[1], world_offset_[2]));
    tf::Quaternion quat_world;
    quat_world.setRPY(M_PI / 2, 0, world_yaw_);
    tf_world.setRotation(quat_world);

    tf_broadcaster_.sendTransform(tf::StampedTransform(tf_world, now, "world", "world_vive"));

    ROS_INFO_THROTTLE(1.0, "Run() @ %d [fps]", [](int& cin) {
      int ans = cin;
      cin     = 0;
      return ans;
    }(run_hz_count));

    run_hz_count++;
    ros::spinOnce();
    loop_rate.sleep();
  }
}
