#include "isr_ros_utils/tools/MsgConverter.h"

namespace ros
{
geometry_msgs::PoseStamped MsgConverter::toPoseStamped(const geometry_msgs::PoseWithCovarianceStamped& in)
{
  geometry_msgs::PoseStamped out;
  out.header = in.header;
  out.pose = in.pose.pose;
  return out;
}

tf2::Quaternion MsgConverter::toQuaternionTF(const geometry_msgs::Quaternion& in)
{
  tf2::Quaternion out;
  tf2::fromMsg(in, out);
  return out;
}

geometry_msgs::Quaternion MsgConverter::toQuaternionMsg(const tf2::Quaternion& in)
{
  return tf2::toMsg(in);
}

void MsgConverter::fromQuaternionToRPY(const tf2::Quaternion& in, double& roll, double& pitch, double& yaw)
{
  tf2::Matrix3x3 matrix(in);
  matrix.getEulerYPR(yaw, pitch, roll);
}

void MsgConverter::fromQuaternionToRPY(const geometry_msgs::Quaternion& in, double& roll, double& pitch, double& yaw)
{
  tf2::Quaternion quat = toQuaternionTF(in);
  tf2::Matrix3x3 matrix(quat);
  matrix.getEulerYPR(yaw, pitch, roll);
}

geometry_msgs::Quaternion MsgConverter::fromRPYToQuaternionMsg(double roll, double pitch, double yaw)
{
  tf2::Quaternion quat;
  quat.setRPY(roll, pitch, yaw);
  geometry_msgs::Quaternion out = toQuaternionMsg(quat);
  return out;
}

nav_msgs::Odometry MsgConverter::toOdometry(const geometry_msgs::TransformStamped& tf)
{
  nav_msgs::Odometry odom;
  odom.header = tf.header;
  odom.child_frame_id = tf.child_frame_id;
  odom.pose.pose = fromTransformToPose(tf.transform);
  return odom;
}

geometry_msgs::PoseStamped MsgConverter::toPoseStamped(const geometry_msgs::TransformStamped& tf)
{
  geometry_msgs::PoseStamped pose;
  pose.header = tf.header;
  pose.pose = fromTransformToPose(tf.transform);
  return pose;
}

geometry_msgs::Pose MsgConverter::fromTransformToPose(const geometry_msgs::Transform& tf)
{
  geometry_msgs::Pose pose;
  pose.position = fromVector3ToPoint(tf.translation);
  pose.orientation = tf.rotation;
  return pose;
}

geometry_msgs::Point MsgConverter::fromVector3ToPoint(const geometry_msgs::Vector3& vector3)
{
  geometry_msgs::Point point;
  point.x = vector3.x;
  point.y = vector3.y;
  point.z = vector3.z;
  return point;
}

geometry_msgs::Vector3 MsgConverter::fromPointToVector3(const geometry_msgs::Point& point)
{
  geometry_msgs::Vector3 vector3;
  vector3.x = point.x;
  vector3.y = point.y;
  vector3.z = point.z;
  return vector3;
}

}  // namespace ros