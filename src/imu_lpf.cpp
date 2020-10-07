#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

ros::Subscriber imu_sub;
ros::Publisher imu_lpf_pub;

static double gravity[3] = {0, 0, 0};

void imuLPF(const sensor_msgs::ImuConstPtr &msg)
{
  constexpr double alpha = 0.8;
  sensor_msgs::Imu out_msg;
  out_msg.header.stamp = ros::Time::now();
  out_msg.header.frame_id = msg->header.frame_id;

  gravity[0] = alpha * gravity[0] + ( 1 - alpha ) * msg->linear_acceleration.x;
  gravity[1] = alpha * gravity[1] + ( 1 - alpha ) * msg->linear_acceleration.y;
  gravity[2] = alpha * gravity[2] + ( 1 - alpha ) * msg->linear_acceleration.z;

  out_msg.linear_acceleration.x = msg->linear_acceleration.x - gravity[0];
  out_msg.linear_acceleration.y = msg->linear_acceleration.y - gravity[1];
  out_msg.linear_acceleration.z = msg->linear_acceleration.z - gravity[2];

  out_msg.angular_velocity = msg->angular_velocity;

  imu_lpf_pub.publish(out_msg);
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "imu_lpf");
  ros::NodeHandle nh;

  imu_lpf_pub = nh.advertise<sensor_msgs::Imu>("/imu/lpf/data_raw", 5);
  imu_sub = nh.subscribe("/imu/data", 5, imuLPF);

  ros::spin();
}
