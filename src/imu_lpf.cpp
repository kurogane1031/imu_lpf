#include <ros/ros.h>
#include <sensor_msgs/Imu.h>

static double gravity[3] = {0, 0, 0};

class ImuFiltersNode{
public:
  ImuFiltersNode()
    :nh()
  {
    imu_lpf_pub = nh.advertise<sensor_msgs::Imu>("/imu/lpf/data", 5);
    imu_sub = nh.subscribe("/imu/data", 5, &ImuFiltersNode::imuCallback, this);
  }

  ros::NodeHandle nh;
  ros::Subscriber imu_sub;
  ros::Publisher imu_lpf_pub;
  void imuCallback(const sensor_msgs::ImuConstPtr &msg){
    constexpr double alpha = 0.0;
    low_pass_filter(msg, alpha);
  }

  void low_pass_filter(const sensor_msgs::ImuConstPtr &msg, const double &alpha){
    sensor_msgs::Imu out_msg;
    out_msg.header.stamp = ros::Time::now();
    out_msg.header.frame_id = msg->header.frame_id;

    gravity[0] = alpha * gravity[0] + (1 - alpha) * msg->linear_acceleration.x;
    gravity[1] = alpha * gravity[1] + (1 - alpha) * msg->linear_acceleration.y;
    gravity[2] = alpha * gravity[2] + (1 - alpha) * msg->linear_acceleration.z;

    out_msg.linear_acceleration.x = msg->linear_acceleration.x - gravity[0];
    out_msg.linear_acceleration.y = msg->linear_acceleration.y - gravity[1];
    out_msg.linear_acceleration.z = msg->linear_acceleration.z - gravity[2];

    out_msg.angular_velocity = msg->angular_velocity;

    imu_lpf_pub.publish(out_msg);
  }
};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "imu_lpf");
  ImuFiltersNode node;

  ros::spin();
}
