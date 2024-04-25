#ifndef LEICA_TIMESYNC_H
#define LEICA_TIMESYNC_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/Imu.h>

class LeicaTimesync
{
public:
    LeicaTimesync();

private:
    ros::NodeHandle nh_;

    ros::Subscriber point_sub_;
    ros::Publisher point_pub_;

    ros::Subscriber imu_sub_;
    ros::Publisher imu_pub_;

    void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);
};

#endif // LEICA_TIMESYNC_H
