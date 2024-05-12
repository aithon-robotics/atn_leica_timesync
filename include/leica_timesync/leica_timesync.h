#ifndef LEICA_TIMESYNC_H
#define LEICA_TIMESYNC_H

#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/Imu.h>

// LowPassFilter class definition
#include "LowPassFilter.h"

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

    ros::Publisher offset_pub_;
    ros::Publisher raw_offset_pub_;

    void pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& msg);

    LowPassFilter filter_ = LowPassFilter(0.3, 0.005);
    double filtered_diff_ = 0.0;
    bool init_filter_ = true;
};

#endif // LEICA_TIMESYNC_H
