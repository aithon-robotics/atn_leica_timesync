#include "leica_timesync.h"

LeicaTimesync::LeicaTimesync() : nh_("~")
{
    point_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/leica/position", 10, &LeicaTimesync::pointCallback, this, ros::TransportHints().tcpNoDelay());
    point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/leica/position/rostime", 10);

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data_raw", 10, &LeicaTimesync::imuCallback, this, ros::TransportHints().tcpNoDelay());
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/leica/imu/rostime", 10);
}

void LeicaTimesync::pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    geometry_msgs::PointStamped output_msg = *msg;
    output_msg.header.stamp = ros::Time::now() - ros::Duration(0.06); // Subtract a delay of 60ms from the current timestamp
    point_pub_.publish(output_msg);
}

void LeicaTimesync::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    sensor_msgs::Imu output_msg = *msg;
    output_msg.header.stamp = ros::Time::now() - ros::Duration(0.003); // Subtract a delay of 3ms from the current timestamp
    imu_pub_.publish(output_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "leica_timesync_node");
    LeicaTimesync leica_timesync;
    ros::spin();
    return 0;
}