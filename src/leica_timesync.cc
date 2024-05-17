#include "leica_timesync.h"

LeicaTimesync::LeicaTimesync() : nh_("~")
{
    point_sub_ = nh_.subscribe<geometry_msgs::PointStamped>("/leica/position", 10, &LeicaTimesync::pointCallback, this, ros::TransportHints().tcpNoDelay());
    point_pub_ = nh_.advertise<geometry_msgs::PointStamped>("/leica/position/rostime", 10);

    imu_sub_ = nh_.subscribe<sensor_msgs::Imu>("/imu/data_raw", 10, &LeicaTimesync::imuCallback, this, ros::TransportHints().tcpNoDelay());
    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("/imu/data_raw/rostime", 10);

    offset_pub_ = nh_.advertise<std_msgs::Float64>("/leica/clockoffset", 10);
    raw_offset_pub_ = nh_.advertise<std_msgs::Float64>("/leica/raw_clockoffset", 10);
}

void LeicaTimesync::pointCallback(const geometry_msgs::PointStamped::ConstPtr& msg)
{
    geometry_msgs::PointStamped output_msg = *msg;

    output_msg.header.stamp = ros::Time(msg->header.stamp.toSec() + filtered_diff_);
    output_msg.header.frame_id = "leica";

    point_pub_.publish(output_msg);
}

void LeicaTimesync::imuCallback(const sensor_msgs::Imu::ConstPtr& msg)
{
    sensor_msgs::Imu output_msg = *msg;
    std_msgs::Float64 filtered_diff_msg;
    std_msgs::Float64 raw_offset_msg;

    double clock_offset = ros::Time::now().toSec() - msg->header.stamp.toSec();

    // Initialize filter with first clock offset
    if(init_filter_)
    {
        filter_.init_filter(clock_offset);
        init_filter_ = false;
    }

    filtered_diff_ = filter_.update(clock_offset);

    output_msg.header.stamp = ros::Time(msg->header.stamp.toSec() + filtered_diff_);
    output_msg.header.frame_id = "imu";

    imu_pub_.publish(output_msg);

    filtered_diff_msg.data = filtered_diff_;
    offset_pub_.publish(filtered_diff_msg);
    
    raw_offset_msg.data = clock_offset;
    raw_offset_pub_.publish(raw_offset_msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "leica_timesync_node");
    LeicaTimesync leica_timesync;
    ros::spin();
    return 0;
}
