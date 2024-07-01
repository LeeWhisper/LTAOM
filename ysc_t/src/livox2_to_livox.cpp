#include <livox_ros_driver2/CustomMsg.h>
#include <livox_ros_driver/CustomMsg.h>
#include <ros/ros.h>

void livox_cllback(const livox_ros_driver::CustomMsgConstPtr &msg);
ros::Publisher pub;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "livox_to_livox2");

    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe<livox_ros_driver::CustomMsg>("/livox/lidar", 10, livox_cllback);
    pub = nh.advertise<livox_ros_driver2::CustomMsg>("/livox2/lidar", 10);
    ros::spin();

    return 0;
}

void livox_cllback(const livox_ros_driver::CustomMsgConstPtr &msg)
{
    livox_ros_driver2::CustomMsg livox2;
    livox2.header = msg->header;
    livox2.lidar_id = msg->lidar_id;
    livox2.point_num = msg->point_num;
    livox2.rsvd = msg->rsvd;
    livox2.timebase = msg->timebase;
    // livox2.points.resize(msg->point_num);

    for (int i = 0; i < msg->points.size(); i++)
    {
        livox_ros_driver2::CustomPoint point;

        point.line = msg->points[i].line;
        point.offset_time = msg->points[i].offset_time;
        point.reflectivity = msg->points[i].reflectivity;
        point.tag = msg->points[i].tag;
        point.x = msg->points[i].x;
        point.y = msg->points[i].y;
        point.z = msg->points[i].z;

        livox2.points.push_back(point);
    }

    pub.publish(livox2);
}