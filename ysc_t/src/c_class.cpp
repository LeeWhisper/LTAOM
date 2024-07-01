#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <queue>

using namespace std;

class tmp
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;

    queue<nav_msgs::Odometry::ConstPtr> gtBuffer;

public:
    void gtHandler(const nav_msgs::Odometry::ConstPtr &msgIn);

    tmp(/* args */);
};

tmp::tmp(/* args */)
{
    sub = nh.subscribe<nav_msgs::Odometry>("/ground_truth", 10, &tmp::gtHandler, this);
}

void tmp::gtHandler(const nav_msgs::Odometry::ConstPtr &msgIn)
{
    cout << msgIn->pose.pose.position.x << endl;
    cout << "gtMsg get" << endl;
    gtBuffer.push(msgIn);
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "c_class");

    tmp tm;

    while (ros::ok())
    {
        ros::spinOnce();
        ros::Rate(10).sleep();
    }

    return 0;
}
