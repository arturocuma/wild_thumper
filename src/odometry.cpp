#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <math.h>


double vel_x = 0.0;
double vel_y = 0.0;
double ang_z = 0.0;

double vel_th = 0.0;

ros::Time last_vel_time(0.0);


void velCallback(const geometry_msgs::Twist& msg)
{
  //callback every time the robot's linear velocity is received
  ros::Time current_time = ros::Time::now();

  vel_x = msg.linear.x;
  vel_y = msg.linear.y;
  ang_z = msg.angular.z;

  vel_th = (current_time - last_vel_time).toSec();
  last_vel_time = current_time;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("motor_control", 50, velCallback);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 50);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double th = 0.0;

  double vx = 0.0;
  double vy = 0.0;
  double vth = 0.0;
  double dt = 0.0;

  double rate = 10.0;

  ros::Time current_time;
  current_time = ros::Time::now();

  ros::Rate r(rate);
  while(nh.ok()){

    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    vx = vel_x;
    vx = vel_y;
    vth = ang_z;
    dt = vel_th;

    //compute odometry in a typical way given the velocities of the robot
    double delta_th = vth * dt;
    double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
    double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
    

    //Position of the robot
    x += delta_x;
    y += delta_y;
    th += delta_th;

    //since all odometry is 6DOF we'll need a quaternion created from yaw
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);

    //first, we'll publish the transform over tf
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = current_time;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";

    odom_trans.transform.translation.x = x;
    odom_trans.transform.translation.y = y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;

    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    //next, we'll publish the odometry message over ROS
    nav_msgs::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";

    //set the position
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.y = y;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;

    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vx;
    odom.twist.twist.linear.y = vy;
    odom.twist.twist.angular.z = vth;

    //publish the message
    odom_pub.publish(odom);
    r.sleep();
  }
}
