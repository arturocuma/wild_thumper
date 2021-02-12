#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <math.h>


double vel_x = 0.0;
double vel_y = 0.0;
double vel_th = 0.0;


void velCallback(const geometry_msgs::Twist& msg)
{
  //callback every time the robot's linear velocity is received
  //current_time = ros::Time::now();

  vel_x = msg.linear.x;
  vel_y = msg.linear.y;
  vel_th = msg.angular.z;

  //dt = (current_time - last_vel_time).toSec();
  //last_vel_time = current_time;
}

int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_publisher");

  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("motor_control", 100, velCallback);
  ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
  tf::TransformBroadcaster odom_broadcaster;

  double x = 0.0;
  double y = 0.0;
  double z = 0.0; 
  double dt = 0.0;
  double th = 0.0;

  double rate = 30.0;

  ros::Time current_time, last_time;
  current_time = ros::Time::now();
  last_time = ros::Time::now();

  ros::Rate r(rate);
  while(nh.ok()){
    
    ros::spinOnce();               // check for incoming messages
    current_time = ros::Time::now();

    //compute odometry in a typical way given the velocities of the robot
    dt = (current_time - last_time).toSec();
    double delta_x = (vel_x * cos(th) - vel_y * sin(th)) * dt;
    double delta_y = (vel_x * sin(th) + vel_y * cos(th)) * dt;
    double delta_th = vel_th * dt;
    

    //Position of the robot
    std::cout << dt;
    std::cout << "x y z: ";
    std::cout << x << " " << y << " "<< z;

    x += delta_x;
    y += delta_y;
    z = 0.082;
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
    odom_trans.transform.translation.z = z;
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
    odom.pose.pose.position.z = z;
    odom.pose.pose.orientation = odom_quat;

    //set the pose covariance
    odom.pose.covariance = {0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.2, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.2, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.2};


    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = vel_x;
    odom.twist.twist.linear.y = vel_y;
    odom.twist.twist.angular.z = vel_th;

    //set the twist covariance
    odom.twist.covariance = {0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.2, 0.0, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.2, 0.0,
                            0.0, 0.0, 0.0, 0.0, 0.0, 0.2};

    //publish the message
    odom_pub.publish(odom);
    last_time = current_time;
    r.sleep();
    
  }
}
