#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>
#include <cmath>

class AutomaticDiscover {
  public:
  AutomaticDiscover(): gen(std::random_device{}()), angle_dist(M_PI/2/* 1.57 */,(2*M_PI/3)/* 2.09 */), tfListener(tfBuffer){
    ros::NodeHandle nh_;

    // Attente du service
    ROS_INFO("Waiting for service");
    ros::service::waitForService("/dynamic_map");
    ROS_INFO("Service available");

    client = nh_.serviceClient<nav_msgs::GetMap>("/dynamic_map");

    laser_sub_=nh_.subscribe<sensor_msgs::LaserScan>("scan", 10, &AutomaticDiscover::laser_cb, this);
    cmd_pub_=nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  }

  private:
  ros::Subscriber laser_sub_;
  ros::Publisher cmd_pub_;
  ros::ServiceClient client;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  geometry_msgs::PoseStamped RobotPose;
  double roll, pitch, yaw;

  double angular_speed = 0.5;
  double linear_speed = 0.5;
  std::default_random_engine gen;
  std::uniform_real_distribution<double> angle_dist;

  void GetRobotPose(){
    geometry_msgs::TransformStamped transform;
    
    try {
      transform = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
    }

    catch (tf2::TransformException &ex){
      ROS_ERROR("%s",ex.what());
    }

    RobotPose.header = transform.header;
    RobotPose.pose.position.x = transform.transform.translation.x;
    RobotPose.pose.position.y = transform.transform.translation.y;
    RobotPose.pose.orientation.x = transform.transform.rotation.x;
    RobotPose.pose.orientation.y = transform.transform.rotation.y;
    RobotPose.pose.orientation.z = transform.transform.rotation.z;
    RobotPose.pose.orientation.w = transform.transform.rotation.w;

    tf2::Quaternion q(
      RobotPose.pose.orientation.x,
      RobotPose.pose.orientation.y,
      RobotPose.pose.orientation.z,
      RobotPose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);

    return;
  }

  void laser_cb(const sensor_msgs::LaserScan::ConstPtr& scanmsg){
    size_t size = scanmsg->ranges.size();
    // ROS_INFO("size: %zu", size);

    float left_mean, right_mean, front_mean=0.0;
    for(int i=(size/2)-(size/5); i<=(size/2)+(size/5); i++){
      front_mean+=scanmsg->ranges[i];
    }
    front_mean=front_mean/((size/5)*2);
    // ROS_INFO("front_mean: %f", front_mean);
    for(int i=2*(size/3); i<=(size); i++){
      left_mean+=scanmsg->ranges[i];
    }
    left_mean=left_mean/(size/3);
    // ROS_INFO("left_mean: %f", left_mean);
    for(int i=0; i<=(size/3); i++){
      right_mean+=scanmsg->ranges[i];
    }
    right_mean=right_mean/(size/3);
    // ROS_INFO("right_mean: %f", right_mean);

    if(front_mean>=1 /* && right_mean >= 0.2 && left_mean >= 0.2 */){
      if(front_mean<999){
        publish_forward();
      }
      else{ //On va dans les limbes
        getbacktosafety();
      }
    }
    else{
      (left_mean>=right_mean) ? publish_rot(true) : publish_rot(false);
    }
  }

  void publish_forward(){
    geometry_msgs::Twist twistmsg;
    twistmsg.linear.x = linear_speed;
    cmd_pub_.publish(twistmsg);
  }

  void publish_rot(bool sens){ //true = gauche, false = droite 
    geometry_msgs::Twist twistmsg;
    twistmsg.linear.x = 0.0;

    double random_angle = angle_dist(gen);
    ROS_INFO("random_angle: %f", random_angle);
    
    GetRobotPose();
    double temp_yaw = yaw;
    double desired_angle;

    if(sens){
      desired_angle=(temp_yaw+random_angle>=M_PI) ? -2*M_PI+(temp_yaw+random_angle) : temp_yaw+random_angle;
      ROS_INFO("Gauche : %f + %f = %f",temp_yaw,random_angle,desired_angle);
    }
    else{
      desired_angle=(temp_yaw-random_angle<=-M_PI) ? 2*M_PI+(temp_yaw-random_angle) : temp_yaw-random_angle;
      ROS_INFO("Droite : %f + %f = %f",temp_yaw,random_angle,desired_angle);
    }

    while(!((yaw>(desired_angle-0.2))&&(yaw<(desired_angle+0.2)))){
      twistmsg.angular.z = (sens) ? angular_speed : -angular_speed;
      cmd_pub_.publish(twistmsg);

      GetRobotPose();
    }
    
    twistmsg.angular.z = 0.0;
    return;
  }

  void getbacktosafety(){
    geometry_msgs::Twist twistmsg;
    twistmsg.linear.x = 0.0;

    GetRobotPose();
    double temp_yaw = yaw;
    double desired_angle=(temp_yaw+M_PI>=M_PI) ? -M_PI+temp_yaw : temp_yaw+M_PI;
    while(!((yaw>(desired_angle-0.05))&&(yaw<(desired_angle+0.05)))){
      twistmsg.angular.z = angular_speed;
      cmd_pub_.publish(twistmsg);
      GetRobotPose();
    }

    twistmsg.angular.z = 0.0;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "autom");
  AutomaticDiscover autom;
  ros::spin();
  return 0;
}

