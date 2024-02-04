#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/PointStamped.h>
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
    backtobase_pub_=nh_.advertise<geometry_msgs::PointStamped>("clicked_point", 10);
  }

  private:
  ros::Subscriber laser_sub_;
  ros::Publisher cmd_pub_;
  ros::Publisher backtobase_pub_;
  ros::ServiceClient client;
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;

  geometry_msgs::PoseStamped RobotPose;
  double roll, pitch, yaw;

  double angular_speed = 0.5;
  double linear_speed = 0.5;
  std::default_random_engine gen;
  std::uniform_real_distribution<double> angle_dist;

  nav_msgs::MapMetaData metadata_msg;
  int width = 0;
  int height = 0;
  float previous_discovery = 0;

  void GetRobotPose(){
    geometry_msgs::TransformStamped transform;
    
    try {
      transform = tfBuffer.lookupTransform("map", "base_link", ros::Time(0), ros::Duration(1.0));
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

    if(mapCheck()<7 /* 8.5 */ /* % de la map découverte*/){ /* 10% = Sim map complètement découverte */
      float left_mean, right_mean, front_mean=0.0;
      for(int i=(size/2)-(size/5); i<=(size/2)+(size/5); i++){
        front_mean+=scanmsg->ranges[i];
      }
      front_mean=front_mean/((size/5)*2);
      // ROS_INFO("front_mean: %f", front_mean);

      for(int i=4*(size/5); i<(size); i++){
        left_mean+=scanmsg->ranges[i];
      }
      left_mean=left_mean/(size/5);
      // ROS_INFO("left_mean: %f", left_mean);

      for(int i=0; i<=(size/5); i++){
        right_mean+=scanmsg->ranges[i];
      }
      right_mean=right_mean/(size/5);
      // ROS_INFO("right_mean: %f", right_mean);

      if(front_mean>=1 && right_mean >= 0.5 && left_mean >= 0.5){
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
    else{
      backtobase();
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
      // ROS_INFO("Gauche : %f + %f = %f",temp_yaw,random_angle,desired_angle);
    }
    else{
      desired_angle=(temp_yaw-random_angle<=-M_PI) ? 2*M_PI+(temp_yaw-random_angle) : temp_yaw-random_angle;
      // ROS_INFO("Droite : %f + %f = %f",temp_yaw,random_angle,desired_angle);
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

  float mapCheck(){
    // Création de la requête
    nav_msgs::GetMap srv;

    float mean = 0;

    // Appel du service
    if (client.call(srv))
    {

      // La carte est disponible dans srv.response.map
      metadata_msg = srv.response.map.info;
      width = metadata_msg.width;
      height = metadata_msg.height;
      // ROS_INFO("width: %d, height: %d", width, height);

      // Parcourir les données de la carte
      for (int row = 0; row < height; ++row) {
        for (int col = 1; col <= width; ++col) {
          int index = col + row * width;
          if(srv.response.map.data[index] == 0){mean++;}
        }
      }
      mean=(mean/(height*width))*100;
      if(mean!= previous_discovery){
        ROS_INFO("Map decouverte a %f pourcents", mean);
        previous_discovery=mean;
      }
    }
    else
    {
      ROS_ERROR("Echec de l'appel du service");
    }
    return mean;
  }

  void backtobase(){
    geometry_msgs::PointStamped base;
    base.point.x = 0;
    base.point.y = 0;
    base.header.stamp = ros::Time::now();
    base.header.frame_id = "map";

    geometry_msgs::Twist twistmsg;
    twistmsg.linear.x = 0;
    twistmsg.angular.z = 0;
    cmd_pub_.publish(twistmsg);

    ROS_INFO("Going back to base");
    backtobase_pub_.publish(base);

    ros::shutdown();
    ROS_INFO("Should not appear");
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "autom");
  AutomaticDiscover autom;
  ros::spin();
  return 0;
}

