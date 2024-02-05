#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PointStamped.h"
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

//TODO: faire la sélection des waypoints ici plutôt que dans Map Processing

class Venture {
  public:
  Venture() : tfListener(tfBuffer){
    ros::NodeHandle nh_;
    
    path_sub_=nh_.subscribe<nav_msgs::Path>("path_map", 10, &Venture::path_cb, this);
    cmd_pub_=nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  }
  
  private:
  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener;
  ros::Subscriber path_sub_;
  ros::Publisher cmd_pub_;

  geometry_msgs::PoseStamped RobotPose;
  nav_msgs::Path base_path;

  double L = 0.4; //m
  double w_limit = 0.4 /* 0.25 */; //rad/s
  ros::Duration dt; //s
  ros::Time previous_time;
  bool first_waypoint = true;

  const double Kp = 8.0;
  const double Ki = 0.5;
  const double Kd = 0.03;


  void path_cb(const nav_msgs::Path::ConstPtr& pathmsg){
    if (pathmsg->poses.size()!=0){
      base_path = *pathmsg;
      first_waypoint = true;
      SpeedProcessing();
    }
    else {ROS_INFO("Empty path!");}
    return;
  }

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


    if (first_waypoint) {
      dt = ros::Duration (0.5);
      first_waypoint = false;
    }
    else {
      dt = transform.header.stamp - previous_time;
    }

    previous_time = transform.header.stamp;
    return;
  }

  void SpeedProcessing(){
    geometry_msgs::Twist twistmsg;
    double omega = 0; //Vitesse angulaire du robot
    double vx = 0.1; //Vitesse linéaire du robot (constante)
    double theta;

    int waypoints_index = 0;
    bool end_of_path = false;

    while (!end_of_path) {
      GetRobotPose();
      ros::Duration(0.5).sleep();
      //condition qui regarde si on est assez proches du point d'arrivée, si on est a moins de 0,1 m de notre arrivé alors on sort de notre while en passe end_of_path à true
      if(std::sqrt(std::pow(RobotPose.pose.position.x - base_path.poses[base_path.poses.size()-1].pose.position.x, 2) + std::pow(RobotPose.pose.position.y - base_path.poses[base_path.poses.size()-1].pose.position.y, 2)) > 0.1) {
        std::tie(theta, omega) = FeedbackTrackingController(RobotPose, base_path, &waypoints_index);
        twistmsg.linear.x = vx;
        twistmsg.angular.z = omega;
        // ROS_INFO("vitesse : %f ,  %f ,  %f",twistmsg.linear.x, twistmsg.linear.y, twistmsg.angular.z );
      }

      else {
        end_of_path=true;
        base_path.poses.clear();
        twistmsg.linear.x = 0;
        twistmsg.angular.z = 0;
      }

      cmd_pub_.publish(twistmsg);
    }
  }

  std::tuple<double, double> FeedbackTrackingController(geometry_msgs::PoseStamped position, nav_msgs::Path waypoints, int* waypoints_index) {
    // Get robot pose and orientation
    double robot_pos_x = position.pose.position.x;
    double robot_pos_y = position.pose.position.y;

    double roll, pitch, yaw;
    tf2::Quaternion q(
      position.pose.orientation.x,
      position.pose.orientation.y,
      position.pose.orientation.z,
      position.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, yaw);
    double theta = std::abs(yaw);

    // Get nth waypoint pose
    double x_waypoint_n = waypoints.poses[*waypoints_index].pose.position.x;
    double y_waypoint_n = waypoints.poses[*waypoints_index].pose.position.y;
    // Get (n+1)th waypoint pose
    double x_waypoint_nplus1 = waypoints.poses[(*waypoints_index)+1].pose.position.x;
    double y_waypoint_nplus1 = waypoints.poses[(*waypoints_index)+1].pose.position.y;


    // Project current robot pose on the path
    double delta_wp_x = x_waypoint_nplus1 - x_waypoint_n;
    double delta_wp_y = y_waypoint_nplus1 - y_waypoint_n;

    double up = ((robot_pos_x - x_waypoint_n) * (delta_wp_x) + (robot_pos_y - y_waypoint_n) * (delta_wp_y)) / (std::pow(delta_wp_x,2) + std::pow(delta_wp_y,2)); 

    double xo = x_waypoint_n + up * (delta_wp_x); //coordonnées de la position projetée la plus proche sur le segment de chemin
    double yo = y_waypoint_n + up * (delta_wp_y); //coordonnées de la position projetée la plus proche sur le segment de chemin

    // ROS_INFO("condition: %f", std::sqrt((x_waypoint_nplus1 - xo) * (x_waypoint_nplus1 - xo) + (y_waypoint_nplus1 - yo) * (y_waypoint_nplus1 - yo)) );

    if (std::sqrt((x_waypoint_nplus1 - xo) * (x_waypoint_nplus1 - xo) + (y_waypoint_nplus1 - yo) * (y_waypoint_nplus1 - yo)) <= L) {
      if (*waypoints_index < base_path.poses.size() - 2) {
        ROS_INFO("Changement index waypoint: %d -> %d", *waypoints_index, ((*waypoints_index)+1));
        (*waypoints_index)+=1;
      }
    }

    // Vecteur représentatif de la portion de chemin que suit le robot
    std::vector<double> wpvec = {delta_wp_x, y_waypoint_nplus1 - y_waypoint_n};

    // Vecteur unitaire du changement de position que doit opérer le robot
    std::vector<double> norm_vec = {wpvec[0] / std::sqrt(wpvec[0] * wpvec[0] + wpvec[1] * wpvec[1]),
                                    wpvec[1] / std::sqrt(wpvec[0] * wpvec[0] + wpvec[1] * wpvec[1])};

    std::vector<double> result = {xo + L * norm_vec[0], yo + L * norm_vec[1]};

    double cap = std::atan2(result[1] - robot_pos_y, result[0] - robot_pos_x);
    theta = std::copysign(theta, cap);

    double delta_theta = cap - theta; //TODO LE PROBLEME EST PROB ICI
    // ROS_INFO("angle voulu : %f, angle reel : %f", cap, theta);

    // Angular velocity
    double omega = delta_theta / dt.toSec(); //rad/s

    if (omega >= w_limit) {
        omega = w_limit;
    } else if (omega <= -w_limit) {
        omega = -w_limit;
    }

    return std::make_tuple(theta, omega);
  }
};


int main(int argc, char **argv)
  {
    ros::init(argc, argv, "venture");
    Venture venture;
    ros::spin();
    return 0;
  }
