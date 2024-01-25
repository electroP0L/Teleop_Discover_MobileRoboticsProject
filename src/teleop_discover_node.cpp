#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

class TeleopDiscover {
public:
  TeleopDiscover(){
    ros::NodeHandle nh_;
    // Note: 2 services : static_map & dynamic_map
    joy_sub_=nh_.subscribe<sensor_msgs::Joy>("joy", 10, &TeleopDiscover::joy_cb, this);
    cmd_pub_=nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  }

  void joy_cb(const sensor_msgs::Joy::ConstPtr& joymsg){
    geometry_msgs::Twist twistmsg;
    twistmsg.linear.x = joymsg->axes[1]/2;
    twistmsg.angular.z = joymsg->axes[0];
    cmd_pub_.publish(twistmsg);
  }

private:
  ros::Subscriber joy_sub_;
  ros::Publisher cmd_pub_;
};


int main(int argc, char **argv)
  {
    ros::init(argc, argv, "teleop_joy");
    TeleopDiscover teleop;
    ros::spin();
    return 0;
  }

