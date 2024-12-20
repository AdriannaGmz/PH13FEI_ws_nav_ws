// Listener of joystick for autonomous navigation. Acts as exta Emergency stop
// Listens to buttons X, Y, A or B
// when pressed, uses twist_mux to:
//    - send velocities 0 to topic xestop_vel
//    - send bool to topic e_stop
// Check Joystick with       'jstest /dev/input/f710'  
// Buttons for E-Stop:        X-» 2 Y-» 3 A-» 0 B-» 1

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>

//class def
class ExtraEstop{
  public:
    ExtraEstop();
  private:
    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
    ros::NodeHandle nh_;
    bool enable_estop_;
    ros::Subscriber joy_sub_;
    ros::Publisher vel_pub_;
    ros::Publisher extra_estop_pub_;
    geometry_msgs::Twist twist_msg;
    std_msgs::Bool estop_flag;
};

//class const: initialize parameters (enable_estop_ ), create subscriber/publisher
ExtraEstop::ExtraEstop():  enable_estop_(false){
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy_teleop/joy", 10, &ExtraEstop::joyCallback, this);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("xestop_vel", 1);
  extra_estop_pub_ = nh_.advertise<std_msgs::Bool>("e_stop", 1);

  // msgs to be sent: zero velocities and true to pause_nav
  twist_msg.angular.x = 0; twist_msg.angular.y = 0; twist_msg.angular.z = 0;
  twist_msg.linear.x = 0; twist_msg.linear.y = 0; twist_msg.linear.z = 0;
  estop_flag.data = false;
}

void ExtraEstop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  extra_estop_pub_.publish(estop_flag);
  enable_estop_ = joy->buttons[0]||joy->buttons[1]||joy->buttons[2]||joy->buttons[3];
  if (enable_estop_){
    vel_pub_.publish(twist_msg);
    estop_flag.data = true;
    extra_estop_pub_.publish(estop_flag);
    ROS_INFO("E-Stop Button pressed");
  } 
}


int main(int argc, char** argv){
  ros::init(argc, argv, "mux_estop");
  ExtraEstop extra_Estop;
  ROS_INFO("Emergency Stop node: ready ");
  ros::spin();
}