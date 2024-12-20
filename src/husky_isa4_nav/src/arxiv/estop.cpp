// Listener of joystick for autonomous navigation. Acts as exta Emergency stop
// Listens to buttons X, Y, A or B and when pressed, kill the amcl node (move_base node)
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
    std_msgs::Bool estop_flag;
};

//class constructor: initialize some parameters (enable_estop_ ), create subscriber/publisher
ExtraEstop::ExtraEstop():  enable_estop_(false){
  joy_sub_ = nh_.subscribe<sensor_msgs::Joy>("/joy_teleop/joy", 10, &ExtraEstop::joyCallback, this);
  vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/husky_velocity_controller/cmd_vel", 1);
  extra_estop_pub_ = nh_.advertise<std_msgs::Bool>("/amcl/extra_estop", 1);
  estop_flag.data = false;
}

void ExtraEstop::joyCallback(const sensor_msgs::Joy::ConstPtr& joy){
  enable_estop_ = joy->buttons[0]||joy->buttons[1]||joy->buttons[2]||joy->buttons[3];
  if (enable_estop_){
    geometry_msgs::Twist twist;
    twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = 0;
    twist.linear.x = 0; twist.linear.y = 0; twist.linear.z = 0;
    vel_pub_.publish(twist);

    estop_flag.data = true;
    extra_estop_pub_.publish(estop_flag);
    ROS_INFO("E-Stop Button pressed");
  } 
}


int main(int argc, char** argv){
  ros::init(argc, argv, "extra_estop");
  ExtraEstop extra_Estop;
  ROS_INFO("Emergency Stop node: ready ");
  ros::spin();
}