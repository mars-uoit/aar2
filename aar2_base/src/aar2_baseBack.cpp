#include "ros/ros.h"
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "roboteq_msgs/Command.h"
#include "roboteq_msgs/Feedback.h"
#include "tf/tf.h"
#include <message_filters/sync_policies/approximate_time.h>

//#include <tf/transform_broadcaster.h>

#include <string>
#include <cmath>


using std::string;
using namespace roboteq_msgs;
using namespace message_filters::sync_policies;

typedef message_filters::sync_policies::ApproximateTime<roboteq_msgs::Feedback,
					       roboteq_msgs::Feedback> SyncPolicy;

//ros::Publisher odom_pub;
ros::Publisher velocityL;
ros::Publisher velocityR;
//tf::TransformBroadcaster *odom_broadcaster;

//static double ENCODER_RESOLUTION = 250*4; //dont need
double wheel_circumference = 1.036828;
double wheel_base_length = 1.0922;
double wheel_diameter = 0.3302;
double encoder_poll_rate;
std::string odom_frame_id;
size_t error_count;
double target_speed = 0.0;
double target_direction = 0.0;
double twist_factor = 10.0;
double rot_cov = 0.0;
double pos_cov = 0.0;

static double L_MAX = 1000.0 * 3.74;
static double R_MAX = 1000.0 * 3.74;

// Persistent variables
double prev_x = 0, prev_y = 0, prev_w = 0;
ros::Time prev_time;
long prevEnc1=0, prevEnc2=0;

double wrapToPi(double angle) {
    angle += M_PI;
    bool is_neg = (angle < 0);
    angle = fmod(angle, (2.0*M_PI));
    if (is_neg) {
        angle += (2.0*M_PI);
    }
    angle -= M_PI;
    return angle;
}

void cmd_velCallback(const geometry_msgs::Twist::ConstPtr& msg) {

    // Convert mps to rpm
    double A = msg->linear.x;
    double B = msg->angular.z * (wheel_base_length/2.0);
    
    // Factor for linear is tuned (1000.0/23.0 * 3.74), angular is not yet tuned try 1.0 atm
    double A_rads = A*2 / wheel_diameter * 1000.0/23.0 * 3.74;
    double B_rads = B*2 / wheel_diameter * 1000.0/23.0 * 3.74;
    
    // Convert rpm to relative
    double Left = (A_rads-B_rads) ;
    double Right = (A_rads+B_rads) ;
    
    // ROS_INFO("Arpm: %f, Arel: %f, Brpm: %f, Brel: %f", A_rpm, A_rel, B_rpm, Right);
    
    // Bounds check
    if(Left > L_MAX)
        Left = L_MAX;
    if(Left < -1*L_MAX)
        Left = -1*L_MAX;
    if(Right > R_MAX)
        Right = R_MAX;
    if(Right < -1*R_MAX)
        Right = -1*R_MAX;

    //send motor command
    roboteq_msgs::Command cmdL;  
    roboteq_msgs::Command cmdR;
    cmdL.commanded_velocity=Left;
    cmdR.commanded_velocity=Right;
    velocityL.publish(cmdL);
    velocityR.publish(cmdR);
    
}

/*void controlLoop() {
    // ROS_INFO("Relative move commands: %f %f", target_speed, target_direction);
    try {
     mc->move(target_speed, target_direction);
    } catch(const std::exception &e) {
     if (string(e.what()).find("did not receive") != string::npos
         || string(e.what()).find("failed to receive an echo") != string::npos) {
            ROS_WARN("Error commanding the motors: %s", e.what());
     } else {
            ROS_ERROR("Error commanding the motors: %s", e.what());
            mc->disconnect();
     }
    }
}*/

/*void errorMsgCallback(const std::string &msg) {
    ROS_ERROR("%s", msg.c_str());
}

void warnMsgCallback(const std::string &msg) {
    ROS_WARN("%s", msg.c_str());
}

void infoMsgCallback(const std::string &msg) {
    ROS_INFO("%s", msg.c_str());
}

void debugMsgCallback(const std::string &msg) {
    ROS_DEBUG("%s", msg.c_str());
}*/

void queryEncoders(const roboteq_msgs::Feedback::ConstPtr& left_msg, const roboteq_msgs::Feedback::ConstPtr& right_msg) {

    long encoder1, encoder2;
    ros::Time now = ros::Time::now();
    try{encoder1=left_msg->measured_position;}    
        catch(const std::exception &e) {ROS_WARN("Trouble reading left encoder"); return;}
    try{encoder2=right_msg->measured_position;}    
        catch(const std::exception &e) {ROS_WARN("Trouble reading right encoder"); return;}
    
    double delta_time = (now - prev_time).toSec();
    prev_time = now;
    
    // Convert to rads for each wheel from delta encoder ticks
    double left_v = encoder1-prevEnc1;
    prevEnc1=encoder1;
    if (abs(left_v)>100000) {ROS_INFO("Left Encoder Wrap Around"); return;} // Position is reported in rads, and wraps around +-6M 
    left_v /= delta_time;
	 left_v /= 82; //82:1 gear ratio

    double right_v = encoder2 -prevEnc2;
    prevEnc2=encoder2;
    if (abs(right_v)>100000) {ROS_INFO("Right Encoder Wrap Around"); return;}
    right_v /= delta_time;
	 right_v /= 82; //82:1 gear ratio
    
    

    double v = 0.0;
    double w = 0.0;
    
    double r_L = wheel_diameter/2.0;
    double r_R = wheel_diameter/2.0;
    
    v += r_L/2.0 * left_v;
    v += r_R/2.0 * right_v;

    w += r_R/wheel_base_length * right_v;
    w -= r_L/wheel_base_length * left_v;

    
    // Update the states based on model and input
    prev_x += delta_time * v * cos(prev_w + delta_time * (w/2.0)); // Why w/2.0?
    
    prev_y += delta_time * v * sin(prev_w + delta_time * (w/2.0));
    prev_w += delta_time * w;
    prev_w = wrapToPi(prev_w);
    
    // ROS_INFO("%f", prev_w);
    
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(prev_w);
    
    // Populate the msg
    /*nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = now;
    odom_msg.header.frame_id = odom_frame_id;
    odom_msg.pose.pose.position.x = prev_x/2.0;
    odom_msg.pose.pose.position.y = prev_y/2.0;
    odom_msg.pose.pose.orientation = quat;
    odom_msg.pose.covariance[0] = pos_cov;
    odom_msg.pose.covariance[7] = pos_cov;
    odom_msg.pose.covariance[14] = 1e100;
    odom_msg.pose.covariance[21] = 1e100;
    odom_msg.pose.covariance[28] = 1e100;
    odom_msg.pose.covariance[35] = rot_cov;
    
    // odom_msg.twist.twist.linear.x = v/delta_time;
    odom_msg.twist.twist.linear.x = v;
    // odom_msg.twist.twist.angular.z = w/delta_time;
    odom_msg.twist.twist.angular.z = w;
    //{ROS_WARN("Publish odom"); return;}
    odom_pub.publish(odom_msg);
    //{ROS_WARN("Odom published"); return;}
    // TODO: Add TF broadcaster**
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = now;
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    
    odom_trans.transform.translation.x = prev_x;
    odom_trans.transform.translation.y = prev_y;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = quat;
    
    odom_broadcaster->sendTransform(odom_trans);*/
}

int main(int argc, char **argv) {
    // Node setup
    ros::init(argc, argv, "aar2_base");
    ros::NodeHandle n;
    prev_time = ros::Time::now();
    
    // Serial port parameter
    //std::string port;
    //n.param("serial_port", port, std::string("/dev/motor_controller"));
    
    // Wheel diameter parameter
    n.param("wheel_diameter", wheel_diameter, 0.3302);
    
    wheel_circumference = wheel_diameter * M_PI;
    
    // Wheel base length
    n.param("wheel_base_length", wheel_base_length, 1.0922);
    
    // Odom Frame id parameter
    n.param("odom_frame_id", odom_frame_id, std::string("odom"));

    // Load up some covariances from parameters
    n.param("rotation_covariance",rot_cov, 1.0);
    n.param("position_covariance",pos_cov, 1.0);
    
    // Setup Encoder polling
    //n.param("encoder_poll_rate", encoder_poll_rate, 25.0);
    //ros::Rate encoder_rate(encoder_poll_rate);
    
    // Odometry Publisher
    //odom_pub = n.advertise<nav_msgs::Odometry>("odom", 5);
    
    // Motor CMD Publisher hard coded for now
    velocityL = n.advertise<roboteq_msgs::Command>("LeftBack/cmd", 10);
    velocityR = n.advertise<roboteq_msgs::Command>("RightBack/cmd", 10);
    
    // TF Broadcaster**
    //odom_broadcaster = new tf::TransformBroadcaster;
    
    // cmd_vel Subscriber
    ros::Subscriber sub = n.subscribe("cmd_vel", 1, cmd_velCallback);

    message_filters::Subscriber<roboteq_msgs::Feedback> left_sub(n, "LeftBack/feedback", 1);
    message_filters::Subscriber<roboteq_msgs::Feedback> right_sub(n, "RightBack/feedback", 1);
    //message_filters::TimeSynchronizer<roboteq_msgs::Feedback, roboteq_msgs::Feedback> sync(left_sub, right_sub, 10);
    message_filters::Synchronizer<SyncPolicy> sync (SyncPolicy(5), left_sub, right_sub);
    sync.registerCallback(boost::bind(&queryEncoders, _1, _2));
    
    // Spinner
    //ros::AsyncSpinner spinner(1);
    //spinner.start();

    ros::spin();
    
    return 0;
}

