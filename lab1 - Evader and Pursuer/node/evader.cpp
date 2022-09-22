#include <ros/ros.h>

// Publish to a topic with this message type
#include <ackermann_msgs/AckermannDriveStamped.h>
// AckermannDriveStamped messages include this message type
#include <ackermann_msgs/AckermannDrive.h>

// Subscribe to a topic with this message type
#include <sensor_msgs/LaserScan.h>

// for printing
#include <iostream>

// for RAND_MAX
#include <cstdlib>

class Evader {
private:
    // A ROS node
    ros::NodeHandle n;

    // car parameters
    double max_speed, max_steering_angle;
    
    // lidar parameters
    
    double scan_beams, scan_field_of_view;

    // Listen for scan messages
    ros::Subscriber laser_sub;

    // Publish drive data
    ros::Publisher drive_pub;

    // previous desired steering angle
    double prev_angle=0.0;


public:
    Evader() {
        // Initialize the node handle
        n = ros::NodeHandle("~");

        // get topic names
        std::string drive_topic, odom_topic, scan_topic;
        n.getParam("evader_topic", drive_topic);
        n.getParam("scan_topic", scan_topic);

        // get car parameters
        n.getParam("max_speed", max_speed);
        n.getParam("max_steering_angle", max_steering_angle);
        
        // get lidar parameters
        n.getParam("scan_beams", scan_beams);
        n.getParam("scan_field_of_view", scan_field_of_view);

        // Make a publisher for drive messages
        drive_pub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);

        // Start a subscriber to listen to odom messages
	    laser_sub = n.subscribe(scan_topic, 1, &Evader::laser_callback, this);

    }


    void laser_callback(const sensor_msgs::LaserScan & msg) {

        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        drive_msg.speed = max_speed;
        double random = ((double) rand() / RAND_MAX);
        double rand_ang = max_steering_angle * random - max_steering_angle / 2.0;
        for (size_t i = 0; i < msg.ranges.size(); i++) {
            if(msg.ranges[i] < 2.0){
                drive_msg.steering_angle = rand_ang;
            }
            else{
                drive_msg.steering_angle = 0;
            }
        }
        drive_st_msg.drive = drive_msg;
        drive_pub.publish(drive_st_msg);
    }
};


int main(int argc, char ** argv) {
    ros::init(argc, argv, "evader_drive");
    Evader rw;
    ros::spin();
    return 0;
}
