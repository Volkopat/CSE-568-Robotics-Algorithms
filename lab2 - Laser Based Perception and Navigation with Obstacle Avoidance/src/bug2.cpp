#include <ros/ros.h>
#include <ros/package.h>
#include <cmath>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <iostream>
#include <stdio.h>
#include <cstdlib>
#include <tf/tf.h> 

void goalSeek();
void wallFollow();
void goalLine();

enum ROBOT_STATE {GOAL_SEEK, WALL_FOLLOW};
enum GOAL_SEEK_STATE {TURN, WALK};
enum SWITCH {ON, OFF};

ROBOT_STATE currentState = GOAL_SEEK;
GOAL_SEEK_STATE goalState = TURN;
SWITCH mux = ON;

ros::Publisher pub;

double leftFront = INT_MAX;
double front = INT_MAX;
double rightFront = INT_MAX;

double moveForward = 0.8;
double currentX = 0.0;
double currentY = 0.0;
double currentDirection = 0.0;
double adjustDirection = 0.250;

double goalX = 4.5;
double goalY = 9.0;

double wallTurn = 1;
double wallThreshold = 0.50;
double nearWall = 0.15;
double directionPrecision = 2 * (M_PI/180);
double distancePrecision = 0.2;

bool lineFlag = false;
double lineStartx = 0;
double lineStarty = 0;
double lineEndx = 0;
double lineEndy = 0;
double goalLineM = 0;
double goalLineC = 0;

double distanceToGoalLine = 0.15;
double nearGoalLine = 0.1;
double distanceToGoalFromHP = 0;
double distanceToGoalFromLP = 0;
double lphpDiff = 0;

double hitX = 0;
double hitY = 0;
double leaveX = 0;
double leaveY = 0;

double getDistance(double x1, double y1, double x2, double y2) {
	return sqrt(pow((x2 - x1), 2) + pow((y2 - y1), 2));
}

void poseCallback(const nav_msgs::Odometry & msg) {
	currentX = msg.pose.pose.position.x;
	currentY = msg.pose.pose.position.y;
	tf::Quaternion angle(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,
			msg.pose.pose.orientation.z, msg.pose.pose.orientation.w);
	currentDirection = tf::getYaw(angle);
    if(mux == ON){goalLine();}
    else{
        if(currentState = GOAL_SEEK){goalSeek();}
        else if(currentState = WALL_FOLLOW){wallFollow();}
        else{}
    }
}

void laserScanCallback(const sensor_msgs::LaserScan & msg) {
    leftFront = msg.ranges[360];
    front = msg.ranges[180];
    rightFront = msg.ranges[0];
}

void goalLine(){ //Calculating the goal line
    if(lineFlag == false){
        currentState = GOAL_SEEK;
        lineStartx = currentX;
        lineStarty = currentY;
        lineEndx = goalX;
        lineEndy = goalY;
        goalLineM = (lineEndy - lineStarty)/(lineEndx-lineStartx);
        goalLineC = lineEndy - (goalLineM*lineEndx);
        lineFlag = true;
    }
    if(currentState == GOAL_SEEK){goalSeek();}
    else if(currentState == WALL_FOLLOW){wallFollow();}
}

void goalSeek(){
    std::cout << "Current state is GOAL SEEK!" << std::endl;
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;

    if(mux == ON){
        if (leftFront < wallThreshold || front < wallThreshold || rightFront < wallThreshold){
            currentState = WALL_FOLLOW;
            hitX = currentX;
            hitY = currentY;
            distanceToGoalFromHP = getDistance(hitX, hitY, goalX, goalY);
            msg.angular.z = wallTurn;
            pub.publish(msg);
            return;
        }
    }

    if(goalState == TURN){
        double desiredDirection = atan2(goalY-currentY,goalX-currentX);
        double directionError = desiredDirection - currentDirection;
        if(fabs(directionError) > directionPrecision){
            if(directionError>0){
                msg.angular.z = adjustDirection;
            }
            else{
                msg.angular.z = -adjustDirection;
            }
            pub.publish(msg);
        }
        else{
            goalState = WALK;
            pub.publish(msg);
        }
    }
    else if(goalState = WALK){
        double positionDifference = getDistance(currentX, currentY, goalX, goalY);
        if(positionDifference > distancePrecision){
            msg.linear.x = moveForward;
            pub.publish(msg);
            double desiredDirection = atan2(goalY-currentY,goalX-currentX);
            double directionError = desiredDirection - currentDirection;
            if(fabs(directionError) > directionPrecision){
                goalState = TURN;
            }
        }
        else{
            std::cout << "Success!" << std::endl;
            pub.publish(msg);
        }
    }
    else{}
}
void wallFollow(){
    std::cout << "Current state is WALL_FOLLOW!" << std::endl;
    geometry_msgs::Twist msg;
    msg.linear.x = 0;
    msg.linear.y = 0;
    msg.linear.z = 0;
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = 0;

    if(mux == ON){
        std::cout << "STUCK!" << std::endl;
        double x = currentX;
        double y = (goalLineM*x) + goalLineC;
        distanceToGoalLine = getDistance(currentX, currentY, x, y);;
        if (distanceToGoalLine < nearGoalLine){
            double leaveX = currentX;
            double leaveY = currentY;
            distanceToGoalFromLP = getDistance(leaveX, leaveY, goalX, goalY);;
            double diff = distanceToGoalFromLP - distanceToGoalFromHP;
            if(diff > lphpDiff){
                currentState = GOAL_SEEK;
            }
        }
        return;
    }

    if(leftFront > wallThreshold && front > wallThreshold && rightFront > wallThreshold){
        msg.linear.x = moveForward; //Straight
        msg.angular.z = -wallTurn; //Right
    }
    else if(leftFront > wallThreshold && front < wallThreshold && rightFront > wallThreshold){
        msg.angular.z = wallTurn; //Left
    }
    else if(leftFront < wallThreshold && front > wallThreshold && rightFront < wallThreshold){
        msg.linear.x = moveForward; //Straight
    }
    else if(leftFront < wallThreshold && front > wallThreshold && rightFront > wallThreshold){
        msg.linear.x = moveForward; //Straight
        msg.angular.z = -wallTurn; //Right
    }
    else if(leftFront > wallThreshold && front < wallThreshold && rightFront < wallThreshold){
        msg.angular.z = wallTurn; //Left
    }
    else if(leftFront < wallThreshold && front < wallThreshold && rightFront > wallThreshold){
        msg.angular.z = wallTurn; //Left
    }
    else if(leftFront < wallThreshold && front < wallThreshold && rightFront < wallThreshold){
        msg.angular.z = wallTurn; //Left
    }
    else if(leftFront < wallThreshold && front > wallThreshold && rightFront < wallThreshold){
        msg.linear.x = moveForward; //Straight
        msg.angular.z = -wallTurn; //Right
    }
    else {}
}

int main(int argc, char ** argv) {
    ros::init(argc, argv, "perception");
    ros::NodeHandle n,n1,n2;
    ros::Subscriber scan_sub = n.subscribe("base_scan", 1000, laserScanCallback);
    ros::Subscriber goal_sub = n1.subscribe("base_pose_ground_truth", 1000, poseCallback);
    pub = n2.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
    ros::Rate r(30);
    while (ros::ok() && (getDistance(currentX, currentY, goalX, goalY) > 1.0)) {
		r.sleep();
		ros::spin();
	}
    return 0;
} 