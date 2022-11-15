#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <cmath>
#include <math.h>
#include <tf/tf.h>
using namespace std;
ros::Publisher vel_pub;

enum STATE {TURN, MOVE, STOP};
STATE state = TURN;

void odometry(const nav_msgs::Odometry::ConstPtr& msg);
vector <pair<int,int>> road;

int rows = 20;
int cols = 18;
int startX = 11;
int startY = 0;
int goalX = 5;
int goalY = 9;
int centX= (rows/2)-1;
int centY = (cols/2)-1;
int finalX, finalY;

int mapGrid[20][18] = 
		   {{0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0},
              	    {0,0,0,0,0,0,0,0,0,0,0,0,1,0,1,0,0,0},
              	    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
              	    {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
              	    {0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
              	    {0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0},
              	    {0,0,1,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0},
              	    {0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,1,1,0},
              	    {0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,1,1,1},
                    {0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,1,1},
                    {0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,1,1,1},
              	    {0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,1,0},
              	    {0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0},
              	    {0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0},
              	    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
              	    {0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0},
              	    {0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,0},
              	    {0,0,0,0,0,0,0,0,1,1,1,0,0,1,1,1,1,0},
              	    {0,0,0,0,0,0,0,1,1,1,0,0,0,1,1,1,1,0},
                   {0,0,0,0,0,0,0,0,1,1,0,0,0,1,1,1,1,1}};

int main(int argc, char **argv)
{
	ros::init(argc, argv, "astar");
	ros::NodeHandle n;

	int flag = 0;
	int count = 0;
	
	int openMap[rows][cols] = {0};
	int closedMap[rows][cols] = {0};
	int mapValue[rows][cols][3] = {0};
	pair<int,int> pathCoordinates[20][18];
	int sourceX = startX;
	int sourceY = startY;

	int matrixX = centX - goalY;
 	int matrixY = centY + goalX;
 	if(mapGrid[matrixX][matrixY] == 1)
 	{ 
  		for(int i=matrixX-1;i<=matrixX+1;i++)
  		{
   			for(int j=matrixY-1;j<=matrixY+1;j++)	
   			{
    				if(mapGrid[i][j] ==0)
    				{
    					finalX = i;
        				finalY = j;
        			}
   			}
  		} 
 	}
 	
	finalX = matrixX ;
	finalY = matrixY ;	
	
	for(int i=0;i<rows;i++)
	{
		for(int j=0;j<cols;j++)	
		{
			int changeX = abs(finalX - i);
			int changeY = abs(finalY - j); 	
		   	if(changeX < changeY) {mapValue[i][j][0] = (changeX * 14) + ((changeY - changeX) * 10);}
		   	if(changeY < changeX) {mapValue[i][j][0] = (changeY * 14) + ((changeX - changeY) * 10);}
			if(changeX == changeY) {mapValue[i][j][0] = changeX * 14;}
		  }
	 }
	 
	 while(true)
	 {	
		openMap[startX][startY] = -1;	
		int tempX, tempY, temp = INT_MAX;  
		for(int i = centX-1; i <= centX+1; i++)
		{
		  	for(int j = centY-1; j <= centY+1; j++)	
		  	{
		  		if(i<0 || i>=rows || j<0 || j>=cols) {continue;}
		  		if(mapGrid[i][j] == 1 || openMap[i][j] == -1) {continue;}
		    		if(i == centX || j == centY)
		    		{
		     			if((openMap[i][j] == 1 && (10 + mapValue[centX][centY][1] < mapValue[i][j][1])) || openMap[i][j] == 0)
		     			{
				      		mapValue[i][j][1] = 10 + mapValue[centX][centY][1];
				      		mapValue[i][j][2] = mapValue[i][j][0] + mapValue[i][j][1];
				      		pathCoordinates[i][j] = make_pair(centX, centY); 
		     			}	
		    		}
		    		else 
		    		{
		     			if((openMap[i][j] == 1 && (14 + mapValue[centX][centY][1] < mapValue[i][j][1])) || openMap[i][j] == 0)
		     			{
				      		if(i == centX-1 && j == centY-1 && mapGrid[centX][centY-1] == 1 && mapGrid[centX-1][centY] == 1){continue;}
						if(i == centX+1 && j == centY+1 && mapGrid[centX][centY+1] == 1 && mapGrid[centX+1][centY] == 1){continue;}
						if(i == centX-1 && j == centY+1 && mapGrid[centX-1][centY] == 1 && mapGrid[centX][centY+1] == 1){continue;}
						if(i == centX+1 && j == centY-1 && mapGrid[centX][centY-1] == 1 && mapGrid[centX+1][centY] == 1){continue;}
				      		mapValue[i][j][1] = 10 + mapValue[centX][centY][1];
				      		mapValue[i][j][2] = mapValue[i][j][0] + mapValue[i][j][1];
				      		pathCoordinates[i][j] = make_pair(centX,centY); 	
		     			} 
		    		}
		    		openMap[i][j] = 1;  
		  	}	
		} 
		
		for(int i=0; i<rows; i++)
		{
			for(int j=0; j<cols; j++)
			{
 				if(openMap[i][j] == 1)
 				{	
  					if(i == finalX && j == finalY){flag = 1; break;}
  					if(temp > mapValue[i][j][2])
  					{
   						temp = mapValue[i][j][2];
   						tempX = i;
   						tempY = j;
  					}
 				}	
			}
			if(flag == 1) {break;} 
		}
		if(flag == 1) {break;}
		sourceX = tempX;
		sourceY = tempY; 
		count++;
	}
	
	int nextX = finalX;
	int nextY = finalY;

	while(nextX != sourceX || nextY != sourceY)
	{
		closedMap[nextX][nextY] = 1;
		int x = nextY - centY;
		int y = centX - nextX;
		int x2 = nextX;
		int y2 = nextY;
		road.push_back(make_pair(x,y));
		nextX = pathCoordinates[x2][y2].first;
		nextY = pathCoordinates[x2][y2].second;
	}

	vel_pub = n.advertise<geometry_msgs::Twist>( "/cmd_vel", 1000);
	ros::Subscriber sub = n.subscribe("/base_pose_ground_truth",10, odometry);

	ros::Rate rate(5);
	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
  	return 0;
}

void odometry(const nav_msgs::Odometry::ConstPtr& msg)
{
	double roll, pitch, yaw;	
	geometry_msgs::Pose2D pose2d;
	geometry_msgs::Twist odom_msg;
	
 	pose2d.x = msg->pose.pose.position.x;
 	pose2d.y = msg->pose.pose.position.y;

 	tf::Quaternion q(
	msg->pose.pose.orientation.x,
	msg->pose.pose.orientation.y,
	msg->pose.pose.orientation.z,
	msg->pose.pose.orientation.w);
 	tf::Matrix3x3 m(q);
 	m.getRPY(roll, pitch, yaw);

	double angle = atan2(road.back().second - pose2d.y,road.back().first - pose2d.x);
 	double distance = sqrt(pow(road.back().second - pose2d.y, 2) + pow(road.back().first - pose2d.x, 2));

	
 	if(abs(angle - yaw) > 0.05 && state == TURN){state = TURN;}
 	else if(distance > 0.3 ){state = MOVE;}
 	else {state = STOP;}
 
 	int matrixX = int(centX - pose2d.y);
 	int matrixY = int(centY + pose2d.x);
 	int poseX, poseY;
 	if(mapGrid[matrixX][matrixY] == 1)
 	{ 
  		for(int i=matrixX-1;i<=matrixX+1;i++)
  		{
   			for(int j=matrixY-1;j<=matrixY+1;j++)	
   			{
    				if(mapGrid[i][j] ==0)
    				{
    					poseX = i;
        				poseY = j;
        			}
   			}
  		} 
 	}
 	
	poseX = matrixX ;
	poseY = matrixY ;	
 	double remaining = sqrt(pow(finalX - poseX, 2) + pow(finalY - poseY, 2));

	if(remaining <= 1)   
	{
		odom_msg.linear.x = 0.0;
		odom_msg.angular.z = 0.0;
		vel_pub.publish(odom_msg);
		return;
	} 
	if (state == TURN)
	{
		odom_msg.linear.x = 0.0;
		odom_msg.angular.z = 0.8; 
	}
	else if(state == MOVE)
	{
		odom_msg.linear.x = 0.5;
		odom_msg.angular.z = 0.0;
	}
	else if(state == STOP)
	{
		odom_msg.linear.x = 0.0;
		odom_msg.angular.z = 0.0;	
		road.pop_back();
		state = TURN;
	}
 	vel_pub.publish(odom_msg);
}
