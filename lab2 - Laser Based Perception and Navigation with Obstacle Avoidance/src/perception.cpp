#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <vector>
#include <utility>
#include <stdlib.h>
#include <time.h> 
#include <math.h>
using namespace std;

double x[1000],y[1000];
int scanLength;

void laserScanCallback(const sensor_msgs::LaserScan msg)
{
    scanLength = msg.ranges.size();
    double minAngle = msg.angle_min;
    double angleIncrement = msg.angle_increment;
    for(int i=0;i<scanLength;i++)
    {
        x[i]=0;
        y[i]=0;
    }
    for(int i=0;i<scanLength;i++)
    {
        if(msg.ranges[i] >= 3.0)
            continue;
        x[i] = msg.ranges[i]*sin(minAngle);
        y[i] = msg.ranges[i]*cos(minAngle); 
        minAngle += angleIncrement;
    }
}

int main( int argc, char** argv )
{
    ros::init(argc, argv, "perception");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/base_scan",10, laserScanCallback);   
    ros::Publisher ransac_pub = n.advertise<visualization_msgs::Marker>("/ransac_vis", 10);
    ros::Rate r(10);
    
    while (ros::ok())
    {
        visualization_msgs::Marker points,ransac_line;
        points.header.frame_id= "base_laser_link";
        points.header.stamp= ros::Time::now();
        points.ns = "perception";
        points.action= visualization_msgs::Marker::ADD;
        points.pose.orientation.w = 1.0;
        points.id = 0;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.1;
        points.scale.y = 0.1;
        points.color.g = 1.0;
        points.color.a = 1.0;
    
        ransac_line.header.frame_id= "base_laser_link";
        ransac_line.header.stamp= ros::Time::now();
        ransac_line.ns = "perception";
        ransac_line.action= visualization_msgs::Marker::ADD;
        ransac_line.pose.orientation.w = 1.0;
        ransac_line.id = 1;
        ransac_line.type = visualization_msgs::Marker::LINE_LIST;
        ransac_line.scale.x = 0.1;
        ransac_line.color.r = 1.0;
        ransac_line.color.a = 1.0;
        
        vector <pair<double,double>> pointCloud;
        for(int i=0;i<scanLength;i++)
        {
            geometry_msgs::Point p;
            p.x = x[i];
            p.y = y[i];
            if(p.x != 0 && p.y != 0)
            {
                pointCloud.push_back(make_pair(p.x,p.y));
                points.points.push_back(p);
            }
            p.z = 0;
        }
        int inliers = 0;
        srand(time(NULL));
        geometry_msgs::Point p1, p2;
        while(pointCloud.size()>10)
        {
            int indexA, indexB;   
            vector<int> tracker;
            int count = 0;
            for(int i=0; i<10; i++)
            {
                inliers=0;
                int a = rand() % pointCloud.size();  
                int b = rand() % pointCloud.size();
                while(a==b)
                    b = rand() % pointCloud.size();
                double x1 = pointCloud[a].first;
                double y1 = pointCloud[a].second;
                double x2 = pointCloud[b].first;
                double y2 = pointCloud[b].second;
                
                double threshold = 10;
                vector<int> index ;
                double dx = abs(abs(x2) - abs(x1));
                double dy = abs(abs(y2) - abs(y1));
                if(dx <= 0.05)
                {
                    for(int i=0;i<pointCloud.size();i++)
                    {
                        double x = pointCloud[i].first;
                        double y = pointCloud[i].second;
                        if(abs(x-x1) < 0.05 || abs(x-x2) < 0.05)
                        {
                            index.push_back(i);
                            inliers++;
                        }
                    }
                }
                else if(dy <= 0.05)
                {
                    for(int i=0;i<pointCloud.size();i++)
                    {
                        double x = pointCloud[i].first;
                        double y = pointCloud[i].second;
                        if(abs(y-y1) < 0.05 || abs(y-y2) < 0.05)
                        {
                            index.push_back(i);
                            inliers++;
                        }
                    }
                }
                else
                {
                    double slope = (y2-y1)/(x2-x1); 
                    double intercept = y1-(x1 * slope);
                    for(int i=0; i<pointCloud.size(); i++)
                    {
                        double x = pointCloud[i].first;
                        double y = pointCloud[i].second;
                        double distance =  abs(y-((slope*x)+intercept))*100;
                        if(distance <= threshold)
                        {
                            index.push_back(i);
                            inliers++;
                        }
                    } 
                }
                if(inliers > count)
                {
                    count = inliers;
                    tracker = index;
                    indexA = a;
                    indexB = b;
                }
            }
            vector<pair<double,double>> outlierList;
            vector<pair<double,double>> inlierList; 
            for(int i=0;i<pointCloud.size();i++)
            {
                int flag = 0;
                if(i == indexA || i == indexB)
                    continue;
                for(int j=0;j<tracker.size();j++)    
                {
                    if(i==tracker[j])
                    {
                        inlierList.push_back(make_pair(pointCloud[i].first,pointCloud[i].second));
                        flag = 1;
                        break;
                    }
                }
                if(flag == 0)
                   outlierList.push_back(make_pair(pointCloud[i].first,pointCloud[i].second));
            }
            pointCloud = inlierList; 
            double x1 = inlierList[0].first;
            double y1 = inlierList[0].second;
            double d = 0;
            int idx = 0;
            for(int i=1;i<inlierList.size();i++)
            {
                double distance;
                double x2 = inlierList[i].first;
                double y2 = inlierList[i].second; 
                distance = pow((x2-x1),2)+pow((y2-y1),2);
                if(distance > d)
                { 
                    d = distance;
                    idx = i;
                }
            }
            x1 = inlierList[idx].first;
            y1 = inlierList[idx].second;
            d = 0;
            for(int i=0;i<inlierList.size();i++)
            {
                double distance;
                double x2 = inlierList[i].first;
                double y2 = inlierList[i].second; 
                distance=pow((x2-x1),2)+pow((y2-y1),2);
                if(distance > d)
                { 
                    d = distance;
                    idx = i;
                }
            }   
            p1.x = x1;
            p1.y = y1;
            p2.x = inlierList[idx].first;
            p2.y = inlierList[idx].second;
            ransac_line.points.push_back(p1);
            ransac_line.points.push_back(p2); 
        }       
        ransac_pub.publish(points);
        ransac_pub.publish(ransac_line);
        ros::spinOnce();
        r.sleep();       
    }
}