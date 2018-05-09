#include "ros/ros.h"
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <ros/console.h>
#include "point.h"

#include "urg_node/SegmentArray.h"
#include "std_msgs/Byte.h"

using namespace std;

#define STOP 7
#define GOBACK 8
#define NO_GO_RADIUS 0.9

bool dataReady;

urg_node::SegmentArray all;

void segmentCallback(const urg_node::SegmentArray::ConstPtr& msg) {
	all = *msg;
	dataReady = true;
}

vector<float> closestPointOf(const urg_node::Segment& segment){

	vector<float> temp;

	float x = segment.x;
	float y = segment.y;
	float length = segment.length;
	float orientation = segment.orientation;

	float headX = x - 0.5*length*cos(orientation);
	float tailX = x + 0.5*length*cos(orientation);
	float headY = y - 0.5*length*sin(orientation);
	float tailY = y + 0.5*length*sin(orientation);

	float head = sqrt(headX*headX+headY*headY);
	float tail = sqrt(tailX*tailX+tailY*tailY);

	if (head < tail){
		temp.push_back(head);
		temp.push_back(headX);
		temp.push_back(headY);
	} else {
		temp.push_back(tail);
		temp.push_back(tailX);
		temp.push_back(tailY);
	}
	return temp;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "short_range_nav");
	ros::NodeHandle n;
	ros::Subscriber segmentSub = n.subscribe<urg_node::SegmentArray>("/segments", 1, segmentCallback);
	ros::Publisher navPub=n.advertise<std_msgs::Byte>("/shortRangeNav",1);

	std_msgs::Byte nav;
	nav.data = 0;

	while(ros::ok())
	{
		if (dataReady){
			dataReady = false;

			urg_node::Segment segment;

			for(int i = 0; i < all.segments.size(); i++) {
				segment = all.segments[i];
				//ROS_INFO_STREAM("x: " << data.x << " y: " << data.y << " length: " << data.length << " Orientation: " << data.orientation);
				if (closestPointOf(segment)[0] < NO_GO_RADIUS){
					nav.data |= GOBACK; //go backward
				}

				float x = closestPointOf(segment)[1];
				float y = closestPointOf(segment)[2];
				if (y < 1.3){
					if (x > -1.0 && x < -0.15){
						nav.data |= 4; //No Left Turn
					}
					if (x > 0.4 && x < -0.4){
						nav.data |= 2; //No Driving Forward
					}
					if (x > 0.15 && x < 1.0){
						nav.data |= 1; //No Right Turn
					}
				}
			}
			navPub.publish(nav);			
			nav.data = nav.data << 4;
		}
		ros::spinOnce();
	}
}
