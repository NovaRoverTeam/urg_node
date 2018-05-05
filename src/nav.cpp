#include "ros/ros.h"
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <ros/console.h>
#include "point.h"

#include "sensor_msgs/LaserScan.h"
#include "std_msgs/Byte.h"

using namespace std;

// number of range data points = 683

#define NUM_OF_SCAN_POINTS 683
#define MIN_RHO 0.07
#define SEPARATION 0.05
#define AXIS_THRES 0.01
#define MIN_PTS 5
#define STOP 7
#define WINDOW 1


float angle_min, angle_increment;
float rviz_angle_min = -M_PI/6, rviz_angle_max = 7*M_PI/6;
bool dataReady;
vector<Point> pts;


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
	angle_min = scan->angle_min;
	angle_increment = scan->angle_increment;

	for(int i = 0; i < NUM_OF_SCAN_POINTS; i++) {
		float scan_data = scan->ranges[i];
		if (scan_data > MIN_RHO & !isnan(scan_data)){
			pts.push_back(Point(i,angle_min + i * angle_increment, scan_data));
		}
	}
	dataReady = true;
}

vector<Segment> cluster(const vector<Point>& pts){

	vector<Point> temp;
	vector<Point>::const_iterator it0 = pts.begin(), it1 = pts.begin()+1;
	vector<Segment> clusters, clusters0;

	while(it0 < pts.end() & it1 < pts.end()){
		Point pt0 = *(it1 - 1), pt1 = *it1;

		float a = pt1 - pt0;

		bool close = a <= SEPARATION;

		if (close){
			it1++;
		} else {
			temp.assign(it0,it1);
			clusters0.push_back(Segment(temp));
			it0 = it1 + 1;
			it1 = it0 + 1;
		}
	}

	temp.assign(it0,it1);
	clusters0.push_back(Segment(temp));

	for (unsigned int i = 0; i < clusters0.size(); i++){
		clusters.push_back(clusters0[i]);
	}

	return clusters;
}

vector<Segment> split(const Segment cluster){

	const vector<Point> clusterPt = cluster.pts;

	vector<Segment> segments;
	vector<Point> firstPts, secondPts;
	vector<Point>::const_iterator b = clusterPt.begin(), it = clusterPt.begin()+1, e = clusterPt.end();//, result = e;
	float sumaxis, smallest = 1.0;
	vector<float> axiss;

	while (it < e){

		firstPts.assign(b,it);
		secondPts.assign(it,e);

		Segment firstSeg = Segment(firstPts), secondSeg = Segment(secondPts);

		sumaxis = firstSeg.axis(true) + secondSeg.axis(true);
		if (!isnan(sumaxis)){
			axiss.push_back(sumaxis);
		} else {
			axiss.push_back(1);
		}
		/*if (!isnan(sumaxis) && sumaxis < smallest){
			smallest = sumaxis;
			result = it;
		}*/
		
		it++;
	}

	vector<float>::iterator result = min_element(axiss.begin(), axiss.end());

	it = clusterPt.begin() + distance(axiss.begin(), result) + 1;

	firstPts.assign(b,it);
	secondPts.assign(it,e);

	Segment firstSeg = Segment(firstPts), secondSeg = Segment(secondPts);

	segments.push_back(firstSeg);
	segments.push_back(secondSeg);

	return segments;
}

void recursiveSplit(Segment cluster, vector<Segment>* list){
	if (cluster.axis(true) <= AXIS_THRES) list->push_back(cluster);
	else {
		vector<Segment> twoSegs = split(cluster);

		recursiveSplit(twoSegs[0], list);
		recursiveSplit(twoSegs[1], list);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "short_range_nav");
	ros::NodeHandle n;
	ros::Subscriber scanSub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, laserCallback);
	ros::Publisher processPub=n.advertise<sensor_msgs::LaserScan>("/processScan",1);
	ros::Publisher navPub=n.advertise<std_msgs::Byte>("/shortRangeNav",1);

	std_msgs::Byte nav;
	nav.data = 0;

	while(ros::ok())
	{
		if (dataReady){

			sensor_msgs::LaserScan processed;
			
			vector<Segment>::iterator it;

			//ROS_INFO("%u",pts.size());
			vector<Segment> clusters = cluster(pts);

			vector<Segment> listOfSegments0, listOfSegments;

			for (it = clusters.begin()+WINDOW ; it != clusters.end()-WINDOW; ++it){
				recursiveSplit(*it, &listOfSegments0);
			}

			for (it = listOfSegments0.begin() ; it != listOfSegments0.end(); ++it){
				if (it->pts.size() > MIN_PTS) listOfSegments.push_back(*it);
			}

			for (it = listOfSegments.begin(); it != listOfSegments.end(); ++it){
				float x = it->centerX();
				float y = it->centerY();
				float distance = it->distance();
				//float major = it->axis(false);
				//float length = it->length();
				//ROS_INFO("%f %f %f %f", x, y, distance, major/length);
				if (distance < 0.9){
					nav.data |= 8; //go backward
				}
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


			ros::Time scan_time = ros::Time::now();
			//populate the LaserScan message
			processed.header.stamp = scan_time;
			processed.header.frame_id = "laser";
			processed.angle_min = rviz_angle_min;
			processed.angle_max = rviz_angle_max;
			processed.angle_increment = 0.00613592332229;
			processed.time_increment = 2.44140683208e-05;
			processed.range_min = 0.0;
			processed.range_max = 6.0;

			processed.ranges.resize(NUM_OF_SCAN_POINTS);
    		processed.intensities.resize(NUM_OF_SCAN_POINTS);

			for (unsigned int i = 0; i < listOfSegments.size(); i++){
				Segment seg = listOfSegments[i];
				for (unsigned int j = 0; j < seg.pts.size(); j++){

					Point point = seg.pts[j];

					processed.ranges[point.num] = point.rho;
					processed.intensities[point.num] = i;
				}
			}

			//ROS_INFO("---");

			navPub.publish(nav);
			
			nav.data = nav.data << 4;

			processPub.publish(processed);

			pts.clear();
			dataReady = false;
		}


		ros::spinOnce();
	}
}
