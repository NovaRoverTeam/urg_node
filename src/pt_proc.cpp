#include "ros/ros.h"
#include <stdlib.h>
#include <math.h>
#include <iostream>
#include <algorithm>
#include <ros/console.h>
#include "point.h"

#include "sensor_msgs/LaserScan.h"
//#include "urg_node/Segment.h"
#include "urg_node/SegmentArray.h"

using namespace std;

#define MIN_RHO 0.07
#define SEPARATION 0.05
#define AXIS_THRES 0.01
#define MIN_PTS 5
#define WINDOW 1
#define RVIZ_SHOW 1

#ifdef RVIZ_SHOW
#define NUM_OF_SCAN_POINTS 683
#endif


float angle_min, angle_increment;
float rviz_angle_min = -M_PI/6, rviz_angle_max = 7*M_PI/6;
bool dataReady;
vector<Point> pts;


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
	angle_min = scan->angle_min;
	angle_increment = scan->angle_increment;

	for(int i = 0; i < scan->ranges.size(); i++) {
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
	vector<Segment> clusters;

	while(it0 < pts.end() & it1 < pts.end()){
		Point pt0 = *(it1 - 1), pt1 = *it1;

		if (pt1 - pt0 <= SEPARATION){
			it1++;
		} else {
			temp.assign(it0,it1);
			clusters.push_back(Segment(temp));
			it0 = it1 + 1;
			it1 = it0 + 1;
		}
	}

	temp.assign(it0,it1);
	clusters.push_back(Segment(temp));

	return clusters;
}

vector<Segment> split(const Segment& cluster){

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

void recursiveSplit(Segment& cluster, vector<Segment>* list){
	if (cluster.axis(true) <= AXIS_THRES) list->push_back(cluster);
	else {
		vector<Segment> twoSegs = split(cluster);

		recursiveSplit(twoSegs[0], list);
		recursiveSplit(twoSegs[1], list);
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "pt_process");
	ros::NodeHandle n;
	ros::Subscriber scanSub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1, laserCallback);
	ros::Publisher segmentPub=n.advertise<urg_node::SegmentArray>("/segments", 1);

	#ifdef RVIZ_SHOW
	ros::Publisher processPub=n.advertise<sensor_msgs::LaserScan>("/processScan",1);
	#endif

	while(ros::ok())
	{
		if (dataReady){
			dataReady = false;

			urg_node::Segment oneSegment;
			urg_node::SegmentArray all;
			
			vector<Segment> clusters = cluster(pts);
			vector<Segment> listOfSegments0, listOfSegments;

			vector<Segment>::iterator it;

			for (it = clusters.begin()+WINDOW ; it != clusters.end()-WINDOW; ++it){
				recursiveSplit(*it, &listOfSegments0);
			}

			for (it = listOfSegments0.begin() ; it != listOfSegments0.end(); ++it){
				if (it->pts.size() > MIN_PTS) listOfSegments.push_back(*it);
			}

			for (it = listOfSegments.begin(); it != listOfSegments.end(); ++it){
				oneSegment.x = it->centerX();
				oneSegment.y = it->centerY();
				oneSegment.length = it->length();
				oneSegment.orientation = it->orient();

				all.segments.push_back(oneSegment);

				/*float x = it->centerX();
				float y = it->centerY();
				float length = it->length();
				float orient = it->orient();
				int orientDegrees = int(it->orient() * 180 / M_PI);
				ROS_INFO("%f %f %f %n", x, y, length, orientDegrees);*/
			}

			segmentPub.publish(all);

			#ifdef RVIZ_SHOW

			sensor_msgs::LaserScan processed;
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

			processPub.publish(processed);
			#endif

			pts.clear();
		}


		ros::spinOnce();
	}
}
