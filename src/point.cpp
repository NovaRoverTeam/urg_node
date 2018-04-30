#include "point.h"
#include <math.h>

Point::Point(unsigned int NUM,float THETA, float RHO){
	num=NUM;
	theta=THETA;
	rho=RHO;
}
Point::Point(float X, float Y){
	num=0;
	theta=atan2(Y,X);
	rho=sqrt(X*X+Y*Y);
}
float Point::x(){
	return rho*cos(theta);
}
float Point::y(){
	return rho*sin(theta);
}
float Point::operator-(const Point& rhs){
    return sqrt((this->rho)*(this->rho) + (rhs.rho)*(rhs.rho) - 2*(this->rho)*(rhs.rho)*cos(this->theta - rhs.theta));
}

Segment::Segment(const std::vector<Point>& PTS){
	pts = PTS;
	unsigned int n = pts.size();      
	float s_x = 0.0, s_y = 0.0, s_xx = 0.0, s_xy = 0.0, s_yy = 0.0;

	for (unsigned int i = 0; i < n; i++){
		Point pt = pts[i];
		s_x += pt.x();
		s_y += pt.y();

		s_xx += pt.x()*pt.x();
		s_xy += pt.x()*pt.y();
		s_yy += pt.y()*pt.y();
	}

	avg_x = s_x / n;
	avg_y = s_y / n;

	var_xx = s_xx / n - avg_x * avg_x;
	var_xy = s_xy / n - avg_x * avg_y;
	var_yy = s_yy / n - avg_y * avg_y;
}

float Segment::centerX(){
	return avg_x;
}

float Segment::centerY(){
	return avg_y;
}

float Segment::axis(bool min){
	float tr = var_xx + var_yy;
	float det = var_xx * var_yy - var_xy * var_xy;

	return sqrt(0.5 * tr + (min ? -1 : 1) * 0.5 * sqrt(tr * tr - 4 * det));
}

float Segment::orient(){
	return atan2(2*var_xy, var_xx - var_yy) * 90 / M_PI;
}

