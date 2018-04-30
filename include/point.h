#include <vector>
using namespace std;


class Point
{
	public:
	Point(unsigned int NUM,float THETA, float RHO);
	Point(float X, float Y);
	float theta, rho;
	unsigned int num;
	float x();
	float y();
	float operator-(const Point& rhs);
};

class Segment
{
	private:
	float avg_x, avg_y, var_xx, var_xy, var_yy;
	public:
	vector<Point> pts;
	Segment(const vector<Point>& PTS);
	float centerX();
	float centerY();
	float axis(bool min);
	float orient();
};
