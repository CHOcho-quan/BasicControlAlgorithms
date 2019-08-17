#ifndef SECTORMAP_H
#define SECTORMAP_H

#include <vector>

#define PI 3.1415926

using namespace std;

struct Point2D
{
	float x;
	float y;
};

class SectorMap
{
public:
	SectorMap()
	{
		scan_distance_max = 2.1;
		scan_distance_min = 0.1;
		angle_resolution = 1.0;
		heading = 90;
		sector_value = 30;
		sector_scale = 10;
		Uavp.x = 0;
		Uavp.y = 0;
	}
	virtual ~SectorMap()
	{

	}

	void ComputeMV(vector<float> r);
	float CalculDirection(Point2D& goal);
	bool IsFrontSafety();
	void SetUavPosition(Point2D& uav);
	void SetUavHeading(float hd);

	float scan_distance_max;
	float scan_distance_min;
	float angle_resolution;
	float heading;
	float sector_value;
	float sector_scale;
	Point2D Uavp;
	vector<float> map_cv;
	vector<double> ranges;
};

#endif // SPHEREMAP_H
