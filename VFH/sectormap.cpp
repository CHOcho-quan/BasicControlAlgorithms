#include "sectormap.h"

#include <cmath>
#include <algorithm>

/**
 * Class Attributes
 * float scan_distance_max - The maximum range of laser
 * float scan_distance_min - The minimum range of laser
 * float angle_resolution - The angle resolution
 * float heading - heading direction
 * float sector_value - The angle resolution of one single sector
 * float sector_scale - Scale of the angle searched
 * Point2D Uavp - UAV position
 * vector<float> map_cv - Store the certainty value
 * vector<double> ranges - Latest Laser Scan result
 * 
 * Intialization
 * scan_distance_max = 2.1
 * scan_distance_min = 0.1
 * angle_resolution = 1.0
 * heading = 90
 * sector_value = 30
 * sector_scale = 10
 * Uavp.x = 0
 * Uavp.y = 0
 */

// Setting UAV Position
void SectorMap::SetUavPosition(Point2D& uav) {
	Uavp.x = uav.x;
	Uavp.y = uav.y;
}

// Setting UAV Yaw = 360 - (270 + hd) % 360, setting y-axis as 0 degree
void SectorMap::SetUavHeading(float hd) {
	heading = hd;
	heading += 270;
	heading = (int)heading % 360;
	heading = 360 - heading;
}

void SectorMap::ComputeMV(vector<float> r) {
	/*
	Input
	r - Laser Scan result ranges, 360 size of scanned result distance
	*/
	float dist[360] = { 0 };
	ranges.clear();
	map_cv.clear();
	int range_size = r.size();

	for (size_t i = 0; i < range_size; i++)
	{
		if (!std::isnan(r[i]) && !std::isinf(r[i]))
		{
			float scan_distance = r[i];

			// A sector is 30 degree & it's divided into 12 sectors
			int sector_index = std::floor((i*angle_resolution) / sector_value);

			// Discard the data that are out of bound
			if (scan_distance >= scan_distance_max || scan_distance < scan_distance_min)
				scan_distance = 0;
			else
				// This is a weight-adding, the further the point is, the lower the score gets
				scan_distance = scan_distance_max - scan_distance;

			// This dist divided into 12 sectors and store the result cv of that sector
			dist[sector_index] += scan_distance;
		}
		ranges.push_back(r[i]);
	}

	for (int j = 0; j < (int)(360 / sector_value); j++)
	{
		map_cv.push_back(dist[j]);
	}
}

bool SectorMap::IsFrontSafety()
{
	/*
	This function calculates 340 - 380 degree
	if the given sector distance < 0.1, then it's considered not safe
	*/
	float goal_sector = (int)(0 - (sector_value - sector_scale) + 360) % 360;
	int start_index = goal_sector / angle_resolution;
	float scan_distance = 0;
	for (int i = 0; i < (sector_value - sector_scale) * 2 / angle_resolution; i++)
	{
		int real_index = (start_index + i) % (int)(360 / angle_resolution);
		if (!std::isnan(ranges[real_index]) && !std::isinf(ranges[real_index]))
		{
			if (ranges[real_index] < scan_distance_max && ranges[real_index] >= scan_distance_min)
				scan_distance = scan_distance_max - ranges[real_index] + scan_distance;
		}
	}
	if (scan_distance < 0.1)
	{
		return true;
	}

	return false;
}

float SectorMap::CalculDirection(Point2D& goal) {
	float ori;

	// Calculate the realtive goal heading direction
	float G_theta = atan2((goal.y - Uavp.y), (goal.x - Uavp.x));
	float goal_ori = G_theta * 180 / PI;
	if (goal_ori < 0)
	{
		goal_ori += 360;
	}
	goal_ori -= heading;
	goal_ori += 360;
	goal_ori = (int)goal_ori % 360;
 
	// Calculate the goal sector and judge its safetyness
	// The target sector is -30 - 30 around the goal orientation
	float goal_sector = (int)(goal_ori - sector_value + 360) % 360;
	int start_index = goal_sector / angle_resolution;
	float scan_distance = 0;
	for (int i = 0; i < sector_value * 2 / angle_resolution; i++)
	{
		int real_index = (start_index + i) % (int)(360 / angle_resolution);
		if (!std::isnan(ranges[real_index]) && !std::isinf(ranges[real_index]))
		{
			if (ranges[real_index] < scan_distance_max && ranges[real_index] >= scan_distance_min)
				scan_distance = scan_distance_max - ranges[real_index] + scan_distance;
		}
	}

	// If no obstacle, go towards the target
	if (scan_distance < 0.1)
	{
		ori = goal_ori;
		ori += heading;
		ori = (int)ori % 360;

		return ori;
	}

	// Storing the mesh value, 0 for safe, 2 for normal, 3 for dangerous
	vector<int> mesh; // size 12
	for (int i = 0; i < map_cv.size(); i++)
	{
		if (map_cv[i] < 0.1)
			mesh.push_back(0);
		else if (map_cv[i] >= 0.1 && map_cv[i] < 0.3)
			mesh.push_back(2);
		else
			mesh.push_back(4);
	}

	// Calculate candidate direction
	vector<float> cand_dir;
	for (int j = 0; j < mesh.size(); j++)
	{
		if (j == mesh.size() - 1)
		{
			if (mesh[0] + mesh[mesh.size() - 1] == 0)
				cand_dir.push_back(0.0);
		}
		else
		{
			// If two consecutive sector safe, we consider it safe
			if (mesh[j] + mesh[j + 1] == 0)
				cand_dir.push_back((j + 1)*sector_value);
		}
	}

	// Select the least error one out of those candidates
	if (cand_dir.size() != 0) {
		vector<float> delta;
		for (auto &dir_ite : cand_dir) {
			float delte_theta1 = fabs(dir_ite - goal_ori);
			float delte_theta2 = 360 - delte_theta1;
			float delte_theta = delte_theta1 < delte_theta2 ? delte_theta1 : delte_theta2;
			delta.push_back(delte_theta);
		}
		int min_index = min_element(delta.begin(), delta.end()) - delta.begin();
		ori = cand_dir.at(min_index);

		ori += heading;
		ori = (int)ori % 360;

		return ori;
	}

	return -1;
}
