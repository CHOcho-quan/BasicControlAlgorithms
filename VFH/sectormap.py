import math

class SectorMap:
    """
    This is a sector map class that consist the cv information of the map
    
    """
    def __init__(self, scan_distance_max = 2.1, scan_distance_min = 0.1,
                    angle_resolution = 1.0, heading = 90, sector_value = 15,
                    sector_scale = 30, Uavpx = 0, Uavpy = 0):
        """
        Initialization of the class
        Input
        scan_distance_max - laser max scan distance 
        scan_distance_min - laser min scan distance
        angle_resolution - angle resolution of the given laser
        heading - the heading position of UAV or say Yaw angle
        sector_value - the angle value of each sector
        sector_scale - the scanning scale of the sector
        
        """
        self.scan_distance_max = scan_distance_max
        self.scan_distance_min = scan_distance_min
        self.angle_resolution = angle_resolution
        self.heading = heading
        self.sector_value = sector_value
        self.sector_scale = sector_scale
        self.UAVx = Uavpx
        self.UAVy = Uavpy

        self.sectors = 360 / sector_value
        self.MapCV = sectors * [0]
        self.lRanges = []

    def SetUAVHeading(self, hd):
        """
        Set the UAV heading, setting y as 0 degree
        Input 
        hd - Given heading of global
        
        """
        return 360 - (hd + 270) % 360

    def SetUAVPosition(self, x, y):
        """
        Setting the UAV position
        Input
        x - UAV position x
        y - UAV position y
        
        """
        self.UAVx = x
        self.UAVy = y

    def ComputeMV(self, ranges):
        """
        Input
        Ranges - the gotten value of the laser scan ranges
        Output - Calculated MapCV & lRanges
        
        """
        dist = []
        for i in range(len(ranges)):
            if ranges[i] is not None:
                scan_distance = ranges[i]
                sector_index = (i * self.angle_resolution) / self.sector_value

                # We don't want the invalid data
                if (scan_distance > self.scan_distance_max or scan_distance < self.scan_distance_min):
                    scan_distance = 0
                else:
                    # We do a weight sum that further means safer
                    scan_distane = self.scan_distance_max - scan_distance

                dist.append(scan_distance)
            
            self.lRanges.append(ranges[i])
        
        for i in range(0, len(dist)):
            self.MapCV[i/sectors] += dist[i]
                
    def IsFrontSafe(self):
        """
        Judge if the former position is safe
        
        """
        # Here we mean -20 - 20 degree of the car position
        total_distance = 0.0
        start_index = ((0 - (self.sector_scale) + 360) % 360) / self.angle_resolution
        for i in range(0, (self.sector_scale) * 2 / self.angle_resolution):
            if ranges[i] is None: 
                continue
            real_index = (start_index + i) % (360 / self.angle_resolution)

            if (ranges[i] > self.scan_distance_max or ranges[i] < self.scan_distance_min):
                total_distance += 0
            else:
                total_distance += self.scan_distance_max - ranges[i]

        if (total_distance < 0.1): return True
        else: return False

    def CalculateDir(self, goalx, goaly):
        """
        Calculate the disired direction of the goal place
        Input
        goalx - goal position x of world frame
        goaly - goal position y of world frame
        
        """
        theta = math.atan(float(goaly - self.UAVy) / (goalx - self.UAVx))
        ori = theta * 180 / 3.1415926

        # Transform to body frame
        ori -= self.heading
        ori = (360 + ori) % 360

        # Calculate the CV of oriented direction
        start_index = ((ori - self.sector_scale + 360) % 360) / self.angle_resolution
        scan_dis = 0.0
        for i in range(0, sector_scale * 2 / self.angle_resolution):
            real_index = (start_index + i) % (360 / angle_resolution)
            if self.ranges[real_index] is not None and self.ranges[real_index] < self.scan_distance_max and self.ranges[real_index > self.scan_distance_min]:
                scan_dis += self.ranges[real_index]

        if (scan_dis < 0.1):
            ori = (ori + self.heading) % 360
            return ori 
        
        # Calculate Mesh value
        mesh = []
        for i in range(0, len(self.MapCV)):
            if self.MapCV < 0.1:
                mesh.append(0)
            elif self.MapCV < 0.3:
                mesh.append(2)
            else: mesh.append(4)

        # Calculate the candidates
        candidate = []
        for i in range(0, len(mesh)):
            if i != len(mesh) -1 and mesh[i] + mesh[i+1] == 0:
                candidate.append((i+1)*sector_value)
            elif i == len(mesh) - 1 and mesh[i] + mesh[0] == 0:
                candidate.append(0)

        if len(candidate) != 0:
            best_candidate_d = 2147483647
            best_cand = 0
            for cand in candidate:
                delta = min(abs(ori - cand), 360 - (ori - cand))
                if delta < best_candidate_d:
                    best_candidate_d = delta
                    best_cand = cand
            return (best_cand + heading) % 360

        return -1 
