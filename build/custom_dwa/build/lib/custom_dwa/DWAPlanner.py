#!/usr/bin/env python3

import math

class DWAPlanner():
    """
    Custom Dynamic Window Approach (DWA) approach based planner for ROS2
    """
    def __init__(self, config, logger):
        self.config = config
        self.logger = logger

    def dwa_plan(self, currentTwist, currentPose, goalPose, laserData):
        """ Custom DWA Plan routine, evals all feasible velocity commands and chooses the one with the lowest cost. """
        
        stagCount = 0
        lastBestCost = float('inf')

        velWindow = self.generate_dynamic_window(currentTwist)
        
        if not velWindow:
            return None
        
        bestCost = float('inf')
        bestCmd = None
        trajArray = []
        bestCmdId = 0

        for i, (lVel, aVel) in enumerate(velWindow):
            traj = self.predict_traj(lVel, aVel)

            if not traj:
                continue
            
            cost = self.eval_traj(traj, lVel, aVel, currentPose, goalPose, laserData)
            trajArray.append((traj, cost))

            # self.logger.error(f'lVel: {lVel:.3f}, aVel: {aVel:.3f}, cost: {cost:.3f}')

            if cost < bestCost:
                bestCost = cost
                bestCmd = [lVel, aVel]
                bestCmdId = i

        # robot is stuck
        if bestCost == lastBestCost:
            stagCount += 1 
        else:
            stagCount = 0
            lastBestCost = bestCost
        
        if stagCount >= self.config.earlyStopLimit:
            self.logger.warning("DWA Goal not reachable given current pos, Waiting for next goal.")
        
        self.logger.debug(f"bestCost: {bestCost:.3f}, bestCmd: {bestCmd}")
        return bestCmd, trajArray, bestCmdId
    
    def generate_dynamic_window(self, currentTwist):
        """ Generates a set of vel commds based on current vel and accl limits defined in DWAConfig """
        if currentTwist is None:
            currentLv, currentAv = 0.0, 0.0
        else:
            currentLv = currentTwist.linear.x
            currentAv = currentTwist.angular.z

        dlv = self.config.maxDlv * self.config.dt
        dav = self.config.maxDav * self.config.dt

        minLv = max(self.config.minLv, currentLv - dlv)
        maxLv = min(self.config.maxLv, currentLv + dlv)
        minAv = max(self.config.minAv, currentAv - dav)
        maxAv = min(self.config.maxAv, currentAv + dav)

        lvStep = (maxLv - minLv) / max(1, self.config.lvSamples - 1)
        avStep = (maxAv - minAv) / max(1, self.config.avSamples - 1)
        
        # self.logger.debug(f'avStep: {avStep:.3f}')
        return [(minLv + i * lvStep, minAv + j * avStep) for i in range(self.config.lvSamples) for j in range(self.config.avSamples)]

    def predict_traj(self, lv, av):
        """ Sample robot trajectory over a short horizon """

        # Numerical Integration (Euler)
        traj = []
        x, y, theta = 0.0, 0.0, 0.0
        steps = int(self.config.predictionTime / self.config.dt)

        for _ in range(steps):
            x += lv * math.cos(theta) * self.config.dt
            y += lv * math.sin(theta) * self.config.dt
            theta += av *  self.config.dt
            traj.append(((x, y)))
        return traj

    def eval_traj(self, traj, lv, av, currentPose, goalPose, laserData):
        """ Calculate cost of a traj using a weighted linear sum of goal dist, obst proximity, and vel """
        if not traj:
            return float('inf')

        goalCost = self.compute_goal_cost(traj[-1], currentPose, goalPose)
        obsCost = self.compute_obs_cost(traj, laserData)
        velCost = self.compute_vel_cost(lv, av)

        if obsCost == float('inf'):
            return float('inf') # Collision
        

        return (self.config.wGoal * goalCost + self.config.wObs * obsCost + velCost)

    def compute_goal_cost(self, endpoint, currentPose, goalPose):
        """ Return euclid dist from predicted endpoint and goal """
        if goalPose is None or currentPose is None:
            return float('inf')

        cp = currentPose.position
        orientation = currentPose.orientation
        yaw = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y*orientation.y + orientation.z * orientation.z)
            )

        ex = cp.x + endpoint[0] * math.cos(yaw) - endpoint[1] * math.sin(yaw)
        ey = cp.y + endpoint[0] * math.sin(yaw) + endpoint[1] * math.cos(yaw)

        dx = goalPose.position.x - ex
        dy = goalPose.position.y - ey

        return math.sqrt( dx*dx + dy*dy )

    def compute_obs_cost(self, traj, laserData):
        """ Check traj safety with /scan data, return inv_dist to closest obstacle """
        if laserData is None:
            return 0.0

        minDist = float('inf')

        for x, y in traj:
            ang = math.atan2( y , x )
            idx = self.angle_to_laser_index(ang, laserData)
            if 0 <= idx < len(laserData.ranges):
                lDist = laserData.ranges[idx]

                if lDist < float('inf') and self.config.robotRadius > lDist:
                    return float('inf') # Collision

                if lDist < float('inf'):
                    buffer = lDist - self.config.robotRadius

                    if buffer >= 0:
                        minDist = min(minDist, buffer)
        
        return 1.0 / max(minDist, 0.01) if minDist < self.config.minObsDist else 0.0

    def angle_to_laser_index(self, angle, laserData):
        """ Convert continuous angle to discrete index """
        if laserData is None:
            return -1
        angle = angle - laserData.angle_min
        index = int(angle / laserData.angle_increment)
        return max(0, min(index, len(laserData.ranges) - 1))
        
    def compute_vel_cost(self, lv, av):
        """ Returns normalized cost for velocity mag """
        return self.config.wLVel * abs (lv) / self.config.maxLv + self.config.wAVel * abs(av) / self.config.maxAv

    
