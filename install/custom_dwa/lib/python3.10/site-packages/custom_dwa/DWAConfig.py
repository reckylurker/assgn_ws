#!/usr/bin/env python3

class DWAConfig:
    """
        Config Params for custom DWA
    """
    def __init__(self):
        # Robot Constraints = Linear Vel #
        self.minLv = -0.5
        self.maxLv = 0.5

        # Robot Constraints = Angular Vel #
        self.minAv = -1.5
        self.maxAv = 1.5
        
        # Robot Constraints = Linear Accl #
        self.maxDlv = 2.0

        # Robot Constraints = Angular Accl #
        self.maxDav = 2.5

        # DWA Sampling #
        self.lvSamples = 10
        self.avSamples = 50 # Better angular res
        
        # Prediction Time increases latency #
        self.predictionTime = 2.0 
        
        # Trajectory Sampling Time-Step #
        self.dt = 0.1

        # Cost Function #
        self.wGoal = 0.8
        self.wObs = 4.0
        self.wLVel = 0.3
        self.wAVel = 0.2
        
        # Robot Params #
        self.robotRadius = 0.2
        self.minObsDist = 0.3
        self.tolGoal = 0.2
        self.earlyStopLimit = 5 # iters to wait before "stuck"

        

