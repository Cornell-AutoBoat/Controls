import numpy as np
import src.SFR as SFR

def follow(waypoints):
    for targetPt in waypoints:
            # calculate absTargetAngle with the atan2 function
        absTargetAngle = math.atan2(
            targetPt[1]-SFR.tz, targetPt[0]-SFR.tx)
        if absTargetAngle < 0:
            absTargetAngle += 2*np.pi
        # compute turn error by finding the minimum angle
        turnError = find_min_angle(absTargetAngle, currentHeading)

        while np.abs(turnError) > np.pi / 4:
            if turnError > 0:
                thruster_utils.sendValue(1400, 1600)
            else:
                thruster_utils.sendValue(1600, 1400)
            time.sleep(0.5)
        
        start = time.time()
        thruster_utils.sendValue(1700, 1700)
        while (np.sqrt(targetPt[1]-currentPos[1])**2 + (targetPt[0]-currentPos[0])**2) > 1 or time.time() - start > 3:
            pass
        thruster_utils.sendValue(1300, 1300)
        time.sleep(1)
        thruster_utils.sendValue(1500, 1500)


