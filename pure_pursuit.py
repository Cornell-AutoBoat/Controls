"""
This file utilizes the Purdue SIGBot's pure pursuit code: https://wiki.purduesigbots.com/software/control-algorithms/basic-pure-pursuit
and their waypoint injecting and smoothing code here: https://colab.research.google.com/drive/10FAZWfpz0TzWq7MdG9dbxUq_ZiDGPt_w?usp=sharing#scrollTo=ZZXBRKnsVxAL 
Given a list of waypoints the boat must pass through (and other parameters), 
the controller output the desired angular velocity of the boat, which is then transformed into singals to the motors,
until the boat has completed the path or exceeded a time limit.
"""

import numpy as np
import SFR
from thruster_utils import send_value
# from src.path_execution import pid
import time
import math
import rospy
from simple_pid import PID
from test.msg import Done


def sgn(num):
    # helper function: sgn(num)
    # returns -1 if num is negative, 1 otherwise
    if num >= 0:
        return 1
    else:
        return -1


def find_min_angle(absTargetAngle, currentHeading):

    # minAngle = absTargetAngle - currentHeading
    minAngle = currentHeading - absTargetAngle

    if minAngle > np.pi or minAngle < -np.pi:
        minAngle = -1 * sgn(minAngle) * (2*np.pi - abs(minAngle))

    return minAngle


def pt_to_pt_distance(pt1, pt2):
    # helper function: pt_to_pt_distance(pt1, pt2)
    # computes euclidean distance between two points
    distance = np.sqrt((pt2[0] - pt1[0])**2 + (pt2[1] - pt1[1])**2)
    return distance


def move_to_point_step(currentPos, currentHeading, targetPt, desLin, maxAng, pid_heading):

    # initialize proportional controller constant
    Kp_lin = 1
    Kp_turn = 1

    # compute linear error
    linearError = np.sqrt(
        (targetPt[1]-currentPos[1])**2 + (targetPt[0]-currentPos[0])**2)

    # calculate absTargetAngle with the atan2 function
    absTargetAngle = math.atan2(
        targetPt[1]-currentPos[1], targetPt[0]-currentPos[0])
    if absTargetAngle < 0:
        absTargetAngle += 2*np.pi

    # compute turn error by finding the minimum angle
    turnError = find_min_angle(absTargetAngle, currentHeading)

    # apply proportional controller
    # linearVel = np.clip(Kp_lin * linearError, -desLin, desLin)
    if turnError > 1.5 or turnError < -1.5:
        thrust_lin = 1500
    else:
        thrust_lin = desLin

    thrust_ang_offset = pid_heading(turnError)

    # turnVel = np.clip(Kp_turn * turnError, -maxAng, maxAng)

    return thrust_lin + thrust_ang_offset, thrust_lin - thrust_ang_offset


def pure_pursuit_step(path, currentPos, lookAheadDis, LFindex):

    # extract currentX and currentY
    currentX = currentPos[0]
    currentY = currentPos[1]

    # use for loop to search intersections
    lastFoundIndex = LFindex
    foundIntersection = False
    startingIndex = lastFoundIndex

    goalPt = [path[lastFoundIndex][0], path[lastFoundIndex][1]]

    for i in range(startingIndex, len(path)-1):

        # beginning of line-circle intersection code
        x1 = path[i][0] - currentX
        y1 = path[i][1] - currentY
        x2 = path[i+1][0] - currentX
        y2 = path[i+1][1] - currentY
        dx = x2 - x1
        dy = y2 - y1
        dr = math.sqrt(dx**2 + dy**2)
        D = x1*y2 - x2*y1
        discriminant = (lookAheadDis**2) * (dr**2) - D**2

        if discriminant >= 0:
            sol_x1 = (D * dy + sgn(dy) * dx * np.sqrt(discriminant)) / dr**2
            sol_x2 = (D * dy - sgn(dy) * dx * np.sqrt(discriminant)) / dr**2
            sol_y1 = (- D * dx + abs(dy) * np.sqrt(discriminant)) / dr**2
            sol_y2 = (- D * dx - abs(dy) * np.sqrt(discriminant)) / dr**2

            sol_pt1 = [sol_x1 + currentX, sol_y1 + currentY]
            sol_pt2 = [sol_x2 + currentX, sol_y2 + currentY]
            # end of line-circle intersection code

            minX = min(path[i][0], path[i+1][0])
            minY = min(path[i][1], path[i+1][1])
            maxX = max(path[i][0], path[i+1][0])
            maxY = max(path[i][1], path[i+1][1])

            # if one or both of the solutions are in range
            if ((minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY)) or ((minX <= sol_pt2[0] <= maxX) and (minY <= sol_pt2[1] <= maxY)):

                foundIntersection = True

                # if both solutions are in range, check which one is better
                if ((minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY)) and ((minX <= sol_pt2[0] <= maxX) and (minY <= sol_pt2[1] <= maxY)):
                    # make the decision by compare the distance between the intersections and the next point in path
                    if pt_to_pt_distance(sol_pt1, path[i+1]) < pt_to_pt_distance(sol_pt2, path[i+1]):
                        goalPt = sol_pt1
                    else:
                        goalPt = sol_pt2

                # if not both solutions are in range, take the one that's in range
                else:
                    # if solution pt1 is in range, set that as goal point
                    if (minX <= sol_pt1[0] <= maxX) and (minY <= sol_pt1[1] <= maxY):
                        goalPt = sol_pt1
                    else:
                        goalPt = sol_pt2

                # only exit loop if the solution pt found is closer to the next pt in path than the current pos
                if pt_to_pt_distance(goalPt, path[i+1]) < pt_to_pt_distance([currentX, currentY], path[i+1]):
                    # update lastFoundIndex and exit
                    lastFoundIndex = i
                    break
                else:
                    # in case for some reason the robot cannot find intersection in the next path segment, but we also don't want it to go backward
                    lastFoundIndex = i+1

            # if no solutions are in range
            else:
                foundIntersection = False
                # no new intersection found, potentially deviated from the path
                # follow path[lastFoundIndex]
                goalPt = [path[lastFoundIndex][0], path[lastFoundIndex][1]]

    return goalPt, lastFoundIndex


def execute(waypoints, des_lin_vel=1.0, max_ang_vel=1.6, lookahead=1.0, sec=400):
    """
    Main movement controller using Purdue SIGBots's pure pursuit implementation. This function 
    will send signals to the motors based on the turn velocities produced to navigate along the path. The algorithm
    will keep the boat moving at des_lin_vel m/s, only changing the angular velocity
    of the boat. Pure pursuit will NOT BREAK itself, so make sure to call 
    thruster_utils.break_thrusters(sL, sR) after calling PP.

    Args:
        waypoints: n x 2 list in the form [[x1, z1], [x2, z2], ...] of desired
        global coordinates the boat passes through in order. Already injected + smoothed.
        des_lin_vel: desired constant linear velocity, m/s
        max_ang_vel: maximum angular velocity, rad/s
        lookahead: lookahead distance, impacts smoothness of movement, m
        sec: maximum time pure pursuit will run. Can be used to interrupt execution
    Returns:
        sL, sR: the final signals supplied to the motors. Can be used for breaking.
    """
    orig_path = waypoints.copy()

    rospy.loginfo("Waypoint list: " + str(waypoints))

    # path_visualizer(orig_path, waypoints, (1, 1), (-10, -5, 10, 20))

    # How close the ZED needs to be from the final waypoint for PP to stop
    goalRadius = 1
    # How far boat is from final waypoint in path
    distanceToGoal = pt_to_pt_distance(waypoints[0], waypoints[-1])
    LFindex = 0
    sL, sR = 1500, 1500
    start = time.time()

    # Initialize PID
    # TODO: TUNE THESE GAINS
    pid_heading = PID(80.0, 0.0,  16.0, setpoint=0)
    pid_heading.output_limits = (-200, 200)

    # While we are not within goalRadius m of the final waypoint or we have not
    # exceeded our time limit sec, take pure pursuit steps
    while distanceToGoal > goalRadius and time.time() - start < sec:

        # Find the lookahead point
        goalPt, LFindex = pure_pursuit_step(
            waypoints, [SFR.tx, SFR.tz], lookahead, LFindex)
        rospy.loginfo("Lookahead pt: " + str(goalPt))

        currHeading = SFR.heading
        # currHeading = -get_yaw() + np.pi/2
        # if currHeading < 0:
        #     currHeading += 2*np.pi

        thrust_lin = 1500 + 100*des_lin_vel
        
        # Calculate motor signals needed to reach lookahead point
        sL, sR = move_to_point_step(
            [SFR.tx, SFR.tz], currHeading, goalPt, thrust_lin, max_ang_vel, pid_heading)
        SFR.sL, SFR.sR = sL, sR
        send_value(sL, sR)

        rospy.loginfo("sending signals L:" + str(sL), " R:" + str(sR))

        distanceToGoal = pt_to_pt_distance([SFR.tx, SFR.tz], waypoints[-1])

    msg = Done()
    msg.done = True
    SFR.dPub.publish(msg)
