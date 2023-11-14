#!/usr/bin/env python3
import rospy
import SFR
import pure_pursuit
import thruster_utils
from test.msg import Done, MotionPlans, SensorReadings, Controls


def conv_msg_to_list(path):
    # msg.path is of type List[]. We want to convert it to float64[][].
    new_path = []
    for lst in path:
        new_path.append(lst.list)
    return new_path


def control_loop(msg):
    motion = msg.motion_type
    path = msg.path

    SFR.motion = motion
    SFR.path = conv_msg_to_list(path)

    if motion == "stop":
        SFR.stopped = True
        thruster_utils.break_thrusters()

    SFR.callback = True

def callback_sensors(msg):
    SFR.tx = msg.pos_x
    SFR.ty = msg.pos_y
    SFR.heading = msg.heading

def main_loop():
    while not(SFR.finish):
        SFR.callback = False
        if SFR.motion == "path":
            pure_pursuit.execute(SFR.path)
        elif SFR.motion == "fwd":
            thruster_utils.move_straight(direction="F")
        elif SFR.motion == "bwd":
            thruster_utils.move_straight(direction="B")
        elif SFR.motion == "pivot_l":
            thruster_utils.move_pivot(direction="L")
        elif SFR.motion == "pivot_r":
            thruster_utils.move_pivot(direction="R")
        elif SFR.motion == "stop":
            SFR.stopped = True
            thruster_utils.break_thrusters()

        # NOTE: Should replace all busy waiting with threads
        while not(SFR.callback):
            pass

if __name__ == "__main__":
    rospy.init_node('ControlsNode', anonymous=True)
    rospy.Subscriber('motion_plans', MotionPlans, control_loop)
    rospy.Subscriber('sensors', SensorReadings, callback_sensors)
    done_pub = rospy.Publisher('done', Done, queue_size=10)
    SFR.dPub = done_pub
    controls_pub = rospy.Publisher('motor_signals', Controls, queue_size=10)
    SFR.cPub = controls_pub

    main_loop()

    # Until ros is shutdown, spin and wait for callbacks to be evoked
    rospy.spin()
