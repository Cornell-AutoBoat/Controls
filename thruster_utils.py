import time
import serial
import numpy as np
import src.SFR as SFR
from  test.msg import MCdata
import pid

# ser = serial.Serial('dev/ttyACMO')


def sendValue(valL, valR):
    valL = clamp(valL)
    valR = clamp(valR)
    print(str(valL) + " " + str(valR))
    msg = MCdata()
    msg.sR = valR
    msg.sL = valL
    msg.kill = False
    SFR.mcPub.publish(msg)
    # toSend = 'MC R' + str(valR) + ' L' + str(valL)
    # ser.write(str(toSend).encode())


# def voltage_scale(v):
#     return 5.513/(-0.004*v**2 + 0.502*v - 1.681)


def clamp(s):
    return np.clip(int(s), 1100, 1900)


def transform_to_thrust(v, w):
    """
    Transforms desired linear and angular velocities into signals to the motors

    Args:
        v: linear velocity (m/s)
        w: angular velocity (rad/s)
    Returns:
        sL, sR: signals which were sent to the motors
    """
    return pid.pid_control(v, w)
    # sv = voltage_scale(14.33)*(133.35*v) + 1500
    # sw = voltage_scale(14.33)*(96.164*w**2 + 61.017*w)

    # sL = clamp(sv - sw)
    # sR = clamp(sv + sw)

    # sendValue(sL, sR)
    # return sL, sR


def move_straight(direction="F", v=1.0):
    """
    Moves the boat straight. Must be stopped separately through breaking.

    Args:
        direction: one of "F" or "B" to easily move forwards or backwards at 1m/s
        v: desired linear velocity. Positive to move forward, negative to move backwards
    """
    if direction == "B":
        v = -1.0
    return transform_to_thrust(v, 0)


def move_pivot(direction="R", w=0.5):
    """
    Pivot in place. Must be stopped separately through breaking.

    Args:
        direction: one of "L" or "R" to easily pivot at 1rad/s
        w: desired angular velocity. Positive to move counter-clockwise, 
        negative to move clockwise.
    """
    if direction == "R":
        w = -w
    return transform_to_thrust(0, w)


def kill_system():
    msg = MCdata()
    msg.sL = 1500
    msg.sR = 1500
    msg.kill = True
    SFR.mcPub.publish(msg)
    # ser.write('KILL')


def break_thrusters(curr_sL, curr_sR, break_time=0.5):
    # read current signals being supplied to thrusters
    sL = 3000 - curr_sL
    sR = 3000 - curr_sR
    sendValue(sL, sR)
    time.sleep(break_time)
    # kill motors
    sendValue(1500, 1500)
