import time
import serial
import numpy as np
import SFR
from test.msg import Controls
import pid

# ser = serial.Serial('dev/ttyACMO')


def send_value(valL, valR):
    valL = clamp(valL)
    valR = clamp(valR)
    print(str(valL) + " " + str(valR))
    SFR.sL, SFR.sR = valL, valR
    msg = Controls()
    msg.sR = valR
    msg.sL = valL
    SFR.cPub.publish(msg)
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
    # TODO: maybe change this, fix documentation
    if direction == "B":
        send_value(1400, 1400)
    else:
        send_value(1600, 1600)


def move_pivot(direction="R", w=0.5):
    """
    Pivot in place. Must be stopped separately through breaking.

    Args:
        direction: one of "L" or "R" to easily pivot at 1rad/s
        w: desired angular velocity. Positive to move counter-clockwise, 
        negative to move clockwise.
    """
    # TODO: maybe change this, fix documentation
    if direction == "R":
        send_value(1550, 1450)
    else:
        send_value(1450, 1550)


# def kill_system():
#     msg = MCdata()
#     msg.sL = 1500
#     msg.sR = 1500
#     msg.kill = True
#     SFR.mcPub.publish(msg)
    # ser.write('KILL')


def break_thrusters(break_time=0.5):
    # TODO: test this
    # read current signals being supplied to thrusters
    sL, sR = 3000 - SFR.sL, 3000 - SFR.sR
    send_value(sL, sR)
    time.sleep(break_time)
    # kill motors
    send_value(1500, 1500)


def tune_pid_on_heading():
    print("Tuning pid")
    desired_heading = SFR.heading
    data = []
    offsets = [0, 50, 0, -50, 0, -100, 0, 100, 0, -150, 0, 150, 0]
    initial_time = time.time()
    for offset in offsets:
        send_value(1500 + offset, 1500 - offset)
        step_time = time.time()
        while time.time() - step_time < 2:
            cur_heading = SFR.heading
            data.append(f"{time.time() - initial_time}, {offset}, {cur_heading - desired_heading}")
            time.sleep(0.2)
    write_to_text(data)
    print("Finished tuning pid")


def write_to_text(data):
    with open('data.txt', 'w') as f:
        for line in data:
            f.write(line)
            f.write('\n')
