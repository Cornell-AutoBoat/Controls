from simple_pid import PID
import thruster_utils
import time
import src.SFR as SFR
import csv
import numpy as np


def clamp(x):
    return np.clip(int(x), 1100, 1900)


def pid_test():
    with open('pid_data_angular', 'w') as f:
        writer = csv.writer(f)
        header = ['Time', 'Input', 'Outputy']
        writer.writerow(header)
        start1 = time.time()
        for i in [0, -50, 0, 50, 0, -100, 0, 100, 0, -150, 0, 150, 0, -200, 0, 200]:
            start2 = time.time()
            prev_w_output = 0
            # While sufficient error or too much time passed, recalculate motor signals
            v_output = 0
            w_output = i
            sL = clamp(v_output - (w_output + prev_w_output) + 1500)
            sR = clamp(v_output + (w_output + prev_w_output) + 1500)
            thruster_utils.sendValue(sL, sR)
            while time.time() - start2 < 3.0:
                writer.writerow([str(time.time() - start1), str(w_output), str(SFR.ang_vx), str(SFR.ang_vy), str(SFR.ang_vz)])
                print(str(time.time() - start1), str(w_output), str(SFR.ang_vx), str(SFR.ang_vy), str(SFR.ang_vz))
                time.sleep(0.1)
    SFR.done = True


def pid_control(v, w): 
    print("starting pid")
    with open('pid_data_angular', 'w') as f:
        writer = csv.writer(f)
        header = ['Time', 'Input', 'Outputy']
        writer.writerow(header)
        start = time.time()
        # Initialize controllers with Kp, Ki, Kd, desired linear/angular velocity
        # pid_v = PID(133, 0.1, 0.05, setpoint=v)
        pid_w = PID(70.0, 0.0,  13.0, setpoint=0)

        # Min and max motor signals
        # pid_v.output_limits = (-400, 400)
        pid_w.output_limits = (-400, 400)

        # Update every 0.01 seconds
        # pid_v.sample_time = 0.01
        pid_w.sample_time = 0.01

        # Calculate errors (desired - actual)
        # v_error = SFR.lin_v - v
        w_error = SFR.ang_vy - w

        prev_w_output = 0

        start1 = time.time()

        # While sufficient error or too much time passed, recalculate motor signals
        while np.abs(w_error) > np.pi/4 or time.time() - start < 1.0:
            # v_output = pid_v(v_error)
            v_output = 0
            if v > 0:
                v_output = 100
            elif v < 0:
                v_output = -100
            w_output = pid_w(w_error)
            print("w out: " + str(w_output))

            writer.writerow([str(time.time() - start1), str(w_output), str(SFR.ang_vy)])

            # send signals to motors
            sL = clamp(v_output - (w_output + prev_w_output) + 1500)
            sR = clamp(v_output + (w_output + prev_w_output) + 1500)
            thruster_utils.sendValue(sL, sR)

            # recalculate error
            v_error = SFR.lin_v - v
            w_error = SFR.ang_vy - w

            prev_w_output = w_output + prev_w_output
            time.sleep(0.1)

    return sL, sR
