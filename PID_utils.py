"""
PID tuning:

1. Set all gains to zero: Start with the PID controller in its simplest form, which is a proportional controller. 
This means setting the integral and derivative gains (I and D) to zero.

2. Tune the Proportional Gain (P): Increase P until the output oscillates. 
Once the output oscillates, reduce the P slightly until the oscillation stops. 
(This is known as the ultimate gain, Ku)

3. Tune the Integral Gain (I): Implement integral action to eliminate the steady state error. 
Start increasing the I gain from zero, until any offset is corrected in sufficient time for the process. 
If the system becomes unstable (starts to oscillate or becomes too sluggish), reduce I.

4. Tune the Derivative Gain (D): Implement derivative action to improve the settling time and overshoot. 
Increase D, if required, until the loop is quick to reach its reference after a load disturbance. 
If the system becomes unstable or too sensitive to noise, reduce D.
"""

import time
import matplotlib.pyplot as plt
from simple_pid import PID

from lib.lidar_driver import LD06
from lib.platform_utils import get_platform
from lib.matplotlib_2D import plot_2D


# Initialize LiDAR
lidar = LD06(port = '/dev/ttyS0',  # Serial port on Raspberry Pi
             speed = 10,
             format = None,
             visualization = plot_2D(plotrange=1500),
             out_len = 38,
             platform=get_platform())


delta_P             = 0.01      # range threshold for stable P
P_tuning_factor     = 0.2       # 20%
dc_limits           = (0.1, 1)  # duty cycle min/max boundaries
speed_history_len   = 50        # number of speed values to keep for stability check
loop_time           = 3         # seconds duration of a tuning cycle

# PID values: find P by setting set I and D to zero
P                   = 0.04
pid                 = PID(P, 0, 0, setpoint=10)
# TODO: after P is found, initialize I and D
I                   = 0.01
D                   = 0.01


# Prepare the plot
plt.ion()  # Enable interactive mode
fig, ax = plt.subplots()
line, = ax.plot([], [], 'r-', label='Speed')  # Initialize the line object with a label
ax.set_xlim(0, loop_time)  # Set x-axis limits to loop_time for faster observation
ax.set_ylim(0, 1)  # Set y-axis limits
ax.set_xlabel('Time (s)')
ax.set_ylabel('Speed (Hz)')
ax.set_title('LiDAR Speed Response')
ax.legend()  # Add a legend

# update the plot of speed over time
def update_plot(t, s):
    line.set_xdata(t)
    line.set_ydata(s)
    ax.relim()
    ax.autoscale_view()
    fig.canvas.draw()
    fig.canvas.flush_events()

# Main loop
start_time = time.time()
time_data = []
speed_data = []
last_speeds = []

try:
    while True:
        current_time = time.time() - start_time
        speed = lidar.speed
        print(f"Speed: {speed:.2f} Hz")
        dc = pid(speed)
        print(f"Duty Cycle: {dc:.2f}")
        
        # update duty cycle safely
        dc = max(min(dc, dc_limits[1]), dc_limits[0])
        
        # simple anti-windup: If actuator is saturated, stop integrating
        if dc >= 1 or dc <= 0.1:
            pid.components[1] = 0  # Reset the integral (I) component
        
        lidar.pwm.change_duty_cycle(dc)

        # Append data for plotting and tracking
        time_data.append(current_time)
        speed_data.append(speed)
        last_speeds.append(speed)
        if len(last_speeds) > speed_history_len:
            last_speeds.pop(0)
        
        update_plot(time_data, speed_data)

        # Check if the system is stable based on the last speed values
        if max(last_speeds) - min(last_speeds) < delta_P:
            # TODO: System is now stable; proceed to tune I and D
            print("P found: ", P)

        # # Print the current P, I, D values
        # print(f'Current PID values: P={P}, I={I}, D={D}')

        # Run for some seconds, then adjust P, I, and D
        if current_time >= loop_time:
            P *= 1 - P_tuning_factor    # decrease P by tuning factor
            pid.tunings = (P, I, D)     # Update PID tunings
            start_time = time.time()    # Reset the timer
            time_data = []              # Clear the time data
            speed_data = []             # Clear the speed data
            last_speeds = []            # Clear the speed history

except KeyboardInterrupt:
    print("Exiting...")
    plt.close(fig)  # Close the plot window

finally:
    # Perform any cleanup here if necessary
    # print("Cleaning up resources...")
    pass
