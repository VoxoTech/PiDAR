import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt


pwm_measurements = [(0.1, 2.52), (0.12, 3.24), (0.15, 4.24), (0.2, 5.99), (0.3, 9.38), 
                   (0.305, 9.56), (0.31, 9.72), (0.317, 10.0), (0.32, 10.08), 
                   (0.35, 11.16), (0.4, 12.84), (0.45, 14.53), (0.5, 16.28)]

pwm, speed = zip(*pwm_measurements)

# Convert pwm and speed to numpy arrays
pwm = np.array(pwm)
speed = np.array(speed)

# Define the form of the function you want to fit (e.g., a linear function).
def func(pwm, m, b):
    return m * pwm + b

# Use curve_fit to find the optimal parameters.
params, params_covariance = curve_fit(func, pwm, speed)
print("m, b:", params)

# Plot the data points and the fitted curve.
plt.scatter(pwm, speed, label='Data')
plt.plot(pwm, func(pwm, *params), color='red', label='Fitted function')

plt.legend(loc='best')
plt.show()


# ------------------------------------------------
params = [34.33718363, -0.89974737]

def func(pwm, m, b):
    return m * pwm + b

custom_pwm = 0.25
custom_speed = func(custom_pwm, *params)  # Calculate the corresponding speed
print(f"calculated: PWM {custom_pwm} -> speed {custom_speed}")

custom_speed = 10
custom_pwm = (custom_speed - params[1]) / params[0]  # Calculate the corresponding PWM 
print(f"calculated: PWM {custom_speed} -> speed {custom_pwm}")
