import matplotlib.pyplot as plt
import numpy as np

iterations = 500
dt = 0.1
timesteps = int(iterations/dt)

x = np.zeros(timesteps)
time = np.arange(0, iterations, dt)

setpoint = 1000
kp = 0.5
kd = 1
ki = 0.01
m = 1
a = 0
v = 0
I = 0

for i in range(timesteps-1):

    x[i+1] = x[i] + v * dt
    error = setpoint - x[i+1]
    force = (error * kp) - (v * kd) + (I * ki) - 10
    force = max(min(force, 10), -10)
    I += error * dt if force < 10 and force > -10 else 0
    a = force/m
    v += a * dt

plt.plot(time, x)
plt.xlabel('Time')
plt.ylabel('Position')
plt.show()
print(x[-1])