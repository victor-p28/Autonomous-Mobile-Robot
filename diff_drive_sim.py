import matplotlib.pyplot as plt
import numpy as np

iterations = 1000
dt = 0.1
timeSteps = int(iterations/dt)

x = np.zeros(timeSteps)
time = np.arange(0, iterations, dt)

force = 10
m = 1
a = 0
v = 0

for i in range(timeSteps-1):
    x[i+1] = x[i] + v * dt
    a = force/m
    v += a * dt

plt.plot(time, x)
plt.xlabel('Time')
plt.ylabel('Position')
plt.show()