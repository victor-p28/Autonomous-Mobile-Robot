import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


class Controller():
    def __init__(self, kp, kd, ki):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.I = 0

    def control(self, v, error):
        force = (error * self.kp) - (v * self.kd) + (self.I * self.ki) - 5
        return force


class PointMass():
    def __init__(self, mass):
        self.mass = mass
        self.position = 0
        self.velocity = 0

    def massUpdate(self, force, dt):
        self.position += self.velocity * dt
        acceleration = force / self.mass
        self.velocity += acceleration * dt


class Simulation():
    def __init__(self, iterations, dt, setpoint, pid, mass):
        self.iterations = iterations
        self.dt = dt
        self.setpoint = setpoint
        self.timesteps = int(iterations / dt)
        self.x = np.zeros(self.timesteps)
        self.time = np.arange(0, iterations, dt)
        self.pid = pid
        self.mass = mass

    def run(self):
        for i in range(self.timesteps-1):
            error = self.setpoint - self.mass.position
            force = self.pid.control(self.mass.velocity, error)
            force = np.clip(force, -10, 10)
            self.pid.I += error * self.dt if abs(force) < 10 else 0
            self.mass.massUpdate(force, self.dt)
            self.x[i+1] = self.mass.position


def main():
    iterations = 500
    dt = 0.1

    setpoint = 100
    kp = 0.5
    kd = 0.5
    ki = 0.1
    m = 1

    pid = Controller(kp, kd, ki)
    mass = PointMass(m)
    sim = Simulation(iterations, dt, setpoint, pid, mass)
    sim.run()

    fig, ax = plt.subplots()
    line, = ax.plot([], [])
    ax.set_xlim(sim.time[0], sim.time[-1])
    ax.set_ylim(min(sim.x) - 10, max(sim.x) + 10)
    ax.set_xlabel('Time')
    ax.set_ylabel('Position')

    def update(frame):
        line.set_data(sim.time[:frame], sim.x[:frame])
        return line,

    ani = animation.FuncAnimation(fig, update, frames=len(sim.time), interval=1, blit=True)
    plt.show()
    print(sim.x[-1])


if __name__ == "__main__":
    main()