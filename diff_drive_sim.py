import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import numpy.random as random

random_seed = 42
np.random.seed(random_seed)


class Robot():
    def __init__(self):
        self.x = 0
        self.y = 0
        self.theta = 0

    def update(self, v, omega, dt):
        self.theta += omega*dt
        self.x += v*np.cos(self.theta)*dt
        self.y += v*np.sin(self.theta)*dt


class Controller():
    def __init__(self, kp, kd, ki):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.I = 0
        self.lasterror = 0

    def control(self, error, dt):
        self.I += error*dt
        output = (error * self.kp) + ((error-self.lasterror)/dt * self.kd) + (self.I * self.ki)
        self.lasterror = error
        return output


class Simulations():
    def __init__(self, dt, xtarget, ytarget, linearPID, angularPID, robot, noise):
        self.dt = dt
        self.xtarget = xtarget
        self.ytarget = ytarget
        self.linearPID = linearPID
        self.angularPID = angularPID
        self.robot = robot
        self.x = []
        self.y = []
        self.threshold = 0.1
        self.step = 0
        self.max_steps = 1000
        self.noise = noise

    def run(self):

        while True:
            if self.step >= self.max_steps:
                break

            noisyx = self.robot.x + random.normal(0, self.noise)
            noisyy = self.robot.y + random.normal(0, self.noise)

            linear_error = np.sqrt((self.xtarget - noisyx)**2 + (self.ytarget - noisyy)**2)
            if linear_error < self.threshold:
                break
            angular_error = np.arctan2((self.ytarget - noisyy), (self.xtarget - noisyx)) - self.robot.theta
            angular_error = (angular_error + np.pi) % (2 * np.pi) - np.pi

            linearv = self.linearPID.control(linear_error, self.dt)
            linearv = np.clip(linearv*np.cos(angular_error), 0, 1)

            angularv = self.angularPID.control(angular_error, self.dt)
            angularv = np.clip(angularv, -1, 1)

            self.robot.update(linearv, angularv, self.dt)

            self.x.append(self.robot.x)
            self.y.append(self.robot.y)

            self.step += 1


class Animator():
    def __init__(self, sim):
        self.sim = sim
        self.fig, (self.ax_xy, self.ax_x, self.ax_y) = plt.subplots(1, 3, figsize=(14, 4))
        self.time = np.arange(len(sim.x)) * sim.dt

    def animate(self):
        self.ax_xy.set_xlim(-1, self.sim.xtarget + 2)
        self.ax_xy.set_ylim(-1, self.sim.ytarget + 2)
        self.ax_xy.set_title("XY Path")
        self.ax_xy.plot(self.sim.xtarget, self.sim.ytarget, 'r*', markersize=12, label="Target")
        path_line, = self.ax_xy.plot([], [], 'b-', lw=2)
        robot_dot, = self.ax_xy.plot([], [], 'bo', markersize=8)

        self.ax_x.set_xlim(0, len(self.time))
        self.ax_x.set_ylim(min(self.sim.x) - 1, max(self.sim.x) + 1)
        self.ax_x.set_title("X over Time")
        self.ax_x.axhline(self.sim.xtarget, color='r', linestyle='--', label="Target")
        x_line, = self.ax_x.plot([], [], 'b-')

        self.ax_y.set_xlim(0, len(self.time))
        self.ax_y.set_ylim(min(self.sim.y) - 1, max(self.sim.y) + 1)
        self.ax_y.set_title("Y over Time")
        self.ax_y.axhline(self.sim.ytarget, color='r', linestyle='--', label="Target")
        y_line, = self.ax_y.plot([], [], 'g-')

        def update(i):
            path_line.set_data(self.sim.x[:i], self.sim.y[:i])
            robot_dot.set_data([self.sim.x[i]], [self.sim.y[i]])
            x_line.set_data(self.time[:i], self.sim.x[:i])
            y_line.set_data(self.time[:i], self.sim.y[:i])
            return path_line, robot_dot, x_line, y_line

        plt.tight_layout()
        return animation.FuncAnimation(self.fig, update, frames=len(self.time), interval=20, blit=True, repeat=False)


def main():
    xtarget = 10
    ytarget = 10
    dt = 0.1

    robot1 = Robot()
    linearcont1 = Controller(0.8, 0.1, 0.0)
    angularcont1 = Controller(0.8, 0.1, 0.0)
    sim = Simulations(dt, xtarget, ytarget, linearcont1, angularcont1, robot1, noise=0.6)
    sim.run()

    animator = Animator(sim)
    ani = animator.animate()
    plt.show()


if __name__ == "__main__":
    main()