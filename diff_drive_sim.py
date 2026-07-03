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
        v += np.random.normal(0, 0.1)
        omega += np.random.normal(0, 0.1)
        self.theta += omega*dt
        self.x += v*np.cos(self.theta)*dt
        self.y += v*np.sin(self.theta)*dt


class Controller():
    def __init__(self, kp, kd, ki):
        self.kp = kp
        self.kd = kd
        self.ki = ki
        self.integral_error = 0
        self.last_error = 0

    def control(self, error, dt):
        self.integral_error += error*dt
        output = (error * self.kp) + ((error-self.last_error)/dt * self.kd) + (self.integral_error * self.ki)
        self.last_error = error
        return output


class Estimator():
    def __init__(self, kx, ky):
        self.last_x = 0
        self.last_y = 0
        self.kx = kx
        self.ky = ky

    def smooth(self, x, y):
        new_x = x*self.kx + self.last_x*(1-self.kx)
        new_y = y*self.ky + self.last_y*(1-self.ky)
        self.last_x = new_x
        self.last_y = new_y
        return new_x, new_y


class Simulations():
    def __init__(self, dt, x_target, y_target, linear_PID, angular_PID, robot, noise, estimator=None):
        self.dt = dt
        self.x_target = x_target
        self.y_target = y_target
        self.linear_PID = linear_PID
        self.angular_PID = angular_PID
        self.estimator = estimator
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

            noisy_x = self.robot.x + random.normal(0, self.noise)
            noisy_y = self.robot.y + random.normal(0, self.noise)

            if self.estimator is None:
                estimated_x = noisy_x
                estimated_y = noisy_y
            else:
                estimated_x, estimated_y = self.estimator.smooth(noisy_x, noisy_y)

            linear_error = np.sqrt((self.x_target - estimated_x)**2 + (self.y_target - estimated_y)**2)

            if linear_error < self.threshold:
                linearv = 0
                angularv = 0
                self.robot.update(linearv, angularv, self.dt)
                break

            else:
                angular_error = np.arctan2((self.y_target - estimated_y), (self.x_target - estimated_x)) - self.robot.theta
                angular_error = (angular_error + np.pi) % (2 * np.pi) - np.pi

                linear_v = self.linear_PID.control(linear_error, self.dt)
                linear_v = np.clip(linear_v*np.cos(angular_error), 0, 1)

                angular_v = self.angular_PID.control(angular_error, self.dt)
                angular_v = np.clip(angular_v, -1, 1)

                self.robot.update(linear_v, angular_v, self.dt)

            self.x.append(self.robot.x)
            self.y.append(self.robot.y)

            self.step += 1


class Animator():
    def __init__(self, sim):
        self.sim = sim
        self.fig, (self.ax_xy, self.ax_x, self.ax_y) = plt.subplots(1, 3, figsize=(14, 4))
        self.time = np.arange(len(sim.x)) * sim.dt

    def animate(self):
        self.ax_xy.set_xlim(-1, self.sim.x_target + 2)
        self.ax_xy.set_ylim(-1, self.sim.y_target + 2)
        self.ax_xy.set_title("XY Path")
        self.ax_xy.plot(self.sim.x_target, self.sim.y_target, 'r*', markersize=12, label="Target")
        path_line, = self.ax_xy.plot([], [], 'b-', lw=2)
        robot_dot, = self.ax_xy.plot([], [], 'bo', markersize=8)

        self.ax_x.set_xlim(0, len(self.time))
        self.ax_x.set_ylim(min(self.sim.x) - 1, max(self.sim.x) + 1)
        self.ax_x.set_title("X over Time")
        self.ax_x.axhline(self.sim.x_target, color='r', linestyle='--', label="Target")
        x_line, = self.ax_x.plot([], [], 'b-')

        self.ax_y.set_xlim(0, len(self.time))
        self.ax_y.set_ylim(min(self.sim.y) - 1, max(self.sim.y) + 1)
        self.ax_y.set_title("Y over Time")
        self.ax_y.axhline(self.sim.y_target, color='r', linestyle='--', label="Target")
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
    x_target = 8
    y_target = 12
    dt = 0.1

    robot = Robot()
    linear_cont = Controller(0.8, 0.1, 0)
    angular_cont = Controller(0.8, 0.1, 0)
    estimator = Estimator(0.3, 0.3)
    sim = Simulations(dt, x_target, y_target, linear_cont, angular_cont, robot, noise=0.5, estimator=estimator)
    sim.run()

    ani = Animator(sim)
    ani = ani.animate()
    plt.show()


if __name__ == "__main__":
    main()