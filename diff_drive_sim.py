import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np


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
    def __init__(self, dt, xtarget, ytarget, linearPID, angularPID, robot):
        self.dt = dt
        self.xtarget = xtarget
        self.ytarget = ytarget
        self.linearPID = linearPID
        self.angularPID = angularPID
        self.robot = robot
        self.x = []
        self.y = []
        self.threshold = 0.1

    def run(self):

        while True:
            linear_error = np.sqrt((self.xtarget - self.robot.x)**2 + (self.ytarget - self.robot.y)**2)
            if linear_error < self.threshold:
                break
            angular_error = np.arctan2((self.ytarget - self.robot.y), (self.xtarget - self.robot.x)) - self.robot.theta
            angular_error = (angular_error + np.pi) % (2 * np.pi) - np.pi

            linearv = self.linearPID.control(linear_error, self.dt)
            linearv = np.clip(linearv*np.cos(angular_error), 0, 1)

            angularv = self.angularPID.control(angular_error, self.dt)
            angularv = np.clip(angularv, -1, 1)

            self.robot.update(linearv, angularv, self.dt)

            self.x.append(self.robot.x)
            self.y.append(self.robot.y)


def main():
    xtarget = 10
    ytarget = 10
    dt = 0.1

    robot = Robot()
    linearcont = Controller(0.8, 0.1, 0.0)
    angularcont = Controller(0.8, 0.1, 0.0)
    sim = Simulations(dt, xtarget, ytarget, linearcont, angularcont, robot)
    sim.run()
    time = np.arange(0, len(sim.x)*dt, dt)

    fig, (ax_xy, ax_x, ax_y) = plt.subplots(1, 3, figsize=(14, 4))

    ax_xy.set_xlim(-1, xtarget + 2)
    ax_xy.set_ylim(-1, ytarget + 2)
    ax_xy.set_title("XY Path")
    ax_xy.plot(xtarget, ytarget, 'r*', markersize=12, label="Target")
    path_line, = ax_xy.plot([], [], 'b-', lw=1)
    robot_dot, = ax_xy.plot([], [], 'bo', markersize=8)

    ax_x.set_xlim(0, len(time))
    ax_x.set_ylim(min(sim.x) - 1, max(sim.x) + 1)
    ax_x.set_title("X over Time")
    ax_x.axhline(xtarget, color='r', linestyle='--', label="Target")
    x_line, = ax_x.plot([], [], 'b-')

    ax_y.set_xlim(0, len(time))
    ax_y.set_ylim(min(sim.y) - 1, max(sim.y) + 1)
    ax_y.set_title("Y over Time")
    ax_y.axhline(ytarget, color='r', linestyle='--', label="Target")
    y_line, = ax_y.plot([], [], 'g-')

    def update(i):
        path_line.set_data(sim.x[:i], sim.y[:i])
        robot_dot.set_data([sim.x[i]], [sim.y[i]])
        x_line.set_data(time[:i], sim.x[:i])
        y_line.set_data(time[:i], sim.y[:i])
        return path_line, robot_dot, x_line, y_line

    ani = animation.FuncAnimation(fig, update, frames=len(time), interval=20, blit=True)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()