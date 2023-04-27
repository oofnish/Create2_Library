import math
import time


class World:
    """
    World is an internal representation of the environment around the robot, intended to allow the bot to query for
    information (eg, known obstacles, cost-to-go, metrics, etc.), and to allow the bot's experiences to update the
    representation with new data
    """

    def __init__(self, robotref, grid_resolution=500, grid_size=101, initial_position=None):
        self.robot = robotref
        # Latest sensor reading, retrieved using GetSensors. This should be considered the "Truth" for sensor data, and
        # should only be refreshed in one place during behaviors
        self.sensors = None

        # Odometry
        # useful metrics
        self.D = 235 #self.robot.WHEEL_BASE  # distance between wheels
        self.max_wheel_velocity = 500  # maximum possible wheel velocity (linear)
        self.grid_resolution = grid_resolution
        self.grid_size = grid_size

        # position / state tracking
        if initial_position is None:
            # default initial position is somewhere in the middle of the occupancy grid.
            initial_position = [grid_resolution * grid_size / 2, grid_resolution * grid_size / 2, 0]
        self.x = 0  # Current x position +x is forward from robot's initial orientation
        self.y = 0  # Current y position
        self.theta = 0  # Current angle w.r.t starting orientation

        # velocity tracking
        self.v_L = 0  # Latest left wheel velocity (linear)
        self.v_R = 0  # Latest Right wheel Velocity (linear)
        self.last_update_time = time.time_ns()  # Latest wheel velocity update. Velocity is constant for the duration

        # World representation;  occupancy grid with landmarks
        self.landmarks = []
        self.occupancy_grid = [[0 for _ in range(grid_size)] for _ in range(grid_size)]

        # statistics
        # total distance traveled
        self.total_dist = 0

    def sense(self, refresh=False):
        """
        Gather latest sensor data if we've never done so, or refresh is true
        """
        if refresh is True or self.sensors is None:
            self.sensors = self.robot.get_sensors()

        return self.sensors

    def update_wheel_velocities(self, v_l, v_r):
        """
        Provide an update to wheel velocities for odometry purposes.
        """
        # get the current time and calculate how much time has passed since the last update
        current_time = time.time_ns()
        dt = current_time - self.last_update_time

        # update odometry
        self.update_position(dt)

        # update occupancy grid
        self.update_free_area(self.x, self.y)

        # save the newly set wheel velocities, making sure the wheel velocities don't exceed max; we should probably
        # throw some kind of error if this occurs as it would probably mean that odometry is borked.
        self.v_L = max(min(v_l, self.max_wheel_velocity), -self.max_wheel_velocity)
        self.v_R = max(min(v_r, self.max_wheel_velocity), -self.max_wheel_velocity)

        # send the actual drive command to the robot. convenient to do this here, though it's a bit ugly from a
        # dependency standpoint
        self.robot.drive_direct(int(v_l), int(v_r))

        # set the last update time for future reference.
        self.last_update_time = current_time

    def update_position(self, dt):
        """
        Update odometry information based on wheel velocities and time.  This is about as accurate as a blind person
        throwing darts on a spinning table, but it's what we have.
        """

        # Angular velocity of robot based on independent wheel velocities.
        omega = (self.v_R - self.v_L) / self.D

        if self.v_L != self.v_R:
            # non-equal velocities create an arc of movement
            r = self.D * (self.v_L + self.v_R) / (2 * (self.v_R - self.v_L))
            d_theta = omega * dt

            self.x += r * (math.sin(self.theta + d_theta) - math.sin(self.theta))
            self.y += r * (math.cos(self.theta) - math.cos(self.theta + d_theta))
            self.theta += d_theta
        else:
            # equal velocities produce a linear-ish position calculation
            # velocity component
            v = (self.v_R + self.v_L) / 2
            self.x += v * dt * math.cos(self.theta)
            self.y += v * dt * math.sin(self.theta)

    def add_landmark(self, x, y, label):
        """
        adds a 3-tuple to the landmarks list, x, y, and a label.  needs a lot more for it to be useful...
        """
        self.landmarks.append((x, y, label))

    def update_free_area(self, x, y):
        """
        Updates the occupancy grid based on a given x,y world position. We assume that if the robot can be at the
        position, the space is unoccupied
        """
        #grid_x = int(x / self.grid_resolution)
        #grid_y = int(y / self.grid_resolution)

        #self.occupancy_grid[grid_x][grid_y] = 1
        pass

    def get_position(self):
        """
        returns the current robot position for printing purposes or whatever
        """
        return self.x, self.y, self.theta

    def get_landmarks(self):
        """
        gets the list of landmark tuples
        """
        return self.landmarks

    def get_occupancy_grid(self):
        """
        gets the occupancy grid (list of lists)
        """
        return self.occupancy_grid
