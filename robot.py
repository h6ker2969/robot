import numpy as np
import math
import pygame

# Calculate Euclidean distance from two points in the enviornment
def distance(p1, p2):
    p1 = np.array(p1)
    p2 = np.array(p2)

    return np.linalg.norm(p1-p2)

# Robot
class ROBOT:
    def __init__(self, startpos, width):
        # From meters to pixels
        self.m2p = 3779.52

        # Robot dims
        self.x = startpos[0]
        self.y = startpos[1]
        self.heading = 0

        # Meters/s
        self.vl = 0.01*self.m2p
        self.vr = 0.01*self.m2p
    
        # Velocities
        self.maxspeed = 0.02*self.m2p   
        self.minspeed = 0.01*self.m2p

        # Minimum distance from the obstacles
        self.min_obs_dist = 100

        # Countdown (seconds)
        self.countdown = 5

    def avoid_obstacles(self, point_cloud, dt):
        closest_obs = None
        dist = np.inf 

        if len(point_cloud) > 1:   
            for point in point_cloud:
                if dist > distance([self.x, self.y], point):
                    dist = distance([self.x, self.y], point)
                    closest_obs = (point, dist)

        if  closest_obs[1] < self.min_obs_dist and self.countdown > 0:
            self.countdown -= dt
            self.move_backward()
        else:
            self.countdown = 5
            self.move_forward()

    def move_backward(self):
        self.vr = - self.minspeed
        self.vl = - self.minspeed/2

    def move_forward(self):
        self.vr = self.minspeed
        self.vl = self.minspeed

    def kinematics(self, dt):
        self.x += ((self.vl+self.vr)/2)*math.cos(self.heading)*dt
        self.y += ((self.vl+self.vr)/2)*math.sin(self.heading)*dt
        self.heading += (self.vr-self.vl)/self.w*dt

        if self.heading>2*math.pi or self.heading<-2*math.pi:
                self.heading = 0

        self.vr = max(min(self.maxspeed, self.vr), self.minspeed)
        self.vl = max(min(self.maxspeed, self.vl), self.minspeed)

# Graphics
class Graphics:
    def __init__(self, dimensions, robot_img_path, map_img_path):
        pygame.init()
        # RGB Colors
        from pallete import BLACK, WHITE, GREEN, BLUE, RED, YELLOW
        self.black = BLACK
        self.white = WHITE
        self.green = GREEN
        self.blue = BLUE
        self.red = RED
        self.yel = YELLOW

        # Map

        # Load images
        self.robot = pygame.image.load(robot_img_path)
        self.map = pygame.image.load(map_img_path)

        # Dimensions
        self.height, self.width = dimensions

        # Window
        pygame.display.set_caption("Obsatcle Avoidance Program")
        self.map = pygame.display.set_mode((self.width, self.height))
        self.map_blit(self.map_img, (0, 0))

    def draw_robot(self, x, y, heading):
        rotated = pygame.transform.rotozoom(self.robot, math.degrees(heading), 1)
        rect = rotated.get_rect(center = (x, y)) 
        self.map_blit(rotated, rect)

    def draw_sensor_data(self, point_cloud):
        for point in point_cloud:
            pygame.draw.circle(self.map, self.red, point, 3, 0)

# Ultrasonic Sensor
class Ultrasonic:
    def __init__(self,  sensor_range, map):
        self.sensor_range = sensor_range
        self.map_width, self.map_height =   pygame.display.get_surface().get_size()
        self.map = map

    def sense_obstacles(self, x, y, heading):
        obstacles = []
        x1, y1 = x, y
        start_angle = heading + self.sensor_range[1]
        finish_angle = heading - self.sensor_range[1]
        for angle in np.linespace(start_angle, finish_angle, 10, False):
            x2 = x1 + self.sensor_range[0] * math.cos(angle)
            y2 = self.sensor.range[0] * math.sin(angle)
            for i in range(0, 100):
                u = i/100
                x = int(x2*u+x1*(1-u))
                y = int(y2*u+y1*(1-u))
                if 0 < x < self.map_width and   0 < y < self.map_height: 
                    color = self.map.get_at((x, y))
                    self.map.set_at((x, y), (0, 208, 255))
                    if (color[0], color[1], color[2]) == (0, 0, 0):
                        obstacles.append([x, y])
                        break

        return obstacles            