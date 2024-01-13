from robot import ROBOT, Graphics, Ultrasonic
import pygame
import math

MAP_DIMENSIONS = (600, 1200)

# Enviornment Graphics
gfx = Graphics(MAP_DIMENSIONS, 'robot.png', 'map.png')
 
# Robot
start = (200, 200)
robot = ROBOT(start, 0.01*3779.52)

# Sensor
sensor_range = 250, math.radians(40)
ultrasonic = Ultrasonic(sensor_range, gfx.map)

dt = 0
last_time = pygame.time.get_ticks()

# Simulation
running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    dt = (pygame.time.get_ticks()-last_time)/1000

    gfx.map.blit(gfx.map_img, (0, 0))

    robot.kinematics(dt)
    gfx.draw_robot(robot.x, robot.y, robot.heading)
    point_cloud = ultrasonic.sense_obstacles(robot.x, robot.y ,robot.heading)
    robot.avoid_obstacles(point_cloud, dt)  
    gfx.draw_sensor_data(point_cloud)

    pygame.display.update()         