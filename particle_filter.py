# ------------------------------------------------------------------------
# coding=utf-8
# ------------------------------------------------------------------------
#
#  Created by Martin J. Laubach on 2011-11-15
#
# ------------------------------------------------------------------------

from __future__ import absolute_import

import random
import math
import bisect

from draw import Maze

"""
# Smaller maze

maze_data = ( ( 2, 0, 1, 0, 0 ),
              ( 0, 0, 0, 0, 1 ),
              ( 1, 1, 1, 0, 0 ),
              ( 1, 0, 0, 0, 0 ),
              ( 0, 0, 2, 0, 1 ))
"""

# 0 - empty square
# 1 - occupied square
# 2 - occupied square with a beacon at each corner, detectable by the robot

maze_data = ( ( 1, 1, 0, 0, 1, 2, 0, 0, 0, 0 ),
              ( 1, 2, 0, 0, 1, 1, 0, 0, 0, 0 ),
              ( 0, 1, 1, 0, 0, 0, 0, 1, 0, 1 ),
              ( 0, 0, 0, 0, 1, 0, 0, 1, 1, 2 ),
              ( 1, 1, 0, 1, 1, 2, 0, 0, 0, 0 ),
              ( 1, 1, 1, 0, 1, 1, 1, 0, 2, 0 ),
              ( 2, 0, 0, 0, 0, 0, 0, 0, 0, 0 ),
              ( 1, 2, 0, 1, 1, 0, 1, 0, 0, 0 ),
              ( 0, 0, 0, 0, 1, 1, 0, 0, 1, 0 ),
              ( 0, 0, 1, 0, 2, 1, 0, 0, 1, 0 ))

PARTICLE_COUNT = 1500    # Total number of particles

# ------------------------------------------------------------------------
# Some utility functions

def add_noise(level, *coords):
    return [x + random.uniform(-level, level) for x in coords]

def add_little_noise(*coords):
    return add_noise(0.02, *coords)

def add_some_noise(*coords):
    return add_noise(0.1, *coords)

StdDev = 1.3

def w_gauss(a, b):
    # This is just a gaussian I pulled out of my hat, near to
    # robbie's measurement => 1, further away => 0
    error = a - b
    g = math.e ** -(error ** 2 / (2 * StdDev ** 2))
    return g

# ------------------------------------------------------------------------
class WeightedDistribution(object):
    def __init__(self, state):
        accum = 0.0
        self.state = state
        self.distribution = []
        for x in state:
            accum += x.w
            self.distribution.append(accum)

    def pick(self):
        return self.state[bisect.bisect_left(self.distribution, random.uniform(0, 1))]

# ------------------------------------------------------------------------
class Particle(object):
    def __init__(self, x, y, heading=270, w=1, noisy=False):
        if noisy:
            x, y = add_some_noise(x, y)
        self.x = x
        self.y = y
        self.h = heading
        self.w = w

    def __repr__(self):
        return "(%f, %f, w=%f)" % (self.x, self.y, self.w)

    @property
    def xy(self):
        return self.x, self.y

    @property
    def xyh(self):
        return self.x, self.y, self.h

    @classmethod
    def create_random(cls, count, maze):
        return [cls(*maze.random_free_place(), w=1.0 / count) for _ in range(0, count)]

    def read_sensor(self, maze):
        """
        Find distance to nearest beacon.
        """
        return maze.distance_to_nearest_beacon(*self.xy)

# ------------------------------------------------------------------------
class Robot(Particle):
    speed = 0.2

    def __init__(self, maze):
        super(Robot, self).__init__(*maze.random_free_place(), heading=90)
        self.chose_random_direction()
        self.step_count = 0

    def chose_random_direction(self):
        heading = random.uniform(0, 360)
        dx = math.sin(math.radians(heading)) * self.speed
        dy = math.cos(math.radians(heading)) * self.speed
        self.dx, self.dy = dx, dy

    def read_sensor(self, maze):
        """
        Poor robot, it's sensors are noisy and pretty strange,
        it only can measure the distance to the nearest beacon(!)
        and is not very accurate at that too!
        """
        return add_little_noise(super(Robot, self).read_sensor(maze))[0]

    def move(self, maze):
        """
        Move the robot. Note that the movement is stochastic too.
        """
        while True:
            self.step_count += 1
            xx, yy = add_noise(0.02, self.x + self.dx, self.y + self.dy)
            if maze.is_free(xx, yy) and self.step_count % 70 != 0:
                self.x, self.y = xx, yy
                break
            # Bumped into something or too long in same direction,
            # chose random new direction
            self.chose_random_direction()

# ------------------------------------------------------------------------

world = Maze(maze_data)
world.draw()

# initial distribution assigns each particle an equal probability
particles = Particle.create_random(PARTICLE_COUNT, world)
robbie = Robot(world)

while True:
    # Read robbie's sensor
    r_d = robbie.read_sensor(world)

    # Update particle weight according to how good every particle matches
    # robbie's sensor reading
    for p in particles:
        if world.is_free(*p.xy):
            p_d = p.read_sensor(world)
            p.w = w_gauss(r_d, p_d)
        else:
            p.w = 0

    # ---------- Try to find current best estimate for display ----------
    # (not part of the core algorithm)
    # We just compute the mean for all particles that have a reasonably
    # good weight.
    m_x, m_y, m_count = 0, 0, 0
    for p in particles:
        if p.w > 0.6:
            m_count += 1
            m_x += p.x
            m_y += p.y

    if m_count > PARTICLE_COUNT / 10:
        m_x /= m_count
        m_y /= m_count

        # Now compute how good that mean is -- check how many particles
        # actually are in the immediate vicinity
        m_count = 0
        for p in particles:
            if world.distance(p.x, p.y, m_x, m_y) < 1:
                m_count += 1
    else:
        m_x = None

    # ---------- Show current state ----------
    world.show_particles(particles)

    if m_x is not None:
        world.show_mean(m_x, m_y, m_count > PARTICLE_COUNT * 0.8)

    world.show_robot(robbie)

    # ---------- Shuffle particles ----------
    new_particles = []

    # Normalise weights
    nu = sum(p.w for p in particles)
    if nu:
        for p in particles:
            p.w = p.w / nu

    # create a weighted distribution, for fast picking
    dist = WeightedDistribution(particles)

    for _ in particles:
        p = dist.pick()
        new_particle = Particle(p.x, p.y, noisy=True)
        new_particles.append(new_particle)

    particles = new_particles

    # ---------- Move things ----------
    robbie.move(world)

    # Move particles according to my belief of movement (this may
    # be different than the real movement, but it's all I got)
    for p in particles:
        p.x += robbie.dx
        p.y += robbie.dy
