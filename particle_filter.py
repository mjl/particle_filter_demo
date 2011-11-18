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

maze_data = ( ( 2, 0, 1, 0, 0 ),
              ( 0, 0, 0, 0, 1 ),
              ( 1, 1, 1, 0, 0 ),
              ( 1, 0, 0, 0, 0 ),
              ( 0, 0, 2, 0, 1 ))

maze_data = ( ( 1, 1, 0, 0, 1, 2, 0, 0, 0, 0 ),
              ( 1, 2, 0, 1, 1, 1, 0, 0, 0, 0 ),
              ( 0, 1, 0, 0, 0, 0, 0, 1, 0, 1 ),
              ( 0, 0, 0, 0, 1, 0, 0, 1, 1, 2 ),
              ( 1, 1, 1, 1, 1, 2, 0, 0, 0, 0 ),
              ( 1, 1, 1, 0, 1, 1, 1, 0, 0, 0 ),
              ( 2, 0, 0, 0, 0, 0, 0, 0, 0, 0 ),
              ( 1, 2, 0, 1, 1, 0, 0, 2, 0, 0 ),
              ( 0, 0, 0, 0, 1, 1, 0, 0, 1, 0 ),
              ( 0, 0, 0, 0, 2, 1, 0, 0, 1, 0 ))

PARTICLE_COUNT = 1500    # Total number of particles

# ------------------------------------------------------------------------
# Some utility functions

def add_noise(level, *coords):
    return [x + random.uniform(-level, level) for x in coords]

def add_little_noise(*coords):
    return add_noise(0.02, *coords)

def add_some_noise(*coords):
    return add_noise(0.1, *coords)

def weightedPick(particles):
    r = random.uniform(0, 1)
    s = 0.0
    for p in particles:
        if p.w > 0:
            s += p.w
            if r < s:
                return p
    return p

StdDev = 1.4

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
    def __init__(self, maze):
        super(Robot, self).__init__(*maze.random_free_place(), heading=90)
        self.chose_random_direction()
        self.step_count = 0

    def chose_random_direction(self):
        self.dx, self.dy = add_noise(0.1, 0, 0)

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
            if maze.is_free(xx, yy) and self.step_count % 30 != 0:
                self.x, self.y = xx, yy
                break
            # Bumped into something or too long in same direction,
            # chose random new direction
            self.chose_random_direction()

# ------------------------------------------------------------------------

world = Maze(maze_data)
world.draw()

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

    # ---------- Show current state ----------
    world.show_particles(particles)
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
        if p is None:
            p = Particle(*world.random_place(), noisy=True)
        new_particles.append(Particle(p.x, p.y, noisy=True))

    particles = new_particles

    # ---------- Move things ----------
    robbie.move(world)

    # Move particles according to my belief of movement (this may
    # be different than the real movement, but it's all I got)
    for p in particles:
        p.x += robbie.dx
        p.y += robbie.dy
