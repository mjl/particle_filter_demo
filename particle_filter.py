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

from draw import Maze

maze_data = ( ( 1, 0, 1, 0, 0 ),
              ( 0, 0, 0, 0, 1 ),
              ( 1, 1, 1, 0, 0 ),
              ( 1, 0, 0, 0, 0 ),
              ( 0, 0, 1, 0, 1 ))

maze_data = ( ( 1, 1, 0, 0, 1, 1, 0, 0, 0, 0 ),
              ( 1, 1, 0, 1, 1, 1, 0, 0, 0, 0 ),
              ( 0, 0, 0, 0, 0, 0, 0, 1, 0, 1 ),
              ( 0, 0, 0, 0, 1, 0, 0, 1, 1, 1 ),
              ( 1, 1, 1, 1, 1, 1, 0, 0, 0, 0 ),
              ( 1, 1, 1, 0, 1, 1, 1, 0, 0, 0 ),
              ( 1, 1, 0, 0, 0, 0, 0, 0, 0, 0 ),
              ( 1, 1, 0, 1, 1, 0, 0, 1, 0, 0 ),
              ( 0, 0, 0, 0, 1, 1, 0, 0, 1, 0 ),
              ( 0, 0, 0, 0, 1, 1, 0, 0, 1, 0 ))

PARTICLE_COUNT = 500    # Total number of particles
N = 50                  # Number of particles shuffled per iteration

# ------------------------------------------------------------------------
# Some utility functions

def add_noise(level, *coords):
    return [x + random.uniform(-level, level) for x in coords]

def add_little_noise(*coords):
    return add_noise(0.02, *coords)

def add_some_noise(*coords):
    return add_noise(0.1, *coords)

def weightedPick(particles):
    nu = sum(p.w for p in particles)
    r = random.uniform(0, nu)
    s = 0.0
    for p in particles:
        s += p.w
        if r < s:
            return p
    return p

# ------------------------------------------------------------------------
class Particle(object):
    def __init__(self, x, y, heading=270, w=1):
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
        Find distance to nearest corner.
        """
        return maze.distance_to_nearest_corner(*self.xy)

# ------------------------------------------------------------------------
class Robot(Particle):
    def __init__(self, maze):
        super(Robot, self).__init__(*maze.random_free_place(), heading=90)
        self.chose_random_direction()

    def chose_random_direction(self):
        self.dx, self.dy = add_noise(0.1, 0, 0)

    def read_sensor(self, maze):
        """
        Poor robot, it's sensors are noisy and pretty strange,
        it only can measure the distance to the nearest corner(!)
        and is not very accurate at that too!
        """
        return add_little_noise(super(Robot, self).read_sensor(maze))[0]

    def move(self, maze):
        """
        Move the robot. Note that the movement is stochastic too.
        """
        while True:
            xx, yy = add_noise(0.02, self.x + self.dx, self.y + self.dy)
            if maze.is_free(xx, yy):
                self.x, self.y = xx, yy
                break
            # Bumped into something, chose random new direction
            self.chose_random_direction()

# ------------------------------------------------------------------------

m = Maze(maze_data)
m.draw()

particles = Particle.create_random(PARTICLE_COUNT, m)
robbie = Robot(m)

while True:
    # Read robbie's sensor
    r_d = robbie.read_sensor(m)

    # Update particle weight according to how good every particle matches
    # robbie's sensor reading
    for p in particles:
        p_d = p.read_sensor(m)
        # This is just a gaussian I pulled out of my hat, near to
        # robbie's measurement => 1, further away => 0
        g = math.e ** -((r_d - p_d)**2 * 19)
        p.w = g

    # Time for some action!
    m.show_particles(particles)
    m.show_robot(robbie)

    # I'm doing the movement first, it makes life easier

    robbie.move(m)

    # Move particles according to my belief of movement (this may
    # be different than the real movement, but it's all I got)
    for p in particles:
        p.x += robbie.dx
        p.y += robbie.dy

    # ---------- Shuffle particles ----------

    # Remove all particles that cannot be right (out of arena, inside
    # obstacle). Remember how many were removed so we can add that
    # number in the picking phase below
    particles = [p for p in particles if m.is_free(*p.xy)]
    delta_p = PARTICLE_COUNT - len(particles)

    # Pick N particles according to weight, duplicate them and add
    # some noise to their position
    new_particles = []
    for cnt in range(0, N + delta_p):
        p = weightedPick(particles)
        new_particles.append(Particle(p.x, p.y))

    # Add some random noise
    for p in new_particles:
        while True:
            xx, yy = add_some_noise(p.x, p.y)
            if m.is_free(xx, yy):
                p.x = xx
                p.y = yy
                break

    # remove N old particles, kill off those with least weight
    particles = sorted(particles, key=lambda p: p.w)[N:]
    particles += new_particles
