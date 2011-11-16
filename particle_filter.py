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

PARTICLE_COUNT = 2000    # Total number of particles
N = 500                  # Number of particles shuffled per iteration

# Algorithm variants:
# 1 is the particle filter that just relies on particle weights
# to distribute them. It sometimes seems to paint itself into a
# corner when its hypothesis is wrong -- all particles get drawn
# to that wrong location and there aren't any left around the real
# position, so there is little chance of ever correcting that.
# One needs a large amount of particles to counter that (try 2000+).
# 2 is a variant that kills off impossible particles and replaces
# them with fresh, randomly distributed particles. This seems to
# be much more resilient because it re-seeds the hypothesis space
# every now and then. It works better with less particles (400 is
# fine).
ALGO = 1

# ------------------------------------------------------------------------
# Some utility functions

def add_noise(level, *coords):
    return [x + random.uniform(-level, level) for x in coords]

def add_little_noise(*coords):
    return add_noise(0.02, *coords)

def add_some_noise(*coords):
    return add_noise(0.1, *coords)

def weightedPick(particles, nu):
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
        if m.is_free(*p.xy):
            p_d = p.read_sensor(m)
            # This is just a gaussian I pulled out of my hat, near to
            # robbie's measurement => 1, further away => 0
            g = math.e ** -((r_d - p_d)**2 * 7)
            p.w = g
        else:
            p.w = 0

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

    new_particles = []

    # Remove all particles that cannot be right (out of arena, inside
    # obstacle), then add random new ones so the overall count fits
    if ALGO == 2:
        particles = [p for p in particles if p.w > 0]
        new_particles += Particle.create_random(PARTICLE_COUNT - len(particles), m)

    # Pick N particles according to weight, duplicate them and add
    # some noise to their position
    nu = sum(p.w for p in particles)
    for cnt in range(0, N):
        p = weightedPick(particles, nu)
        new_particles.append(Particle(p.x, p.y))

    # Add some random noise
    for p in new_particles:
        p.x, p.y = add_some_noise(p.x, p.y)

    # Ensure we have PARTICLE_COUNT particles, kill off those with
    # the least weight first
    particles += new_particles
    particles = sorted(particles, key=lambda p: p.w, reverse=True)[0:PARTICLE_COUNT]
