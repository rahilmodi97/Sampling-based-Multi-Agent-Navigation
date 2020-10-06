# agent.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
#
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#
# COde submitted by Huzefa Kaaglwala (C48290423) and rahil Modi (C14109603)
import numpy as np
import math
from math import sqrt
import matplotlib.pyplot as plt
from scipy.spatial import distance as dist

class Agent(object):
    def __init__(self, csvParameters, dhor = 10, goalRadiusSq=1):
        """
            Takes an input line from the csv file,
            and initializes the agent
        """
        self.id = int(csvParameters[0]) # the id of the agent
        self.gid = int(csvParameters[1]) # the group id of the agent
        self.pos = np.array([float(csvParameters[2]), float(csvParameters[3])]) # the position of the agent
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(csvParameters[4]), float(csvParameters[5])]) # the goal of the agent
        self.prefspeed = float(csvParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(sqrt(self.gvel.dot(self.gvel )))*self.prefspeed
        self.maxspeed = float(csvParameters[7]) # the maximum sped of the agent
        self.radius = float(csvParameters[8]) # the radius of the agent
        self.goalRadiusSq =goalRadiusSq # parameter to determine if agent is close to the goal
        self.atGoal = False # has the agent reached its goal?
        self.dhor = dhor # the sensing radius
        self.vnew = np.zeros(2) # the new velocity of the agent

    def computeNewVelocity(self, neighbors=[]):
        """
            Your code to compute the new velocity of the agent.
            You probably need to pass here a list of all the agents in the simulation to determine the agent's nearest neighbors.
            The code should set the vnew of the agent to be one of the sampled admissible one. Please do not directly set here the agent's velocity.
        """
        # Initializing variables for computation
        N = 200 # The sample size over which random velocities are generated
        cand_vel = np.zeros((N,2)) # Creating a matrix to store candidate velocities' x and y components
        neighbor_pos = []  #
        neighbor_size = [] #
        neighbor_id = []
        neighbor_vel = []
        alpha = 12.5
        beta = 12.5
        gamma = 25

        # Uniformly sampling candidate velocities using a polar coordinate method
        vel_array = np.random.uniform(low = 0, high = (self.maxspeed)**2+0.001, size = N) # Creating the random velocties
        theta = np.random.uniform(low = 0, high = 2*(np.pi)+0.001, size = N) # Creating random angles
        sample_vel_x = np.sqrt(vel_array)*np.cos(theta) # Vx = sqrt(r)cos(theta)
        sample_vel_y = np.sqrt(vel_array)*np.sin(theta) # Vy = sqrt(r)sin(theta)
        cand_vel[:,0] = sample_vel_x
        cand_vel[:,1] = sample_vel_y
        #plt.scatter(sample_vel_x, sample_vel_y)
        #plt.axis('equal')
        #plt.show()

        goal_dist = dist.euclidean(self.goal, self.pos)
        # This condition will set velocity to goal velocity if the agent goes near the goal by a distance of sensing radius
        if goal_dist > self.dhor:
            for value in neighbors: # If two objects are near each other, they will be counted as "neighbors" and their properties will be enumerated
                if self.id != value.id: # Loop is active only for distinct agents
                    obj_sense_dist = dist.euclidean(value.pos, self.pos) - (self.radius + self.radius) # Object sensing radius between neighbor and self distance and their radii subtracted to give the circunference to circumference distance
                    if obj_sense_dist < self.dhor: # If neighbors are detected, then neighbor attributes are appended
                        neighbor_id.append(value.id)
                        neighbor_pos.append(value.pos)
                        neighbor_size.append(value.radius)
                        neighbor_vel.append(value.vel)

            # Evaluating fitness of velocities and selecting new Velocity
            cost1 = np.zeros(len(cand_vel))
            cost2 = np.zeros(len(cand_vel))
            cost3 = np.zeros(len(cand_vel))
            cost_func = np.zeros(len(cand_vel))
            ttc = np.zeros(len(neighbor_id))
            if len(neighbor_id) != 0: # Calculating the new velocity only if a neighbor is detected
                for x in range(len(cand_vel)):
                    for y in range(len(neighbor_id)):
                        rad = self.radius + neighbor_size[y]
                        rel_pos = self.pos - neighbor_pos[y]
                        c = np.dot(rel_pos, rel_pos) - rad*rad
                        if c < 0:
                            ttc[y] = 0
                        v = cand_vel[x] - neighbor_vel[y]
                        a = np.dot(v, v)
                        b = np.dot(rel_pos,v)
                        if b > 0:
                            ttc[y] = np.inf
                        discr = b*b - a*c
                        if discr <= 0:
                            ttc[y] = np.inf
                        if discr > 0:
                            tau = c / (-b + np.sqrt(discr))
                            if tau < 0:
                                ttc[y] = np.inf
                            else:
                                ttc[y] = tau
                    try: # try-except block to catch errors in calculating minimum ttc
                        ttc_fin = np.amin(ttc)
                        cost1[x] = alpha * dist.euclidean(cand_vel[x], self.gvel)
                        cost2[x] = beta * dist.euclidean(cand_vel[x], self.vel)
                        cost3[x] = gamma / ttc_fin
                        cost_func[x] = cost1[x] + cost2[x] + cost3[x]
                        select_idx = np.argmin(cost_func)
                        fin_vel = cand_vel[select_idx]
                    except ValueError: #raised if 'ttc' is empty
                        self.vnew[:] = self.gvel[:]

                # Updating velocity to computed velocity
                if obj_sense_dist < self.dhor and not self.atGoal:
                    self.vnew[:] = fin_vel[:]
                else:
                    self.vnew[:] = self.gvel[:]
            elif not self.atGoal:
                self.vnew[:] = self.gvel[:]
        elif not self.atGoal:
            self.vnew[:] = self.gvel[:]

    def update(self, dt):
        """
            Code to update the velocity and position of the agent
            as well as determine the new goal velocity
        """
        if not self.atGoal:
            self.vel[:] = self.vnew[:]
            self.pos += self.vel*dt   #update the position

            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq:
                self.atGoal = True  # goal has been reached
            else:
                self.gvel = self.gvel/sqrt(distGoalSq)*self.prefspeed
