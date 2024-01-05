#!/usr/bin/env python3

"""
RRT_2D
@author: huiming zhou
"""

import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt

# sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                 "/Sampling_based_Planning/")
#print(sys.path)
sys.path.append('/home/klein/ws/src/PathPlanning')
# sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
#                 "/../../Sampling_based_Planning/")

# sys.path.append('/home/klein/ws/src/PathPlanning/Sampling_based_Planning')

from Sampling_based_Planning.rrt_2D import env, plotting, utils
# import env, plotting, utils

class Node:
    def __init__(self, n):
        self.x = n[0]
        self.y = n[1]
        self.parent = None


class Rrt:
    def __init__(self, s_start, s_goal, step_len, goal_sample_rate, iter_max):
        self.s_start = Node(s_start)
        self.s_goal = Node(s_goal)
        self.step_len = step_len
        self.goal_sample_rate = goal_sample_rate
        self.iter_max = iter_max
        self.vertex = [self.s_start]

        #self.env = env.Env()
        
        #self.env = occupancy_grid
        self.plotting = plotting.Plotting(s_start, s_goal)
        self.utils = utils.Utils()
        
        # self.x_range = self.env1.x_range
        # self.y_range = self.env1.y_range
        # self.obs_circle = self.env.obs_circle
        # self.obs_rectangle = self.env.obs_rectangle
        # self.obs_boundary = self.env.obs_boundary

    def planning(self, occupancy_grid):
        for _ in range(self.iter_max):
            node_rand = self.generate_random_node(self.goal_sample_rate, occupancy_grid)
            node_near = self.nearest_neighbor(self.vertex, node_rand)
            node_new = self.new_state(node_near, node_rand)
            

            if node_new : #and not self.utils.is_collision(node_near, node_new):
                self.vertex.append(node_new)
                dist, _ = self.get_distance_and_angle(node_new, self.s_goal)

                if dist <= self.step_len :#and not self.utils.is_collision(node_new, self.s_goal):
                    self.new_state(node_new, self.s_goal)
                    return self.extract_path(node_new)

        return None

    def generate_random_node(self, goal_sample_rate, occupancy_grid):
        delta = self.utils.delta

        if np.random.random() > goal_sample_rate:

              return Node((np.random.uniform(0, occupancy_grid.shape[1]), np.random.uniform(0, occupancy_grid.shape[0])))
            # return Node((np.random.uniform(self.x_range[0] + delta, self.x_range[1] - delta),
            #              np.random.uniform(self.y_range[0] + delta, self.y_range[1] - delta)))

        return self.s_goal

    @staticmethod
    def nearest_neighbor(node_list, n):
        return node_list[int(np.argmin([math.hypot(nd.x - n.x, nd.y - n.y)
                                        for nd in node_list]))]

    def new_state(self, node_start, node_end):
        dist, theta = self.get_distance_and_angle(node_start, node_end)

        dist = min(self.step_len, dist)
        node_new = Node((node_start.x + dist * math.cos(theta),
                         node_start.y + dist * math.sin(theta)))
        node_new.parent = node_start

        return node_new

    def extract_path(self, node_end):
        path = [(self.s_goal.x, self.s_goal.y)]
        node_now = node_end

        while node_now.parent is not None:
            node_now = node_now.parent
            path.append((node_now.x, node_now.y))

        return path

    @staticmethod
    def get_distance_and_angle(node_start, node_end):
        dx = node_end.x - node_start.x
        dy = node_end.y - node_start.y
        return math.hypot(dx, dy), math.atan2(dy, dx)
    
def plot_map(occupancy_grid, path, start, goal):
    norm_occupancy_grid = np.copy(occupancy_grid)
    norm_occupancy_grid[norm_occupancy_grid == -1] = 50
    norm_occupancy_grid = norm_occupancy_grid / 100

    fig, ax = plt.subplots()
    ax.imshow(norm_occupancy_grid, cmap='gray', origin='lower')
    
    if path is not None:
        ax.plot([x for (x, y) in path], [y for (x, y) in path], 'r', linewidth=2)
        ax.plot(start[0], start[1], 'go')
        ax.plot(goal[0], goal[1], 'bo')

    plt.grid(True)
    plt.show()


def main():
    x_start = (2, 2)  # Starting node
    x_goal = (1000, 400)  # Goal node

    npy_file_path = 'map_matrix.npy'

        # Charger la matrice d'occupation à partir du fichier .npy
    occupancy_grid = np.load(npy_file_path)

    rrt = Rrt(x_start, x_goal, 0.5, 0.05, 10000)
    path = rrt.planning(occupancy_grid)
    #print(path)

    

    if path:
        plot_map(occupancy_grid, path, x_start, x_goal)
    else:
        print("No Path Found!")

    # if path:
    #     rrt.plotting.animation(rrt.vertex, path, "RRT", True)
    # else:
    #     print("No Path Found!")


if __name__ == '__main__':
    main()
