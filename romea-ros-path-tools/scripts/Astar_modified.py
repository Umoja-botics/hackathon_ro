#!/usr/bin/env python3
"""
A_star 2D
@author: huiming zhou
"""
import sys
sys.path.append('/home/klein/ws/src/romea-ros-path-tools')

import os
import sys
import math
import heapq
import numpy as np
import matplotlib.pyplot as plt
import json
from romea_path_tools.path import Path
# sys.path.append(os.path.dirname(os.path.abspath(__file__)) + "/../../Search_based_Planning/")

# sys.path.append('/Users/ijkep/PycharmProjects/myfirstproject/PathPlanning')

# from Search_based_Planning.Search_2D import plotting, env

import env2, plotting1
import traj_bezier

class AStar:
    """AStar set the cost + heuristics as the priority
    """
    def __init__(self, s_start, s_goal, heuristic_type):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type

        self.Env = env2.Env()  # class Env

        self.u_set = self.Env.motions  # feasible input set
        self.obs = self.Env.obs  # position of obstacles

        self.OPEN = []  # priority queue / OPEN set
        self.CLOSED = []  # CLOSED set / VISITED order
        self.PARENT = dict()  # recorded parent
        self.g = dict()  # cost to come

    def searching(self):
        """
        A_star Searching.
        :return: path, visited order
        """

        self.PARENT[self.s_start] = self.s_start
        self.g[self.s_start] = 0
        self.g[self.s_goal] = math.inf
        heapq.heappush(self.OPEN,
                       (self.f_value(self.s_start), self.s_start))

        while self.OPEN:
            _, s = heapq.heappop(self.OPEN)
            self.CLOSED.append(s)

            if s == self.s_goal:  # stop condition
                break

            for s_n in self.get_neighbor(s):
                new_cost = self.g[s] + self.cost(s, s_n)

                if s_n not in self.g:
                    self.g[s_n] = math.inf

                if new_cost < self.g[s_n]:  # conditions for updating Cost
                    self.g[s_n] = new_cost
                    self.PARENT[s_n] = s
                    heapq.heappush(self.OPEN, (self.f_value(s_n), s_n))

        return self.extract_path(self.PARENT), self.CLOSED

    def searching_repeated_astar(self, e):
        """
        repeated A*.
        :param e: weight of A*
        :return: path and visited order
        """

        path, visited = [], []

        while e >= 1:
            p_k, v_k = self.repeated_searching(self.s_start, self.s_goal, e)
            path.append(p_k)
            visited.append(v_k)
            e -= 0.5

        return path, visited

    def repeated_searching(self, s_start, s_goal, e):
        """
        run A* with weight e.
        :param s_start: starting state
        :param s_goal: goal state
        :param e: weight of a*
        :return: path and visited order.
        """

        g = {s_start: 0, s_goal: float("inf")}
        PARENT = {s_start: s_start}
        OPEN = []
        CLOSED = []
        heapq.heappush(OPEN,
                       (g[s_start] + e * self.heuristic(s_start), s_start))

        while OPEN:
            _, s = heapq.heappop(OPEN)
            CLOSED.append(s)

            if s == s_goal:
                break

            for s_n in self.get_neighbor(s):
                new_cost = g[s] + self.cost(s, s_n)

                if s_n not in g:
                    g[s_n] = math.inf

                if new_cost < g[s_n]:  # conditions for updating Cost
                    g[s_n] = new_cost
                    PARENT[s_n] = s
                    heapq.heappush(OPEN, (g[s_n] + e * self.heuristic(s_n), s_n))

        return self.extract_path(PARENT), CLOSED

    def get_neighbor(self, s):
        """
        find neighbors of state s that not in obstacles.
        :param s: state
        :return: neighbors
        """

        return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]

    def cost(self, s_start, s_goal):
        """
        Calculate Cost for this motion
        :param s_start: starting node
        :param s_goal: end node
        :return:  Cost for this motion
        :note: Cost function could be more complicate!
        """

        if self.is_collision(s_start, s_goal):
            return math.inf

        return math.hypot(s_goal[0] - s_start[0], s_goal[1] - s_start[1])

    def is_collision(self, s_start, s_end):
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """

        if s_start in self.obs or s_end in self.obs:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            if s1 in self.obs or s2 in self.obs:
                return True

        return False

    def f_value(self, s):
        """
        f = g + h. (g: Cost to come, h: heuristic value)
        :param s: current state
        :return: f
        """

        return self.g[s] + self.heuristic(s)

    def extract_path(self, PARENT):
        """
        Extract the path based on the PARENT set.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            #print(s)
            s = PARENT[s]
            path.append(s)

            if s == self.s_start:
                break

        return list(path)

    def heuristic(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        heuristic_type = self.heuristic_type  # heuristic type
        goal = self.s_goal  # goal node

        if heuristic_type == "manhattan":
            return abs(goal[0] - s[0]) + abs(goal[1] - s[1])
        else:
            return math.hypot(goal[0] - s[0], goal[1] - s[1])

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

    # plt.grid(True)
    plt.show()


def remap_coordinates(coord, grid_shape):
    
    grid_shapes = (grid_shape.shape[0], grid_shape.shape[1])
    

    
    new_x_min, new_x_max = -25, 225
    new_y_min, new_y_max = -25, 225

   
    x_scale = (new_x_max - new_x_min) / grid_shapes[0]
    y_scale = (new_y_max - new_y_min) / grid_shapes[1]

   
    new_x = new_x_min + coord[0] * x_scale  
    new_y = new_y_min + coord[1] * y_scale  

    return (new_x, new_y)

def remap_inverse(coord):
    
    new_x_min, new_x_max = 0, 1250
    new_y_min, new_y_max = 0, 1250

    x_scale =  (new_x_max - new_x_min) / 200
    y_scale =  (new_y_max - new_y_min) / 205

    new_x = new_x_min + coord[0] * x_scale  
    new_y = new_y_min + coord[1] * y_scale   # en convertissant en entier on aura environ 1px d'erreur soit 0.2m d'erreur

    return (int(new_x), int(new_y))

def import_json(json_file_path):

    point = []
    # Read and load the JSON data
    with open(json_file_path, 'r') as file:
        json_data = json.load(file)
    
    point.append(((json_data["start"][3:][0]),(json_data["start"][3:][1])))
    for i in range (len(json_data["waypoints"])):
        point.append(((json_data["waypoints"][i][3:][0]),(json_data["waypoints"][i][3:][1])))
        
    return point

def import_json_vigne(json_file_path):

    point = []
    # Read and load the JSON data
    with open(json_file_path, 'r') as file:
        json_data = json.load(file)
    
    
    for i in range (len(json_data["fields"][0]["points"])):
        point.append((json_data["fields"][0]["points"][i][3:][0],json_data["fields"][0]["points"][i][3:][1]))
        
    return point

def import_json_crops(json_file_path):

    point = []
    # Read and load the JSON data
    with open(json_file_path, 'r') as file:
        json_data = json.load(file)
    
    
    for i in range (len(json_data["fields"][1]["points"])):
        point.append((json_data["fields"][1]["points"][i][3:][0],json_data["fields"][1]["points"][i][3:][1]))
        
    return point

def plot_map(occupancy_grid, path):
    norm_occupancy_grid = np.copy(occupancy_grid)
    norm_occupancy_grid[norm_occupancy_grid == -1] = 50
    norm_occupancy_grid = norm_occupancy_grid / 100

    fig, ax = plt.subplots()
    ax.imshow(norm_occupancy_grid, cmap='gray', origin='lower')

    if path is not None:
        ax.plot([x for (x, y) in path], [y for (x, y) in path], 'r', linewidth=2)
        
    # plt.grid(True)
    plt.show()

def main():

   

    map_matrix_path = 'map_matrix.npy'  # Update this to the correct path
    json_file_path = '/home/klein/ws/src/romea-ros-path-tools/scripts/challenge1_waypoints_01.json' # import the json file
    map_matrix = np.load(map_matrix_path)
    norm_occupancy_grid = np.copy(map_matrix)
    norm_occupancy_grid[norm_occupancy_grid == -1] = 50
    norm_occupancy_grid = norm_occupancy_grid / 100

    #waypoint = import_json(json_file_path)
    waypoint_vigne = import_json_vigne(json_file_path)
    waypoint_crope = import_json_crops(json_file_path)

    #waypoint = [(597, 750),(444, 822), (893, 875),(492, 869),(225, 764),(673, 700),(826, 713),(1048, 872),(1115, 1023),(1007, 1087),(825, 1138), (792, 1017),(714, 991),(611, 914)]
    #waypoint = [(597, 750),(893, 875)]#,(444, 822),(492, 869),(225, 764),(673, 700),(826, 713),(1048, 872),(1115, 1023),(1007, 1087),(825, 1138), (792, 1017),(714, 991),(611, 914)]  
    # print(waypoint) (673, 700)
    waypoint = [(600, 750),(673, 700), (893, 875),(444, 822),(225, 764)]
    #waypoint = [(543, 712),(404, 766),(812, 815),(447, 810),(612, 652),(751, 665),(953, 813),(1013, 953),(915, 1013),(750, 1060)]
    # w1 = remap_inverse(waypoint[0],map_matrix)
    # print(w1)
    # w = remap_coordinates(w1, map_matrix) 
    # print(w)
    
    Paths = []

    for i in range (len(waypoint)-1):
    #     if i == 0 :

    #         w1 = (597, 750)
    #         w2 =  (893, 875)

    #     elif i == 1 :
    #         pass
        
    #     else :

    #         #w1 = remap_inverse(waypoint[i]) # transformation point map reel en point px
    #         w1 = waypoint[i] # transformation point map reel en point px
    #         print("w1",w1)
            
    #         #w2 = remap_inverse(waypoint[i+1])
    #         w2 = waypoint[i+1]
    #         print("w2",w2)
       

        #astar = AStar(w1, w2, "euclidean")
        astar = AStar(waypoint[i], waypoint[i+1], "euclidean")
        path, visited = astar.searching()
        #print(path)
        Paths.extend(path[::-1])

    # for i in range(1) :

    #     if i == 0 :

    #         path1 = traj_bezier.traj_lineaire(waypoint_vigne[0],waypoint_vigne[1], 15)
    #         Paths.extend(path1)
    #         path1 = traj_bezier.create_cubic_bezier_turn(waypoint_vigne[1],  waypoint_vigne[2], -10, 10)
    #         Paths.extend(path1)
    #         path = traj_bezier.traj_lineaire(waypoint_vigne[2],waypoint_vigne[3], 15)
    #         Paths.extend(path1)

    #     if i == 1 :

    #         path1 = traj_bezier.traj_lineaire(waypoint_crope[0],waypoint_crope[1], 15)
    #         Paths.extend(path1)
    #         path1 = traj_bezier.create_cubic_bezier_turn(waypoint_crope[1],  waypoint_crope[2], -10, 10)
    #         Paths.extend(path1)
    #         path = traj_bezier.traj_lineaire(waypoint_crope[2],waypoint_crope[3], 15)
    #         Paths.extend(path1)

    # x_coords, y_coords = zip(*Paths)

    # plt.scatter(x_coords, y_coords, color='red', label="Points intermédiaires")
        
        
    # plt.legend()
    # plt.grid(True)
    # plt.xlabel("X")
    # plt.ylabel("Y")
    # plt.title("Points intermédiaires sur un segment de ligne")
    # plt.show()
    
    #plot_map(norm_occupancy_grid,Paths)
    #print(Paths)

    
        

    path_f = Path()
    path_f.name = 'test'
    path_f.columns = ['x', 'y', 'speed']  # 'x' and 'y' are required (these are the default)
    path_f.anchor = (46.339159,3.433923, 278.142)  # for challenge


    for i in range(len(Paths)):
        path_remap = remap_coordinates(Paths[i], map_matrix)
        
        path_f.append_point([path_remap[0], path_remap[1], 0.85])

    path_f.save('test_639.traj')


if __name__ == '__main__':
    main()