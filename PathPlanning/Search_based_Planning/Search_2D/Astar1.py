"""
A_star 2D
@author: huiming zhou
"""

import os
import sys
import math
import heapq
import numpy as np
import matplotlib.pyplot as plt
import json

sys.path.append(os.path.dirname(os.path.abspath(__file__)) +
                "/../../Search_based_Planning/")

# sys.path.append('/home/klein/ws/src/PathPlanning')

from Search_2D import plotting, env


class AStar:
    """AStar set the cost + heuristics as the priority
    """
    def __init__(self, s_start, s_goal, heuristic_type, occupancy_grid):
        self.s_start = s_start
        self.s_goal = s_goal
        self.heuristic_type = heuristic_type
        self.grid = occupancy_grid


        #self.Env = env.Env()  # class Env
        # obstacle_positions = np.argwhere(map_matrix == -1)
        #self.u_set = self.Env.motions  # feasible input set
        obs1 = np.argwhere( self.grid == 100)
        obs2 = np.argwhere( self.grid == -1) # position of obstacles
        self.obs = np.concatenate((obs1, obs2))

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

    # def get_neighbor(self, s):
    #     """
    #     find neighbors of state s that not in obstacles.
    #     :param s: state
    #     :return: neighbors
    #     """

    #     return [(s[0] + u[0], s[1] + u[1]) for u in self.u_set]
    
    def get_neighbor(self, node):
        directions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]  # 4-connectivity
        # directions = [(0, 1), (1, 0), (0, -1), (-1, 0)] 
        neighbors = []
        for direction in directions:
            neighbor = (node[0] + direction[0], node[1] + direction[1])
            if (0 <= neighbor[0] < self.grid.shape[0] and
                0 <= neighbor[1] < self.grid.shape[1] and
                self.grid[neighbor[0], neighbor[1]] != -1 and
                self.grid[neighbor[0], neighbor[1]] != 100):
                   # -1 represents obstacles
                #print(neighbor)
                neighbors.append(neighbor)
        return neighbors

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

        # if s_start in self.obs or s_end in self.obs:
        #     print('obstacle')
        #     return True
        if self.grid[s_start] == -1 or self.grid[s_end] == -1:
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            # if s1 in self.obs or s2 in self.obs:
            #     print(self.obs)
            #     return True

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

     # calcul du centre de la map
    center_x, center_y = norm_occupancy_grid.shape[1] // 2, norm_occupancy_grid.shape[0] // 2

    fig, ax = plt.subplots()
    ax.imshow(norm_occupancy_grid, cmap='gray', origin='lower')
    #extent = [-center_x, center_x, -center_y, center_y]
    #ax.imshow(norm_occupancy_grid, cmap='gray', origin='lower', extent=extent)


    if path is not None:
        
        ax.plot([x for (x, y) in path], [y for (x, y) in path], 'r', linewidth=2)
        ax.plot(start[0], start[1], 'go')
        ax.plot(goal[0], goal[1], 'bo')

    plt.grid(True)
    plt.show()



def import_json(json_file_path):

    point = []
    # Read and load the JSON data
    with open(json_file_path, 'r') as file:
        json_data = json.load(file)
    
    point.append((json_data["start"][3:][0],json_data["start"][3:][1]))
    for i in range (len(json_data["waypoints"])):
        point.append((json_data["waypoints"][i][3:][0],json_data["waypoints"][i][3:][1]))
        
    return point


def main():

        # Load the map matrix
    map_matrix_path = 'map_matrix4.npy'  # Update this to the correct path
    json_file_path = '/home/klein/challenge1_waypoints_01.json' # import the json file
    map_matrix = np.load(map_matrix_path)
    norm_occupancy_grid = np.copy(map_matrix)
    norm_occupancy_grid[norm_occupancy_grid == -1] = 50
    norm_occupancy_grid = norm_occupancy_grid / 100

    s_start = (543, 712)
    s_goal = (404, 766)

    waypoint = [(543, 712),(404, 766),(812, 815),(447, 810),(612, 652),(751, 665),(953, 813),(1013, 953),(915, 1013),(750, 1060)]

    # w1 (543, 712)
    # w2 (404, 766)
    # w2 (812, 815)
    # w1 (812, 815)
    # w2 (447, 810)
    # w1 (447, 810)
    # w2 (204, 712)
    # w1 (204, 712)
    # w2 (612, 652)
    # w1 (612, 652)
    # w2 (751, 665)
    # w1 (751, 665)
    # w2 (953, 813)
    # w1 (953, 813)
    # w2 (1013, 953)
    # w1 (1013, 953)
    # w2 (915, 1013)
    # w1 (915, 1013)
    # w2 (750, 1060)
    # w1 (750, 1060)
    # w2 (720, 948)
    # w1 (720, 948)
    # w2 (649, 923)
    # w1 (649, 923)
    # w2 (556, 852)
    # w1 (404, 766)
    # w2 (812, 815)
    # w1 (812, 815)
    # w2 (447, 810)
    # w1 (447, 810)
    # w2 (204, 712)
    # w1 (204, 712)
    # w2 (612, 652)
    # w1 (612, 652)
    # w2 (751, 665)
    # w1 (751, 665)
    # w2 (953, 813)
    # w1 (953, 813)
    # w2 (1013, 953)
    # w1 (1013, 953)
    # w2 (915, 1013)
    # w1 (915, 1013)
    # w2 (750, 1060)
    # w1 (750, 1060)
    # w2 (720, 948)
    # w1 (720, 948)
    # w2 (649, 923)
    # w1 (649, 923)
    # w2 (556, 852)

    # waypoint = import_json(json_file_path)
    Paths = []

    for i in range (len(waypoint)-1):
        

        astar = AStar(waypoint[i], waypoint[i+1], "euclidean", map_matrix)
        path, visited = astar.searching()
        Paths.extend(path)

    # astar = AStar(s_start, s_goal, "euclidean", map_matrix)
    # plot = plotting.Plotting(s_start, s_goal)

    # path, visited = astar.searching()
    #plot.animation(path, visited, "A*")  # animation

    plot_map(map_matrix, Paths, s_start, s_goal)
   # print(path[0][1])
   # print(type(path))


    # path, visited = astar.searching_repeated_astar(2.5)               # initial weight e = 2.5
    # plot.animation_ara_star(path, visited, "Repeated A*")


if __name__ == '__main__':
    main()
