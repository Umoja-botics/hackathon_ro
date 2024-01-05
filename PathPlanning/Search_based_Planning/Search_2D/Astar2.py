import numpy as np
import heapq
import matplotlib.pyplot as plt

class AStarMatrix:
    def __init__(self, grid, start, goal):
        self.grid = grid
        self.start = start
        self.goal = goal
        self.open_set = []
        self.closed_set = set()
        self.parents = {}
        self.g_scores = {start: 0}
        self.f_scores = {start: self.heuristic(start, goal)}

    def heuristic(self, a, b):
        # Using Manhattan distance as heuristic
        return abs(a[0] - b[0]) + abs(a[1] - b[1])

    def get_neighbors(self, node):
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 4-connectivity
        neighbors = []
        for direction in directions:
            neighbor = (node[0] + direction[0], node[1] + direction[1])
            if (0 <= neighbor[0] < self.grid.shape[0] and
                0 <= neighbor[1] < self.grid.shape[1] and
                self.grid[neighbor] != -1):  # -1 represents obstacles
                neighbors.append(neighbor)
        return neighbors

    def search(self):
        heapq.heappush(self.open_set, (self.f_scores[self.start], self.start))

        while self.open_set:
            current_f_score, current = heapq.heappop(self.open_set)

            if current == self.goal:
                return self.reconstruct_path(current)

            self.closed_set.add(current)

            for neighbor in self.get_neighbors(current):
                if neighbor in self.closed_set:
                    continue

                tentative_g_score = self.g_scores[current] + 1  # Assuming uniform cost

                if neighbor not in self.open_set or tentative_g_score < self.g_scores.get(neighbor, float('inf')):
                    self.parents[neighbor] = current
                    self.g_scores[neighbor] = tentative_g_score
                    self.f_scores[neighbor] = tentative_g_score + self.heuristic(neighbor, self.goal)
                    heapq.heappush(self.open_set, (self.f_scores[neighbor], neighbor))

        return []  # If there's no path

    def reconstruct_path(self, current):
        path = []
        while current in self.parents:
            path.append(current)
            current = self.parents[current]
        path.append(self.start)  # optional: to include the start node
        path.reverse()  # The path is constructed in reverse order
        return path

def visualize_path(map_matrix, path, start, goal):
    print(path)
    vis_map = np.array(map_matrix, dtype=np.int32)
    for y, x in path:
        vis_map[x, y] = 50
    vis_map[start[1], start[0]] = 150
    vis_map[goal[1], goal[0]] = 200
    plt.figure(figsize=(10, 10))
    plt.imshow(vis_map, cmap='hot', interpolation='nearest')
    plt.show()

# Load the map matrix
map_matrix_path = 'map_matrix.npy'  # Update this to the correct path
map_matrix = np.load(map_matrix_path)

# Define the start and goal points as specified
start_point = (0, 0)  
goal_point = (400, 400) 

# Instantiate and run AStar
astar_solver = AStarMatrix(map_matrix, start_point, goal_point)
path = astar_solver.search()

# Visualize the path on the map
visualize_path(map_matrix, path, start_point, goal_point)
