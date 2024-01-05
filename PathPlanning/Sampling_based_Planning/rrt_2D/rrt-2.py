import numpy as np
import matplotlib.pyplot as plt
import random
import math
from scipy.interpolate import CubicSpline

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None

def load_occupancy_grid(npy_file_path):
    return np.load(npy_file_path)

def collision_free(node, occupancy_grid, step_size=1):
    if occupancy_grid[int(node.y)][int(node.x)] != 0:
        return False
    
    if node.parent is not None:
        steps = int(math.hypot(node.x - node.parent.x, node.y - node.parent.y) / step_size)
        for i in range(steps):
            intermediate_x = node.parent.x + (node.x - node.parent.x) * (i / steps)
            intermediate_y = node.parent.y + (node.y - node.parent.y) * (i / steps)
            if occupancy_grid[int(intermediate_y)][int(intermediate_x)] != 0:
                return False
    return True

def nearest_node(node_list, n):
    return min(node_list, key=lambda node: math.hypot(node.x - n.x, node.y - n.y))   #renvoie le noeud le plus proche

def steer(from_node, to_node, extend_length=float('inf')):
    new_node = Node(from_node.x, from_node.y)
    d, theta = distance_and_angle(from_node, to_node)
    
    new_node.x += min(d, extend_length) * math.cos(theta)
    new_node.y += min(d, extend_length) * math.sin(theta)
    
    new_node.cost = from_node.cost + min(d, extend_length)
    new_node.parent = from_node
    
    return new_node

def distance_and_angle(from_node, to_node):
    dx = to_node.x - from_node.x
    dy = to_node.y - from_node.y
    return math.hypot(dx, dy), math.atan2(dy, dx)

def rrt_star_planning(start, goal, occupancy_grid, iter_max=500, extend_length=30.0):
    node_list = [start]
    
    for _ in range(iter_max):
        rnd_node = Node(random.uniform(0, occupancy_grid.shape[1]), random.uniform(0, occupancy_grid.shape[0]))
        nearest = nearest_node(node_list, rnd_node)
        new_node = steer(nearest, rnd_node, extend_length)
        
        if collision_free(new_node, occupancy_grid):
            node_list.append(new_node)
            
            if distance_and_angle(new_node, goal)[0] <= extend_length:
                final_node = steer(new_node, goal)
                if collision_free(final_node, occupancy_grid):
                    return get_path(final_node)
    
        # for i in range(len(node_list)) :
        #     print(node_list[i])
    

    return None

def get_path(node):
    path = [(node.x, node.y)]
    while node.parent is not None:
        node = node.parent
        path.append((node.x, node.y))
    
    return path[::-1]

## new

def smooth_path(path, occupancy_grid):
    if not path:
        return path
    
    # Extraire les points de chemin
    x = [point[0] for point in path]
    y = [point[1] for point in path]
    
    # Créer une spline cubique à travers les points de chemin
    cs_x = CubicSpline(np.arange(len(x)), x)
    cs_y = CubicSpline(np.arange(len(y)), y)
    
    # Échantillonner la spline pour obtenir un chemin lissé
    xnew = np.linspace(0, len(x) - 1, num=5*len(x))
    ynew = np.linspace(0, len(y) - 1, num=5*len(y))
    x_smooth = cs_x(xnew)
    y_smooth = cs_y(ynew)
    
    # Vérifier que le chemin lissé ne passe pas par des obstacles
    smooth_path = []
    for xi, yi in zip(x_smooth, y_smooth):
        if collision_free(Node(xi, yi), occupancy_grid):
            smooth_path.append((xi, yi))
    
    return smooth_path

def plot_map(occupancy_grid, path, start, goal):
    norm_occupancy_grid = np.copy(occupancy_grid)
    norm_occupancy_grid[norm_occupancy_grid == -1] = 50
    norm_occupancy_grid = norm_occupancy_grid / 100

    fig, ax = plt.subplots()
    ax.imshow(norm_occupancy_grid, cmap='gray', origin='lower')
    
    if path is not None:
        ax.plot([x for (x, y) in path], [y for (x, y) in path], 'r', linewidth=2)
        ax.plot(start.x, start.y, 'go')
        ax.plot(goal.x, goal.y, 'bo')

    plt.grid(True)
    plt.show()

# Paramètres
npy_file_path = 'map_matrix.npy'  # Remplacer par le chemin d'accès à votre fichier .npy
start = Node(10, 10)  # Point de départ
goal = Node(200, 955)  # Point d'arrivée

# Charger la carte d'occupation et exécuter RRT*
occupancy_grid = load_occupancy_grid(npy_file_path)
path = rrt_star_planning(start, goal, occupancy_grid)
smoothed_path = smooth_path(path, occupancy_grid)

# Afficher la carte et le chemin
#plot_map(occupancy_grid, path, start, goal)
plot_map(occupancy_grid, smoothed_path, start, goal)