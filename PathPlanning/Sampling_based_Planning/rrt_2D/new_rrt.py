import numpy as np
import matplotlib.pyplot as plt
import random
import math

class Node:
    """
    Classe pour représenter un nœud dans RRT*
    """
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None

def load_occupancy_grid(npy_file_path):
    """
    Charge la matrice d'occupation à partir d'un fichier .npy
    """
    return np.load(npy_file_path)

def collision_free(node, occupancy_grid):
    """
    Vérifie si un nœud est dans un espace libre
    """
    return occupancy_grid[int(node.y)][int(node.x)] == 0

def nearest_node(node_list, n):
    """
    Trouve le nœud le plus proche dans la liste de nœuds
    """
    return min(node_list, key=lambda node: math.hypot(node.x - n.x, node.y - n.y))

def steer(from_node, to_node, extend_length=float('inf')):
    """
    Calcule un nouveau nœud en direction de 'to_node' à partir de 'from_node'
    """
    new_node = Node(from_node.x, from_node.y)
    d, theta = distance_and_angle(from_node, to_node)
    
    new_node.x += min(d, extend_length) * math.cos(theta)
    new_node.y += min(d, extend_length) * math.sin(theta)
    
    new_node.cost = from_node.cost + min(d, extend_length)
    new_node.parent = from_node
    
    return new_node

def distance_and_angle(from_node, to_node):
    """
    Calcule la distance et l'angle entre deux nœuds
    """
    dx = to_node.x - from_node.x
    dy = to_node.y - from_node.y
    return math.hypot(dx, dy), math.atan2(dy, dx)

def rrt_star_planning(start, goal, occupancy_grid, iter_max=500, extend_length=30.0):
    """
    Génère un chemin à l'aide de l'algorithme RRT*
    """
    node_list = [start]
    
    for _ in range(iter_max):
        # Échantillonne un point aléatoire
        rnd_node = Node(random.uniform(0, occupancy_grid.shape[1]), random.uniform(0, occupancy_grid.shape[0]))
        
        # Trouve le nœud le plus proche
        nearest = nearest_node(node_list, rnd_node)
        
        # Steer vers le point échantillonné
        new_node = steer(nearest, rnd_node, extend_length)
        
        # Vérifie si le chemin est libre de collision
        if collision_free(new_node, occupancy_grid):
            node_list.append(new_node)
            
            # Vérifie si nous avons atteint le but
            if distance_and_angle(new_node, goal)[0] <= extend_length:
                final_node = steer(new_node, goal)
                if collision_free(final_node, occupancy_grid):
                    return get_path(final_node)
    
    return None  # Chemin non trouvé

def get_path(node):
    """
    Retrace le chemin à partir du nœud d'arrivée jusqu'au départ
    """
    path = [(node.x, node.y)]
    while node.parent is not None:
        node = node.parent
        path.append((node.x, node.y))
    
    return path[::-1]  # Retourne le chemin inversé

# Paramètres
npy_file_path = 'map_matrix.npy'  # Remplacez par le chemin vers votre fichier .npy
start = Node(10, 10)  # Coordonnées de départ
goal = Node(1000, 60)  # Coordonnées d'arrivée

# Charger la carte de type .npy
occupancy_grid = load_occupancy_grid(npy_file_path)

# Exécuter l'algorithme RRT*
path = rrt_star_planning(start, goal, occupancy_grid)

# Dessiner la carte et la trajectoire
if path is not None:
    plt.plot([x for (x, y) in path], [y for (x, y) in path], 'r')
    plt.plot(start.x, start.y, 'go')
    plt.plot(goal.x, goal.y, 'bo')
    plt.imshow(occupancy_grid, cmap='gray', origin='lower')
    plt.grid(True)
    plt.pause(0.01)  # Nécessaire pour la mise à jour de l'affichage
    plt.show()
else:
    print("No path found!")
