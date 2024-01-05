import numpy as np

# Charger la carte
map_matrix_path = 'map_matrix.npy'  # Assurez-vous que c'est le bon chemin
map_matrix = np.load(map_matrix_path)


def is_collision(obs, s_start, s_end, grid):
        """
        check if the line segment (s_start, s_end) is collision.
        :param s_start: start node
        :param s_end: end node
        :return: True: is collision / False: not collision
        """

        # if s_start in obs or s_end in obs:
        #     print('obstacle')
        #     return True
        if grid[s_start] == -1 or grid[s_end] == -1:
            print('obstacle')
            return True

        if s_start[0] != s_end[0] and s_start[1] != s_end[1]:
            if s_end[0] - s_start[0] == s_start[1] - s_end[1]:
                s1 = (min(s_start[0], s_end[0]), min(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
            else:
                s1 = (min(s_start[0], s_end[0]), max(s_start[1], s_end[1]))
                s2 = (max(s_start[0], s_end[0]), min(s_start[1], s_end[1]))

            # if s1 in obs or s2 in obs:
            #     print('obstacle1')
            #     return True

        return False

import matplotlib.pyplot as plt
import numpy as np

def plot_map(occupancy_grid, path=None):
    # Normalize the occupancy grid to be between 0 and 1
    norm_occupancy_grid = np.copy(occupancy_grid)
    norm_occupancy_grid[norm_occupancy_grid == -1] = 50
    norm_occupancy_grid = norm_occupancy_grid / np.max(norm_occupancy_grid)

    # Create a figure and axis with Matplotlib
    fig, ax = plt.subplots()
    
    # Display the normalized occupancy grid with the origin in the 'lower' position
    ax.imshow(norm_occupancy_grid, cmap='gray', origin='lower')

    # Calculate the center of the grid
    center_x, center_y = norm_occupancy_grid.shape[1] // 2, norm_occupancy_grid.shape[0] // 2

    # Set the limits of the axes to center the (0,0) in the middle of the plot
    ax.set_xlim(-center_x, center_x)
    ax.set_ylim(-center_y, center_y)

    # If a path is provided, plot it on the map
    if path is not None:
        # Adjust path coordinates to the new center and plot
        adjusted_path = [(x - center_x, y - center_y) for x, y in path]
        ax.plot([x for x, y in adjusted_path], [y for x, y in adjusted_path], 'r-', linewidth=2)

    # Show a grid on the plot
    plt.grid(True)
    plt.show()

# To use this function, ensure you have loaded your occupancy grid and path,
# then call plot_map(occupancy_grid, path) with the appropriate arguments.



s_start = (0, 0)
s_goal = (200, 1247)

# Trouver les positions des obstacles
# Les obstacles sont représentés par la valeur -1
obstacle_positions = np.argwhere(map_matrix == -1)
free_space_positions = np.argwhere(map_matrix == 0)
#print(map_matrix)
plot_map(map_matrix)

# Supposons que free_space_positions et obstacle_positions sont vos listes de positions
# Nous allons d'abord convertir ces tableaux NumPy en listes de tuples

free_space_tuples = [tuple(pos) for pos in free_space_positions]
obstacle_tuples = [tuple(pos) for pos in obstacle_positions]

# Maintenant, nous pouvons créer des ensembles et trouver l'intersection
free_space_set = set(free_space_tuples)
obstacle_set = set(obstacle_tuples)

# Trouver l'intersection
intersection = free_space_set.intersection(s_goal)

# Afficher l'intersection
print(intersection)

is_collision(free_space_positions, s_start, s_goal,map_matrix)

# Afficher les positions des obstacles
#print("Positions des obstacles (y, x):")
#print(obstacle_positions)
