"""
Env 2D
@author: huiming zhou
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors


class Env:
    def __init__(self):
        self.x_range = 1251  # size of background
        self.y_range = 1251
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]
        self.obs = self.obs_map()

    def update_obs(self, obs):
        self.obs = obs

    def obs_map(self):
        """
        Initialize obstacles' positions
        :return: map of obstacles
        """

        x = self.x_range
        y = self.y_range
        obs = set()

        for i in range(x):
            obs.add((i, 0))
        for i in range(x):
            obs.add((i, y - 1))

        for i in range(y):
            obs.add((0, i))
        for i in range(y):
            obs.add((x - 1, i))

        # for i in range(600):
        #    obs.add((i, 800))

        # chemin vers le npy
        npy_file_path = 'map_matrix.npy'

        # Charger la matrice d'occupation à partir du fichier .npy
        occupancy_grid = np.load(npy_file_path)

        # Valeur donnée à rechercher
        valeur_recherchee = -1

        # Trouver les indices où la valeur est égale à la valeur recherchée
        indices = np.argwhere(occupancy_grid == valeur_recherchee)

        for indice in indices:
            obs.add((indice[1] + 1, indice[0] + 1))


        # Valeur donnée à rechercher
        valeur_recherchee2 = 100

        # Trouver les indices où la valeur est égale à la valeur recherchée
        indices2 = np.argwhere(occupancy_grid == valeur_recherchee2)

        for indice in indices2:
            obs.add((indice[1] + 1, indice[0] + 1))
            obs.add((indice[1] + 2, indice[0] + 1))
            obs.add((indice[1], indice[0] + 1))
            obs.add((indice[1] + 1, indice[0]))
            obs.add((indice[1] + 1, indice[0] + 2))
            obs.add((indice[1] + 2, indice[0] + 2))
            obs.add((indice[1], indice[0] + 2))
            obs.add((indice[1] + 2, indice[0]))
            obs.add((indice[1], indice[0]))


        '''
        for i in range(400):
            obs.add((401, i))

        for i in range(1250, 700):
            obs.add((1000, i))
        for i in range(600):
            obs.add((600, i))
        '''
        return obs