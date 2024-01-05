
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.colors as mcolors

# chemin vers le npy
npy_file_path = 'map_matrix.npy'

# Charger la matrice d'occupation à partir du fichier .npy
occupancy_grid = np.load(npy_file_path)

# Normaliser les valeurs pour la visualisation
# Les cellules inconnues (-1) seront affichées en gris, les cellules libres (0) en blanc, et les obstacles (100) en noir.
normalized_grid = np.interp(occupancy_grid, (-1, 100), (0.5, 0))

normalized_grid[occupancy_grid == -1] = 0.5

# Créer une carte de couleurs personnalisée pour l'affichage
# Noir pour les obstacles, blanc pour les espaces libres, gris pour les inconnus
colors = [(1, 1, 1), (0.5, 0.5, 0.5), (0, 0, 0)]  # R -> G -> B
n_bins = [3]  
cmap_name = 'map'
cm = mcolors.LinearSegmentedColormap.from_list(cmap_name, colors, N=3)

# Afficher la matrice d'occupation comme une image
plt.imshow(normalized_grid, cmap=cm, origin='lower')
plt.colorbar()
plt.title('Occupancy Grid Map')

# Sauvegarder l'image de la carte
plt.savefig('map_image2.png')
print('Map sauvergardée')

# Afficher l'image
plt.show()
