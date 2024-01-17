#!/usr/bin/env python3

import numpy as np
import math
import matplotlib.pyplot as plt



def traj_lineaire(A,B, N):
    t_step = 1 / (N + 1)
   
    line = []

    for i in range(1, N+1):
        t = i * t_step
        x = A[0] + t * (B[0] - A[0])
        y = A[1] + t * (B[1] - A[1])
        line.append((x,y))
      
    
    return line



def create_cubic_bezier_turn(p0, p3, d, num_points):
    """
    Create a curvilinear turn using a cubic Bezier curve defined by four control points.
    """
    
    # calcule des deux points de control
    p1 = (p0[0] +d, p0[1] + d )
    p2 = (p3[0] +d , p3[1] + d )


    # t values from 0 to 1
    t = np.linspace(0, 1, num_points)
    
    
    # Calculate the Bezier curve points using the cubic formula
    bezier_points = [((1 - t_val)**3 * p0[0] +
                      3 * (1 - t_val)**2 * t_val * p1[0] +
                      3 * (1 - t_val) * t_val**2 * p2[0] +
                      t_val**3 * p3[0],
                      (1 - t_val)**3 * p0[1] +
                      3 * (1 - t_val)**2 * t_val * p1[1] +
                      3 * (1 - t_val) * t_val**2 * p2[1] +
                      t_val**3 * p3[1]) for t_val in t]
    
  
    
    return bezier_points

# def gen_path (W , N, R):
#     """Fonction qui genere une trajectoire pour un champs w
#     W: liste des waypoints
#     N: nombres de points intermediaire pour ligne droite
#     R: Nombre de points intermediaire pour virage    
#     cette fonction retourne une liste de tuple qui constitue les points à suivre
#     """
#     Path_f = []

#     for i in range(len(W)-2):
#         path = traj_lineaire(way[i],way[i+1], N)
#         Path_f.extend(path)
#         path = create_cubic_bezier_turn(way[i+1],  way[i+2], -10, R)
#         Path_f.extend(path)

#     return Path_f



    # path = traj_lineaire(way[0],way[1], N)
    # Path_f.extend(path)
    # path = create_cubic_bezier_turn(way[1],  way[2], -10, num_intermediate_points)
    # Path_f.extend(path)
    # path = traj_lineaire(way[2],way[3], N)
    # Path_f.extend(path)




def plot_points( x_points, y_points):

    
    # Tracer les points intermédiaires
    plt.scatter(x_points, y_points, color='red', label="Points intermédiaires")
    
    
    plt.legend()
    plt.grid(True)
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.title("Points intermédiaires sur un segment de ligne")
    plt.show()

way = [(127.0077505607697, 121.76708463601554),
 (112.97543750454116, 116.46829548282422),
 (115.09374086915928, 110.85580329891741),
 (129.1260539253878, 116.15459245210873)]

#paths = gen_path(way,15,10)

N = 15  # Nombre de points intermédiaires en ligne droite


# Nombres de points intermediaire pour le virage
num_intermediate_points = 10

Path_f = []


path = traj_lineaire(way[0],way[1], N)
Path_f.extend(path)
path = create_cubic_bezier_turn(way[1],  way[2], -10, num_intermediate_points)
Path_f.extend(path)
path = traj_lineaire(way[2],way[3], N)
Path_f.extend(path)

# print(Path_f)

x_coords, y_coords = zip(*Path_f)

plt.scatter(x_coords, y_coords, color='red', label="Points intermédiaires")
    
    
plt.legend()
plt.grid(True)
plt.xlabel("X")
plt.ylabel("Y")
plt.title("Points intermédiaires sur un segment de ligne")
plt.show()

