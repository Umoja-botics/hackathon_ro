#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import os
import cv2

class MapListener(Node):
    def __init__(self):
        super().__init__('map_listener')
        self.subscription = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)
        self.map_data = None

    def map_callback(self, msg):
        # Convertir la liste en matrice 2D
        width = msg.info.width
        height = msg.info.height
        self.map_data = np.array(msg.data).reshape(height, width)
        self.get_logger().info('Map data received and converted to matrix.')

        # Sauvegarder la matrice dans un fichier
        save_path = 'map_matrix.npy'
        np.save(save_path, self.map_data)
        self.get_logger().info(f'Map matrix saved to {save_path}')

    def save_map_image(self):
        if self.map_data is not None:
            # Normaliser les données pour l'affichage de l'image
            # Les valeurs connues varient de 0 à 100, et -1 pour les inconnues
            # Nous allons convertir -1 en 50 pour les visualiser en gris
            self.get_logger().info('start the creation of the map image.')
            image_data = np.copy(self.map_data)
            image_data[image_data == -1] = 50  # Inconnu
            image_data = (image_data - image_data.min()) / (image_data.max() - image_data.min()) * 255.0
            image_data = image_data.astype(np.uint8)

            # Sauvegarder l'image en utilisant OpenCV
            image_path = 'map_image.png'
            cv2.imwrite(image_path, image_data)
            self.get_logger().info(f'Map image saved to {image_path}')

def main(args=None):
    rclpy.init(args=args)
    map_listener = MapListener()

    print("Listening for the map on the 'map' topic...")
    map_listener.save_map_image()
    rclpy.spin(map_listener)

    # Après la fin de la boucle de spin, enregistrez l'image de la carte
    #map_listener.save_map_image()

    # Clean up
    map_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
