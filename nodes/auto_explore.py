#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import time
import math

class RobotExplorer(Node):
    def __init__(self):
        super().__init__('robot_explorer')
        
        # Publisher pour contrôler le mouvement du robot
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Subscriber pour recevoir les données du scanner laser
        self.laser_sub = self.create_subscription(LaserScan, '/scan', self.laser_callback, 10)
        
        # Variables d'état
        self.obstacle_threshold = 0.5  # Distance minimale (en mètres) avant de considérer comme obstacle
        self.exploring = True
        self.laser_data = None
        self.state = 'forward'  # États: 'forward', 'turning', 'rotating'
        self.turn_direction = 1  # 1 pour gauche, -1 pour droite
        self.timer_period = 0.1  # En secondes
        self.timer = self.create_timer(self.timer_period, self.exploration_loop)
        self.rotation_start_time = None
        self.rotation_duration = 0
        
        self.get_logger().info('Robot Explorer a démarré')
    
    def laser_callback(self, msg):
        self.laser_data = msg
    
    def is_obstacle_ahead(self):
        if self.laser_data is None:
            return True  # Par sécurité, on considère qu'il y a un obstacle si pas de données
        
        # Prendre les mesures frontales (devant le robot)
        front_angle_range = 30  # degrés
        ranges = np.array(self.laser_data.ranges)
        
        # Convertir les degrés en indices
        angle_increment = self.laser_data.angle_increment
        front_indices_range = int(front_angle_range * (math.pi/180) / angle_increment)
        
        # Obtenir l'indice du milieu (mesure directement devant)
        middle_idx = len(ranges) // 2
        
        # Définir la plage des indices pour les mesures frontales
        start_idx = middle_idx - front_indices_range // 2
        end_idx = middle_idx + front_indices_range // 2
        
        # S'assurer que les indices sont dans les limites
        start_idx = max(0, start_idx)
        end_idx = min(len(ranges) - 1, end_idx)
        
        # Extraire les mesures frontales
        front_ranges = ranges[start_idx:end_idx]
        
        # Filtrer les valeurs inf ou nan
        valid_ranges = [r for r in front_ranges if not math.isinf(r) and not math.isnan(r)]
        
        if not valid_ranges:
            return True  # Si pas de mesures valides, considérer comme obstacle
        
        # Vérifier si la distance minimale est inférieure au seuil
        min_distance = min(valid_ranges)
        return min_distance < self.obstacle_threshold
    
    def find_best_direction(self):
        if self.laser_data is None:
            return 1  # Par défaut à gauche
        
        ranges = np.array(self.laser_data.ranges)
        
        # Diviser en sections gauche et droite
        left_section = ranges[:len(ranges)//2]
        right_section = ranges[len(ranges)//2:]
        
        # Filtrer les valeurs invalides
        valid_left = [r for r in left_section if not math.isinf(r) and not math.isnan(r)]
        valid_right = [r for r in right_section if not math.isinf(r) and not math.isnan(r)]
        
        # Calculer les moyennes (ou utiliser une valeur par défaut si pas de données valides)
        avg_left = sum(valid_left) / len(valid_left) if valid_left else 0
        avg_right = sum(valid_right) / len(valid_right) if valid_right else 0
        
        # Retourner la direction avec le plus d'espace
        return 1 if avg_left > avg_right else -1
    
    def exploration_loop(self):
        if not self.exploring:
            return
        
        cmd = Twist()
        
        if self.state == 'forward':
            if self.is_obstacle_ahead():
                # Obstacle détecté, passer en mode rotation
                self.state = 'rotating'
                self.turn_direction = self.find_best_direction()
                self.rotation_start_time = time.time()
                # Durée de rotation aléatoire entre 2 et 4 secondes
                self.rotation_duration = 1.0 + np.random.random() * 2.0
                self.get_logger().info(f'Obstacle détecté. Rotation {"gauche" if self.turn_direction == 1 else "droite"}')
            else:
                # Avancer
                cmd.linear.x = 0.3  # Vitesse linéaire modérée
                cmd.angular.z = 0.0
        
        elif self.state == 'rotating':
            # Rotation sur place
            cmd.linear.x = 0.0
            cmd.angular.z = 0.5 * self.turn_direction  # Vitesse angulaire
            
            # Vérifier si la rotation est terminée
            if time.time() - self.rotation_start_time > self.rotation_duration:
                self.state = 'forward'
                self.get_logger().info('Reprise de l\'exploration en avant')
        
        # Publier la commande de mouvement
        self.cmd_vel_pub.publish(cmd)
    
    def stop_exploration(self):
        self.exploring = False
        # Arrêter le robot
        cmd = Twist()
        self.cmd_vel_pub.publish(cmd)
        self.get_logger().info('Exploration arrêtée')

def main(args=None):
    rclpy.init(args=args)
    explorer = RobotExplorer()
    
    try:
        rclpy.spin(explorer)
    except KeyboardInterrupt:
        explorer.stop_exploration()
        explorer.get_logger().info('Exploration interrompue par l\'utilisateur')
    finally:
        # Nettoyage
        explorer.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()