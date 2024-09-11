import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import std_msgs.msg
from tf2_ros import TransformBroadcaster
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header
from geometry_msgs.msg import TransformStamped, Twist


from tf_transformations import euler_from_quaternion

import math
import numpy as np
    

class PotentialFieldPathPlaning(Node):
    def __init__(self):
        super().__init__('prikaz_polja')
        self.ROBOT_RADIUS = 0.5     # radijus opisane kruznice robota                   [m]
        self.KP = 0.5               # faktor pojacanja privlacnog polja                 [/]
        self.KO = 0.02                 # faktor pojacanja odbojnog polja                   [/]
        self.RESOLUTION = 0.01      # rezolucija prosotra                               [m]
        self.LINEAR_BORDER = 0.5    # granica izvan koje je privlacna sila linearna     [m]

        self.map_minimum_x = -4.533999919891357     # minimalna x-koordianta radnog prostora            [m]
        self.map_minimum_y = -2.5369999408721924    # minimalna y-koordianta radnog prostora            [m]
        self.map_maximum_x = 5.165999889373779      # maksimalna x-koordianta radnog prostora           [m]
        self.map_maximum_y = 2.1440000534057617     # maksimalna y-koordianta radnog prostora           [m]

        self.start_x = 2.2         # start x lokacija  [m]
        self.start_y = 0.6          # start y lokacija  [m]
        self.goal_x = -3.0          # cilj x lokacija   [m]
        self.goal_y = -0.2          # cilj y lokacija   [m]

        self.obstacle_x = [0.8, 0.9, 1.0, 1.1, 1.2, 1.3, 1.4, 1.4, 1.4, 1.4, 1.4, -1.5, -1.5, -1.5, -1.5, -1.5, -1.6, -1.7, -1.8]            # x-koorindate tocki prepreka     [m]
        self.obstacle_y = [-0.2, -0.1, 0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, -0.2, -0.3, -0.4, -0.5, -0.6, -0.7, -0.8, -0.9]            # y-koorindate tocki prepreka     [m]

        # self.obstacle_x = [-0.1, 0.0, 0.1, -0.1, 0.0, 0.1, -0.1, 0.0, 0.1]            # x-koorindate tocki prepreka     [m]
        # self.obstacle_y = [-0.1, -0.1, -0.1, 0, 0, 0, 0.1, 0.1, 0.1 ]

        # self.obstacle_x = []            # x-koorindate tocki prepreka     [m]
        # self.obstacle_y = []

        self.potential_field_map = []                                 # mapa iznosa potencijala za svaku tocku radnog prostora
        self.calculated_path = []         # koorinate izracunate putanje od pocetne poicije do cilja
        
        self.publisher_potential_field_map = self.create_publisher(
            PointCloud2,
            '/potential_field_map',
            10)
        
        self.publisher_calculated_path = self.create_publisher(
            Path,
            '/calculated_path',
            10)

        self.publisher_goal_point = self.create_publisher(
            PointStamped,
            '/goal_point',
            10
        )

        self.create_timer(1, self.prikaz_polja)

    def prikaz_polja(self):
        self.potential_field_map = self.calculate_potential_field()
        self.calculated_path = self.calculate_path()


        cloud_points = []
        for i, row in enumerate(self.potential_field_map):
            for j, value in enumerate(row):
                i_n = i * self.RESOLUTION - abs(self.map_minimum_x)
                j_n = j * self.RESOLUTION - abs(self.map_minimum_y)

                cloud_points.append((i_n, j_n, value))
        header = std_msgs.msg.Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "world"
        cloud_msg = point_cloud2.create_cloud_xyz32(header, cloud_points)
        self.publisher_potential_field_map.publish(cloud_msg)

        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "world"
        for point in self.calculated_path:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = point[2]
            path_msg.poses.append(pose)
        self.publisher_calculated_path.publish(path_msg)

        point_msg = PointStamped()
        point_msg.header = path_msg.header
        point_msg.point.x = self.goal_x
        point_msg.point.y = self.goal_y
        point_msg.point.z = 0.0
        self.publisher_goal_point.publish(point_msg)

    def calculate_potential_field(self):  

            n_x = int(round((self.map_maximum_x - self.map_minimum_x) / self.RESOLUTION))   # broj potentiala u x-dimenziji
            n_y = int(round((self.map_maximum_y - self.map_minimum_y) / self.RESOLUTION))   # broj potentiala u y-dimenziji

            potential_field_map = [[0.0 for i in range(n_y)] for i in range(n_x)]    # x,y mapa potentiala u kojoj ce biti spremljenje vrijednosti potentiala za svaku tocku prostora

            for ix in range(n_x):
                x = ix * self.RESOLUTION - abs(self.map_minimum_x)

                for iy in range(n_y):
                    y = iy * self.RESOLUTION - abs(self.map_minimum_y)

                    p_attractive = self.calculate_attractive_potential(x, y)   # odredivanje iznosa privlacnog potentiala
                    p_repulsive = self.calculate_repulsive_potential(x, y) # odredivanje odbojnog potentiala
                    potential_of_point = p_attractive + p_repulsive     # odredivanje ukupnog potentiala u tocki porsotora 
                    potential_field_map[ix][iy] = potential_of_point
            
            return potential_field_map
    
    def calculate_path(self):
        distance_to_goal = np.hypot(self.start_x - self.goal_x, self.start_y - self.goal_y)
        ix = int(round((self.start_x + abs(self.map_minimum_x)) / self.RESOLUTION))
        iy = int(round((self.start_y + abs(self.map_minimum_y)) / self.RESOLUTION))
        self.calculated_path = [[self.start_x, self.start_y, self.potential_field_map[ix][iy]]]

        moving_opt = self.moving_options()

        while distance_to_goal >= self.RESOLUTION * 5:
            minimal_potential = float('inf')
            min_i_x, min_i_y = ix, iy

            for i, _ in enumerate(moving_opt):
                inx = int(ix + moving_opt[i][0])
                iny = int(iy + moving_opt[i][1])
                potential = self.potential_field_map[inx][iny]

                if potential < minimal_potential:
                    minimal_potential = potential
                    min_i_x = inx
                    min_i_y = iny
        
            ix = min_i_x
            iy = min_i_y
            point_x = ix * self.RESOLUTION - abs(self.map_minimum_x)
            point_y = iy * self.RESOLUTION - abs(self.map_minimum_y)

            distance_to_goal = np.hypot(self.goal_x - point_x, self.goal_y - point_y)

            self.calculated_path.append([point_x, point_y, minimal_potential])

        self.drive = True
        return self.calculated_path
    
    def calculate_attractive_potential(self, x, y,):
        point_to_goal = np.hypot(x - self.goal_x, y - self.goal_y)

        if point_to_goal > self.LINEAR_BORDER:    # izracun privlacnog potentiala u linearnom podrucju 
            ap = self.LINEAR_BORDER * self.KP * point_to_goal - 0.5 * self.KP * self.LINEAR_BORDER ** 2
            return ap
        
        else:                                                   # izracun privlacnog potentiala u kvadratnom podrucju
            ap = 0.5 * self.KP * point_to_goal ** 2 
            return ap

    def calculate_repulsive_potential(self, x, y):
        minimum_distance_to_obstacle = float("inf")     # najbliza prepreka u beskonacnosti

        for i, _ in enumerate(self.obstacle_x):
            distance_to_obstacle = np.hypot(x - self.obstacle_x[i], y - self.obstacle_y[i])

            if distance_to_obstacle <= minimum_distance_to_obstacle:
                minimum_distance_to_obstacle = distance_to_obstacle 

        if minimum_distance_to_obstacle <= self.ROBOT_RADIUS:
            if minimum_distance_to_obstacle <= 0.1:
                minimum_distance_to_obstacle = 0.1

            return  0.5 * self.KO * (1.0 / minimum_distance_to_obstacle - 1.0 / self.ROBOT_RADIUS) ** 2   # izracun iznos obojnog potentiala za zonu djelovanja
        else:
            return 0.0                                                                                    # odbojni potential = 0 jer je van zone djelovanja

    def moving_options(self):
        # moguca gibanja od celije do celije, trenutna celija [0, 0]
        moving_opt = [[0, 1],
                [0, -1],
                [1, 0],
                [1, 1],
                [1, -1],
                [-1, 0],
                [-1, -1],
                [-1, 1]]

        return moving_opt


        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_path.append([self.robot_x, self.robot_y])

        angles = euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.robot_theta = angles[2]
        
        if self.drive:
            self.calculate_speed(True)
            robot_path_msg = Path()
            robot_path_msg.header.stamp = self.get_clock().now().to_msg()
            robot_path_msg.header.frame_id = "world"
            for point in self.robot_path:
                pose = PoseStamped()
                pose.header = robot_path_msg.header
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = 0.0
                robot_path_msg.poses.append(pose)
            self.publisher_robot_path.publish(robot_path_msg)


def main():
    rclpy.init()
    node = PotentialFieldPathPlaning()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
