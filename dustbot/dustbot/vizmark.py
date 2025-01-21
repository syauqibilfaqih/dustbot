import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from dustbot_interface.msg import Position
import numpy as np
import time

class VisualizationDustbot(Node):
    def __init__(self):
        # initialize node
        time.sleep(8)
        super().__init__('visualization_dustbot')

        # publishers
        self.grid_map_pub = self.create_publisher(OccupancyGrid, 'occupancy_grid', 10)
        self.robot_marker_pub = self.create_publisher(Marker, 'robot_marker', 10)
        self.garbage_marker_pub = self.create_publisher(Marker, 'garbage_marker',10)

        # declare and get parameter values
        self.declare_parameter('grid_size',10)
        self.grid_size= self.get_parameter('grid_size').get_parameter_value().integer_value

        # Subscribers declaration
        self.global_position_subscriber = self.create_subscription(Position,'/dustbot/global_position', self.robot_callback,10)
        self.garbage_position_subscriber = self.create_subscription(Position,'/dustbot/garbage_position', self.garbage_callback,10)

        # variables from dustbot interface
        self.robot = Position()
        self.garbage = Position()

        # message containers
        self.grid_map_msg = OccupancyGrid()
        self.robot_marker_msg = Marker()
        self.garbage_marker_msg = Marker()

        self.occupancy_grid = np.zeros((self.grid_size, self.grid_size), dtype=np.int8)
        self.set_map()
        self.set_robot()
        self.set_garbage()

        self.grid_map_pub.publish(self.grid_map_msg)
        self.robot_marker_pub.publish(self.robot_marker_msg)
        self.garbage_marker_pub.publish(self.garbage_marker_msg)

    def robot_callback(self,msg):
        self.robot.x = msg.x
        self.robot.y = msg.y
        self.set_robot()
        self.robot_marker_pub.publish(self.robot_marker_msg)

    def garbage_callback(self,msg):
        self.garbage.x = msg.x
        self.garbage.y = msg.y
        self.set_garbage()
        self.garbage_marker_pub.publish(self.garbage_marker_msg)

    def set_map(self):
        self.grid_map_msg.header.frame_id = 'map'
        self.grid_map_msg.info.resolution = 1.0
        self.grid_map_msg.info.width = self.grid_size 
        self.grid_map_msg.info.height = self.grid_size 
        self.grid_map_msg.info.origin.position.x = 0.0
        self.grid_map_msg.info.origin.position.y = 0.0
        self.grid_map_msg.info.origin.position.z = 0.0
        self.grid_map_msg.info.origin.orientation.w = 1.0
        self.grid_map_msg.data = self.occupancy_grid.flatten().tolist()

    def set_robot(self):
        self.robot_marker_msg.header.frame_id = "map"
        self.robot_marker_msg.header.stamp = self.get_clock().now().to_msg()
        self.robot_marker_msg.ns = "my_namespace"
        self.robot_marker_msg.id = 0
        self.robot_marker_msg.type = self.robot_marker_msg.CUBE
        self.robot_marker_msg.action = self.robot_marker_msg.ADD
        self.robot_marker_msg.pose.position.x = 0.5 + self.robot.y # this is y
        self.robot_marker_msg.pose.position.y = 0.5 + self.robot.x # this is x
        self.robot_marker_msg.pose.position.z = 0.0
        self.robot_marker_msg.pose.orientation.x = 0.0
        self.robot_marker_msg.pose.orientation.y = 0.0
        self.robot_marker_msg.pose.orientation.z = 0.0
        self.robot_marker_msg.pose.orientation.w = 1.0
        self.robot_marker_msg.scale.x = 0.5
        self.robot_marker_msg.scale.y = 0.5
        self.robot_marker_msg.scale.z = 0.5
        self.robot_marker_msg.color.a = 1.0  # Don't forget to set the alpha!
        self.robot_marker_msg.color.r = 0.0
        self.robot_marker_msg.color.g = 1.0
        self.robot_marker_msg.color.b = 0.0

    def set_garbage(self):
        self.garbage_marker_msg.header.frame_id = "map"
        self.garbage_marker_msg.header.stamp = self.get_clock().now().to_msg()
        self.garbage_marker_msg.ns = "garbage"
        self.garbage_marker_msg.id = 0
        self.garbage_marker_msg.type = self.garbage_marker_msg.SPHERE
        self.garbage_marker_msg.action = self.garbage_marker_msg.ADD
        self.garbage_marker_msg.pose.position.x = 0.5 + self.garbage.y # this is y
        self.garbage_marker_msg.pose.position.y = 0.5 + self.garbage.x # this is x
        self.garbage_marker_msg.pose.position.z = 0.0
        self.garbage_marker_msg.pose.orientation.x = 0.0
        self.garbage_marker_msg.pose.orientation.y = 0.0
        self.garbage_marker_msg.pose.orientation.z = 0.0
        self.garbage_marker_msg.pose.orientation.w = 1.0
        self.garbage_marker_msg.scale.x = 0.2
        self.garbage_marker_msg.scale.y = 0.2
        self.garbage_marker_msg.scale.z = 0.2
        self.garbage_marker_msg.color.a = 1.0  # Don't forget to set the alpha!
        self.garbage_marker_msg.color.r = 1.0
        self.garbage_marker_msg.color.g = 0.0
        self.garbage_marker_msg.color.b = 0.0
        

def main(args=None):
    rclpy.init(args=args)
    vizmark = VisualizationDustbot()
    rclpy.spin(vizmark)
    vizmark.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()