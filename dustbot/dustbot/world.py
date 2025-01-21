import rclpy
from rclpy.node import Node
from rclpy.task import Future
from dustbot_interface.msg import Position
from dustbot_interface.srv import Direction
from dustbot_interface.srv import Load
import random
import time


class World(Node):

    def __init__(self):
        # Node initialization
        super().__init__('world')

        # giving information that this node is starting
        self.get_logger().info("The world node was started!")
        self.get_logger().info("Waiting before starting the whole process...")
        time.sleep(8)

        # declare and get parameter values
        self.declare_parameter('grid_size',10)
        self.declare_parameter('picks_up', 5)
        self.grid_size= self.get_parameter('grid_size').get_parameter_value().integer_value
        self.picks_up= self.get_parameter('picks_up').get_parameter_value().integer_value

        # give information of grid size and number of picks up to user
        self.get_logger().info("The grid size is: "+str(self.grid_size))
        self.get_logger().info("The number of pick up is: "+str(self.picks_up))

        # declaring topics
        self.global_position_publisher = self.create_publisher(Position, '/dustbot/global_position', 10)
        self.garbage_position_publisher = self.create_publisher(Position, '/dustbot/garbage_position', 10)

        # generate initial position for the robot
        self.robot = Position()
        self.robot.x = 0
        self.robot.y = 0

        # generate initial position for the garbage
        self.garbage = Position()
        self.garbage.x = 0
        self.garbage.y = 0

        # iteration number, constrained by pick up number
        self.i = 0

        # calling garbage position updater
        self.update_garbage_pos()

        # declaring service servers
        self.direction_server = self.create_service(Direction, '/dustbot/set_direction',self.move_robot_callback)
        self.loader_server = self.create_service(Load,'/dustbot/load_garbage', self.loaded_garbage_callback)


    def update_garbage_pos(self):
        # show to logger the number of pick up currently
        self.get_logger().info("Number of picks-up: " + str(self.i+1) + " out of " + str(self.picks_up))

        # generate random garbage position
        self.garbage = Position()
        self.garbage.x = random.randrange(0,self.grid_size)
        self.garbage.y = random.randrange(0,self.grid_size)
        self.get_logger().info("The position of garbage: " + str(self.garbage))
        self.get_logger().info("The position of robot : "+str(self.robot))

        # publish robot and garbage position
        self.global_position_publisher.publish(self.robot)
        self.garbage_position_publisher.publish(self.garbage)
    
    def move_robot_callback(self, request, response):
        # move robot based on the direction
        self.robot.x = self.robot.x + (request.e - request.w)
        self.robot.y = self.robot.y + (request.s - request.n)
        response.moved = True

        # show info on logger
        self.get_logger().info("The incoming direction: ")
        self.get_logger().info("n: "+str(request.n))
        self.get_logger().info("w: "+str(request.w))
        self.get_logger().info("s: "+str(request.s))
        self.get_logger().info("e: "+str(request.e))
        self.get_logger().info("The new robot location: ")
        self.get_logger().info("x: "+str(self.robot.x))
        self.get_logger().info("y: "+str(self.robot.y))
        self.get_logger().info("The garbage location: ")
        self.get_logger().info("x: "+str(self.garbage.x))
        self.get_logger().info("y: "+str(self.garbage.y))

        time.sleep(1)

        # publish robot and garbage new positions
        self.global_position_publisher.publish(self.robot)
        self.garbage_position_publisher.publish(self.garbage)

        return response
    
    def loaded_garbage_callback(self, _request, response):
        self.i = self.i+1

        # conditions if the garbage was picked up but the iteration is still below the number of picks up
        if self.i<self.picks_up:
            response.finished = False
            self.get_logger().info("The garbage has been taken!")
            self.update_garbage_pos()
        else:
            response.finished = True
            self.get_logger().info("All garbages has been taken!")
            
        return response

def main(args=None):
    rclpy.init(args=args)

    world = World()

    # spin the node as long as the iteration bellow the number of pick up
    while world.i<world.picks_up:
        rclpy.spin_once(world)

    # destroy the node and shutdown the process after task was done
    world.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()