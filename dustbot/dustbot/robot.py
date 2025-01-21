import rclpy
import time
from rclpy.node import Node
from dustbot_interface.msg import Position
from dustbot_interface.srv import Direction
from dustbot_interface.srv import Load


class Robot(Node):

    def __init__(self):
        # Node initialization
        super().__init__('robot')
        self.get_logger().info("The robot node was started!")
        time.sleep(8)

        # Subscribers declaration
        self.global_position_subscriber = self.create_subscription(Position,'/dustbot/global_position', self.robot_callback,10)
        self.garbage_position_subscriber = self.create_subscription(Position,'/dustbot/garbage_position', self.garbage_callback,10)

        # Clients declarations
        self.robot_direction_client = self.create_client(Direction, '/dustbot/set_direction')
        while not self.robot_direction_client.wait_for_service(timeout_sec=1.0):
            pass
        self.garbage_loader_client = self.create_client(Load, '/dustbot/load_garbage')
        while not self.garbage_loader_client.wait_for_service(timeout_sec=1.0):
            pass

        # messages and service container defintions
        self.robot = Position()
        self.garbage = Position()
        self.robot_dir_req = Direction.Request()
        self.garbage_load_req = Load.Request()
        self.finish_resp = Load.Response()

        # public variables
        self.robot_distance_on_x = 5 
        self.robot_distance_on_y = 5

    def send_direction_request(self):
        return self.robot_direction_client.call_async(self.robot_dir_req)
    
    def send_load_request(self):
        return self.garbage_loader_client.call_async(self.garbage_load_req) 


    def robot_callback(self,msg):
        self.robot.x = msg.x
        self.robot.y = msg.y

    def garbage_callback(self,msg):
        self.garbage.x = msg.x
        self.garbage.y = msg.y

        self.robot_distance_on_x = self.garbage.x - self.robot.x
        self.robot_distance_on_y = self.garbage.y - self.robot.y

        self.robot_dir_req.w = 1 if self.robot_distance_on_x < 0 else 0
        self.robot_dir_req.e = 1 if self.robot_distance_on_x > 0 else 0
        self.robot_dir_req.n = 1 if self.robot_distance_on_y < 0 else 0
        self.robot_dir_req.s = 1 if self.robot_distance_on_y > 0 else 0

def main(args=None):
    rclpy.init(args=args)

    robot = Robot()
    
    while robot.finish_resp.finished == False:
        if robot.robot_distance_on_x == 0 and robot.robot_distance_on_y == 0:
            future = robot.send_load_request()
            rclpy.spin_until_future_complete(robot,future)
            robot.finish_resp = future.result()
        else:
            future = robot.send_direction_request()
            rclpy.spin_until_future_complete(robot,future)
            move_response = future.result()
            robot.get_logger().info("Is robot moved? "+str(move_response.moved))

    robot.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()