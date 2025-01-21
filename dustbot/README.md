# Dustbot
This is a package of Dustbot, a mobile robot that navigates in a 2-dimensional NxN grid along the four directions (N-S-E-W) with a speed of one grid-cell/s. The main task of Dustbot is to picking garbages up.  

## Requirements
1. ROS 2 Humble
2. Ubuntu 22.04 
3. Python 3.10.12 (tested)

## Installation
To run this package, first, make the ros2 workspace directory if it doesn't exist yet:
```
mkdir -p ros2_ws/src
```
then add two packages, `dustbot` and `dustbot_interface` inside src folder. After that, build the packages
```
cd ~/ros2_ws
colcon build --packages-select dustbot
```

## Design
### Overview
In this package, there are two main nodes `world`, `robot`, and an additional node `vizmark` to visualize the robot. 
#### 1. `world`
This node act as a publisher, to publish the positions of garbage and robot, and as a service server to recive the direction of the robot.
#### 2. `robot`
This node act as a subscriber, to receive position data of garbage and robot, and as a service client to retrive the new direction of the robot, based on the distance between robot and  garbage, and to send the confirmation when the garbage is being loaded.
#### 3. `vizmark` (extra node)
This node will receive `grid_size` parameter and subscribe to the same topic to get garbage and process it to be visualized on RViz.

### Topics and Services
For the topics, there are several, which includes:
```
/dustbot/global_position
/dustbot/garbage_position
```
where both are used for communicating robot and garbage positions respectively. While for service, there are:
```
/dustbot/set_direction
/dustbot/load_garbage
```
which are used for communicating direction and the state of garbage retrieval. Moreover, there are additional topics:
```
occupancy_grid
robot_marker
garbage_marker
```
which are used simply for visualization purposes.

### Messages and Services
In the `dustbot_interface` package, there are several costumized services and messages to ease the communication of the nodes. These are including:
#### 1. Position (msg)
This is used to store the pose of the robot and garbage, which in this case only has two variables, for both x and y point:
```
int8 x
int8 y

```
#### 2. Direction (srv)
This service is used to alter the direction of the robot using the request data, which could be north, south, west, and east, with the value for each of them in the range fo 0 to 1, and if the opposite direction both get the same value, the resultant would be 0. The response that could be retrieved is a boolean to state that the robot really moved:
```
int8 n
int8 w
int8 s 
int8 e
---
bool moved
```
#### 3. Load (srv)
A simple service to indicate that the robot loaded the garbage, and to confirm if the pick up process is done completely:
```
bool loading
---
bool finished
```

### Methods
#### Process
In the beginning, user can input both grid size (NxN) and the number of picks up (P), otherwise, they will have the default values which are N=10 and P=5. `world` node is the main node where most of process occured, and this process will run as long as the iteration (i) is still below the number of picks up:
```python
while world.i<world.picks_up:
        rclpy.spin_once(world)
# destroy the node and shutdown the process after task was done
world.destroy_node()
rclpy.shutdown()

``` 
When robot already picked garbages up until the desired number, the `Load.finished` will be equal to 1:
```python
# from world.py
def loaded_garbage_callback(self,request, response):
        request
        self.i = self.i+1

        # conditions if the garbage was picked up but the iteration is still below the number of picks up
        if self.i<self.picks_up:
            response.finished = False
            self.get_logger().info("The garbage has been taken!")
            time.sleep(1)
            self.update_garbage_pos()
        else:
            response.finished = True
            self.get_logger().info("All garbages has been taken!")
``` 
so the `robot` node will also be stopped:
```python
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
```

#### Directing the Robot
The robot is driven by the acquisition of robot and garbage position, then calculate the distance of them, and determine the direction using this implementation:
```python
        self.robot_distance_on_x = self.garbage.x - self.robot.x
        self.robot_distance_on_y = self.garbage.y - self.robot.y

        self.robot_dir_req.w = 1 if self.robot_distance_on_x < 0 else 0
        self.robot_dir_req.e = 1 if self.robot_distance_on_x > 0 else 0
        self.robot_dir_req.n = 1 if self.robot_distance_on_y < 0 else 0
        self.robot_dir_req.s = 1 if self.robot_distance_on_y > 0 else 0
```

#### RViz Visualization
This part is an additional feature to ease the visualization of what happened in the program. The `vizmark` node gets the robot and garbage positions, then process and publish it to RViz. A little note about the process, there is transformation and a small shift equals to 0.5 to maintain the robot and garbage positions in the middle of the grid cells
```python
self.robot_marker_msg.pose.position.x = 0.5 + self.robot.y # this is y
self.robot_marker_msg.pose.position.y = 0.5 + self.robot.x # this is x
```

## How to Run?
### Regular Run
Source to setup.bash of the workspace:
```
source ~/ros2_ws/install/setup.bash
``` 
then the package can be launched using a single command:
```
ros2 launch dustbot dustbot_launch.xml N:=15 P:=5
```
Where N is the argument for the grid size and P is for the number of picks up.
### RViz Run
The process also can be visualized using RViz, by calling this command:
```
ros2 launch dustbot dustbot_rviz_launch.xml N:=5 P:=3
```

## Debugging
In case it is necessary to collect the data one by one from the nodes, it can be ran separately: 
```
ros2 run dustbot <node_name>
```
To manually call the service, this command can be used:
```
ros2 service call <service_name> <service_type> <values_uses_dictionary_like_type>
```
This is the example of the previous command line:
```
ros2 service call /dustbot/set_direction dustbot_interface/srv/Direction "{n: 0, w: 0, s: 1, e: 1}"
```
Example for loading garbage:
```
ros2 service call /dustbot/load_garbage dustbot_interface/srv/Load "{loading: True}"
```
Or it is also possible to publish a message manually:
```
ros2 topic pub <topic_name> <topic_type> <values_uses_dictionary_like_type>
```
For example:
```
ros2 topic pub /dustbot/garbage_position dustbot_interface/msg/Position '{x: 5, y: 5}'
```

