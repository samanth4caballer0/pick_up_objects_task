#!/usr/bin/env python

import rospy
import numpy as np
import rospkg
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerRequest
import py_trees
import time
import math
import tf
from geometry_msgs.msg import Twist
from mybehaviors import FollowPath, CheckObject, GetObject, LetObject, PlanPath
from nav_msgs.msg import Odometry


# TODO: Create any other required behavior like those to move the robot to a point,
#       add or check elements in the blackboard, ...
def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

class MoveToGoal(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(MoveToGoal, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("goal", access=py_trees.common.Access.READ)
        self.distance_threshold = 0.2
        self.current_pose = [0,0]
        self.last_pose = [0,0]

    def setup(self):
        self.logger.debug("  %s [MoveToGoal::setup()]" % self.name)
        self.goal_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
        self.odom_sub = rospy.Subscriber("/turtlebot/kobuki/odom_ground_truth", Odometry, self.get_odom)
        self.goal_sent = False
        self.blackboard.goal = [0,0]

    def initialise(self):
        self.logger.debug("  %s [MoveToGoal::initialise()]" % self.name)
        self.star_time = time.time()

    def update(self):
        # Send the command to turtlebot
        #could be because the tree is wrong and doesnt actually drop the ball, 
        # or it drops it but then it doesnt know it should go to the next pickup spot
        self.logger.debug("  %s [FindPickupSpot::update()]" % self.name)
        moved_dis = np.linalg.norm(np.array(self.current_pose) - np.array(self.last_pose))
        rospy.logerr("MOVE DIS: %s" % moved_dis)
        idle_time = time.time() - self.star_time
        if not self.goal_sent or (moved_dis < 0.001 and idle_time > 5) : #0.001
            rospy.logerr("SEND GOAL")
            goal = PoseStamped()
            goal.pose.position.x = self.blackboard.goal[0]
            goal.pose.position.y = self.blackboard.goal[1]
            
            self.goal_pub.publish(goal)
            self.goal_sent = True
            self.star_time = time.time()
            time.sleep(2)

        distance_to_goal = np.linalg.norm(np.array(self.blackboard.goal) - np.array(self.current_pose))
        # print(self.current_pose)
        rospy.logerr("DIS: %s" % distance_to_goal)

        if distance_to_goal > self.distance_threshold:
            self.last_pose = self.current_pose
            return py_trees.common.Status.RUNNING
        else:
            # self.goal_sent = False
            rospy.loginfo("Goal not reached!: [%.2f, %.2f]", 
                        self.current_pose[0], self.current_pose[1])
            rospy.loginfo("Current blackboard goal was: [%.2f, %.2f]", 
                        self.blackboard.goal[0], self.blackboard.goal[1])            
            time.sleep(1)
            return py_trees.common.Status.SUCCESS
        
            
    def terminate(self, new_status):
        self.logger.debug("  %s [MoveToGoal::terminate().terminate()][%s->%s]" %
                          (self.name, self.status, new_status))
    
    def get_odom(self, odom):
        self.current_pose = [odom.pose.pose.position.x, odom.pose.pose.position.y]

# Funtion to determine the pick up spot according to blackboard variables
class FindPickupSpot(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(FindPickupSpot, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.blackboard.register_key("current_pickup_index", access=py_trees.common.Access.WRITE)
        
        # Fixed pickup positions in order
        self.pickup_positions = [
            [1.65, 0.7],
            [0.7, 2.20],# First point
            [2.21, 2.90], # ,,,,,2.17, 2.84, segundo  2.25, 2.95 Second point change in y directoin either plus or minus 
               # Third point
        ]
        
    def setup(self):
        self.logger.debug("  %s [FindPickupSpot::setup()]" % self.name)
        self.blackboard.current_pickup_index = 0  # Start with first point
        
    def update(self):
        self.logger.debug("  %s [FindPickupSpot::update()]" % self.name)
        
        # Check if we've visited all points
        if self.blackboard.current_pickup_index >= len(self.pickup_positions):
            rospy.loginfo("All pickup positions visited. No more points to visit. Index: %d", 
                        self.blackboard.current_pickup_index)
            return py_trees.common.Status.FAILURE  # No more points to visit
                
        # Set goal to current pickup point
        current_pos = self.pickup_positions[self.blackboard.current_pickup_index]
        rospy.loginfo("Setting goal to pickup position %d: [%.2f, %.2f]", 
                    self.blackboard.current_pickup_index, current_pos[0], current_pos[1])
        
        self.blackboard.goal = current_pos
        
        # Log increment
        rospy.loginfo("Incrementing pickup index from %d to %d", 
                    self.blackboard.current_pickup_index, 
                    self.blackboard.current_pickup_index + 1)
        
        self.blackboard.current_pickup_index += 1
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        self.logger.debug("  %s [FindPickupSpot::terminate()][%s->%s]" % 
                        (self.name, self.status, new_status))


# Funtion to determine the drop spot according to blackboard variables
class FindDropSpot(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(FindDropSpot, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        
        # Fixed drop position
        self.drop_position = [-0.20, 3.2] #0.35, 3.2 [0.20, 3.5]
        
    def update(self):
        self.logger.debug("  %s [FindDropSpot::update()]" % self.name)
        self.blackboard.goal = self.drop_position
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        self.logger.debug("  %s [FindDropSpot::terminate()][%s->%s]" % 
                        (self.name, self.status, new_status))


class TurnTowardsBall(py_trees.behaviour.Behaviour):
    """
    behavior to make the robot turn to face ball position.
    """
    def __init__(self, name, ball_index=0):
        super(TurnTowardsBall, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("current_pickup_index", access=py_trees.common.Access.READ)
        
        # Ball positions
        self.ball_positions = [
            [1.4, 0.7],    # Ball 1
            [0.7, 2.5],   # Ball 2 
            [2.4, 3.15]   # Ball 3
        ]
        
        self.ball_index = ball_index
        self.current_pose = None
        self.angle_threshold = 0.1  # Radians (~5.7 degrees)
        self.max_angular_vel = 0.8  # Max rotation speed
        self.Kw = 0.5               # Angular velocity gain
        
    def setup(self):
        self.logger.debug("%s [TurnTowardsBall::setup()]" % self.name)
        self.cmd_vel_pub = rospy.Publisher('/turtlebot/kobuki/commands/velocity', Twist, queue_size=1)
        self.odom_sub = rospy.Subscriber("/turtlebot/kobuki/odom_ground_truth", Odometry, self.get_odom)
    
    def update(self):
            self.ball_index  = self.blackboard.current_pickup_index - 1
            # Get target ball position
            ball_position = self.ball_positions[self.ball_index]
            rospy.loginfo("Current ball index: %d", self.ball_index)
            # Calculate angle to ball
            dx = ball_position[0] - self.current_pose[0]
            dy = ball_position[1] - self.current_pose[1]
            target_angle = math.atan2(dy, dx)
            current_angle = self.current_pose[2]
            
            # Calculate angle difference
            angle_diff = wrap_angle(target_angle - current_angle)
            rospy.loginfo("Current angle: %.2f, Target angle: %.2f, Difference: %.2f", 
                         current_angle, target_angle, angle_diff)
            
            # Check if angle difference is within threshold
            if abs(angle_diff) < self.angle_threshold:
                rospy.loginfo("Angle difference within threshold. Stopping rotation.")
                return py_trees.common.Status.SUCCESS
            else:
                # Calculate angular velocity
                angular_vel = -(angle_diff/abs(angle_diff) )* 1.0
                
                # Create and publish command
                cmd = Twist()
                cmd.angular.z = angular_vel
                self.cmd_vel_pub.publish(cmd)
                
                rospy.loginfo("Rotating towards ball: Angular velocity: %.2f", angular_vel)
                return py_trees.common.Status.RUNNING
            
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        x = odom.pose.pose.position.x 
        y = odom.pose.pose.position.y
        
        self.current_pose = np.array([x,y,yaw])
            
        
class Done(py_trees.behaviour.Behaviour):

    def __init__(self, name):
        super(Done, self).__init__(name)
        self.blackboard = self.attach_blackboard_client(name=self.name)
        self.blackboard.register_key("current_pickup_index", access=py_trees.common.Access.READ)
        self.blackboard.register_key("current_pickup_index", access=py_trees.common.Access.WRITE)
        self.blackboard.current_pickup_index = 0  # Start with first point
        
    def update(self):
        print("Current pickup index: {}".format(self.blackboard.current_pickup_index))
        if self.blackboard.current_pickup_index >= 3:  # After 3 pickups
            print("Finished all pickups")
            return py_trees.common.Status.FAILURE  # Terminate tree
        return py_trees.common.Status.SUCCESS  # Continue


if __name__ == "__main__":
    py_trees.logging.level = py_trees.logging.Level.DEBUG
    rospy.init_node("behavior_trees")

    # Create Behaviors
   
    check_object = CheckObject("check_object")
    get_object = GetObject("get_object")
    let_object = LetObject("let_object")
    move_to_pickup = MoveToGoal("move_to_pickup")
    move_to_drop = MoveToGoal("move_to_drop")
    find_pickup_spot = FindPickupSpot("find_pickup_spot")
    find_drop_spot = FindDropSpot("find_drop_spot")
    is_done = Done("done")
    turn_to_ball = TurnTowardsBall("turn_to_ball")
    
    # create tree, define root and add behaviors
    root = py_trees.composites.Sequence(name="Sequence", memory=True)
    root.add_children([ is_done,
                       find_pickup_spot,
                       move_to_pickup,
                       turn_to_ball,
                       check_object,
                        get_object,
                        find_drop_spot,
                        move_to_drop,
                        let_object,
                        ])
    
    behavior_tree = py_trees.trees.BehaviourTree(root=root)
    # call setup method of all tree behaviors
    behavior_tree.setup(timeout=15)

 
    # save tree as image
    rospack = rospkg.RosPack()
    filepath = rospack.get_path("pick_up_objects_task")
    py_trees.display.render_dot_tree(root)

    # manual path because im lazy but you are not (O__O)
    # plan_path.waypoints=[[3.0,-0.78],[3.0,0.7],[1.5,0.7]]

    # tick the tree
    try:
        while not rospy.is_shutdown():
            # if behavior_tree.root.status!=py_trees.common.Status.SUCCESS:
            behavior_tree.tick()
            time.sleep(0.5)
            # else:
            #     print("root returned success, tree done")
            #     break
        print("\n")
    except KeyboardInterrupt:
        print("")
        pass