#!/usr/bin/env python

import rospy
import numpy as np
import rospkg
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger, TriggerRequest
import py_trees
import time
from mybehaviors import FollowPath, CheckObject, GetObject, LetObject, PlanPath
from nav_msgs.msg import Odometry


# TODO: Create any other required behavior like those to move the robot to a point,
#       add or check elements in the blackboard, ...

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

    def update(self):
        # Send the command to turtlebot
        self.logger.debug("  %s [FindPickupSpot::update()]" % self.name)
        moved_dis = np.linalg.norm(np.array(self.current_pose) - np.array(self.last_pose))
        rospy.logerr("MOVE DIS: %s" % moved_dis)

        if not self.goal_sent or moved_dis < 0.001:
            rospy.logerr("SEND GOAL")
            goal = PoseStamped()
            goal.pose.position.x = self.blackboard.goal[0]
            goal.pose.position.y = self.blackboard.goal[1]
            
            self.goal_pub.publish(goal)
            self.goal_sent = True
            time.sleep(2)

        distance_to_goal = np.linalg.norm(np.array(self.blackboard.goal) - np.array(self.current_pose))
        print(self.current_pose)
        rospy.logerr("DIS: %s" % distance_to_goal)

        if distance_to_goal > self.distance_threshold:
            self.last_pose = self.current_pose
            return py_trees.common.Status.RUNNING
        else:
            self.goal_sent = False
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
            [1.65, 0.7],  # First point
            [2.4, 3.10], # Second point change in y directoin either plus or minus 
            [0.7, 2.5]   # Third point
        ]
        
    def setup(self):
        self.logger.debug("  %s [FindPickupSpot::setup()]" % self.name)
        self.blackboard.current_pickup_index = 0  # Start with first point
        
    def update(self):
        self.logger.debug("  %s [FindPickupSpot::update()]" % self.name)
        
        # Check if we've visited all points
        if self.blackboard.current_pickup_index >= len(self.pickup_positions):
            return py_trees.common.Status.FAILURE  # No more points to visit
            
        # Set goal to current pickup point
        self.blackboard.goal = self.pickup_positions[self.blackboard.current_pickup_index]
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
        self.drop_position = [-0.5, 3.5]
        
    def update(self):
        self.logger.debug("  %s [FindDropSpot::update()]" % self.name)
        self.blackboard.goal = self.drop_position
        return py_trees.common.Status.SUCCESS
        
    def terminate(self, new_status):
        self.logger.debug("  %s [FindDropSpot::terminate()][%s->%s]" % 
                        (self.name, self.status, new_status))
                
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
    

    # def __init__(self, name):
    #     super(Done, self).__init__(name)
    #     self.blackboard = self.attach_blackboard_client(name=self.name)
    #     self.blackboard.register_key("pickup_visited", access=py_trees.common.Access.READ)
    #     self.blackboard.register_key("object_delivered", access=py_trees.common.Access.READ)
    #     self.blackboard.register_key("pickup_visited", access=py_trees.common.Access.WRITE)
    #     self.blackboard.register_key("object_delivered", access=py_trees.common.Access.WRITE)

    # def setup(self):
    #     self.logger.debug("  %s [Done::setup()]" % self.name)
        
    # def initialise(self):
    #     self.logger.debug("  %s [Done::initialise()]" % self.name)

    # def update(self):
    #     self.logger.debug("  %s [Done::update()]" % self.name)
        
    #     if self.blackboard.object_delivered == 3:
    #         rospy.logerr("Finised")
    #         return py_trees.common.Status.FAILURE
    #     else:
    #         return py_trees.common.Status.SUCCESS



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
    
    # create tree, define root and add behaviors
    root = py_trees.composites.Sequence(name="Sequence", memory=True)
    root.add_children([ is_done,
                       find_pickup_spot,
                       move_to_pickup,
                       check_object,
                        get_object,
                        find_drop_spot,
                        move_to_drop,
                        let_object,
                        ])
    
    behavior_tree = py_trees.trees.BehaviourTree(root=root)
    # call setup method of all tree behaviors
    behavior_tree.setup(timeout=15)

    # # go to pickup spot sequence
    # go_to_seq = py_trees.composites.Sequence(name="go_to_seq", memory=True)
    # go_to_seq.add_children([find_pickup_spot,move_to_pickup,check_object])
    # retry_pickup = py_trees.decorators.Retry(name="retry_pickup",child=go_to_seq, num_failures=6)

    # pick_place_seq = py_trees.composites.Sequence(name="Pick and Place Object", memory=True)
    # pick_place_seq.add_children([is_finished,retry_pickup,get_object,find_drop_spot,move_to_drop,let_object])
    # repeat_pick_place = py_trees.decorators.Repeat(name="repeat_pick_place",child=pick_place_seq,num_success=100)

    
    # save tree as image
    rospack = rospkg.RosPack()
    filepath = rospack.get_path("pick_up_objects_task")
    py_trees.display.render_dot_tree(root)
    

    # manual path because im lazy but you are not (O__O)
    # plan_path.waypoints=[[3.0,-0.78],[3.0,0.7],[1.5,0.7]]

    # tick the tree
    try:
        while not rospy.is_shutdown():
            if behavior_tree.root.status!=py_trees.common.Status.SUCCESS:
                behavior_tree.tick()
                time.sleep(0.5)
            else:
                print("root returned success, tree done")
                break
        print("\n")
    except KeyboardInterrupt:
        print("")
        pass
    
    
    
    
    
    # class FindPickupSpot(py_trees.behaviour.Behaviour):
    # def __init__(self, name):
    #     super(FindPickupSpot, self).__init__(name)
    #     self.blackboard = self.attach_blackboard_client(name=self.name)
    #     self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
    #     self.blackboard.register_key("pickup_positions", access=py_trees.common.Access.WRITE)
    #     self.blackboard.register_key("pickup_positions", access=py_trees.common.Access.READ)

    #     self.current_pose = [0,0]
        
    # def setup(self):
    #     self.logger.debug("  %s [FindPickupSpot::setup()]" % self.name)
    #     self.odom_sub = rospy.Subscriber("/odom", Odometry, self.get_odom)
    #     self.blackboard.pickup_positions = [[1.4,0.7],[2.4, 3.15],[0.7,2.5]]
    #     self.blackboard.pickup_visited = [False,False,False]
        
    # def initialise(self):
    #     self.logger.debug("  %s [FindPickupSpot::initialise()]" % self.name)

    # def update(self):
    #     self.logger.debug("  %s [FindPickupSpot::update()]" % self.name)
    #     time.sleep(1)
    #     try:
    #         # Finding the closet pickup position that is not visited yet
    #         least_dis = 9999999
    #         pickup_index = -1
    #         for i in range(len(self.blackboard.pickup_positions)):
    #             if self.blackboard.pickup_visited[i] == False:
    #                 distance_to_spot = np.linalg.norm(np.array(self.blackboard.pickup_positions[i]) - np.array(self.current_pose))
    #                 if distance_to_spot < least_dis:
                        
    #                     pickup_index = i
    #                     least_dis = distance_to_spot

    #         rospy.logerr("PICKUP SPOT IS: {}".format(pickup_index))
    #         if pickup_index < 5:
    #             self.blackboard.goal = self.blackboard.pickup_positions[pickup_index]
    #             # Update visited spot
    #             self.blackboard.pickup_visited[pickup_index] = True
    #             return py_trees.common.Status.SUCCESS
    #         else :
    #             rospy.logerr("[FindPickupSpot::update()]" + "Fail")
    #             return py_trees.common.Status.FAILURE
    #     except:
    #         rospy.logerr("[FindPickupSpot::update()]" + "Fail")
    #         return py_trees.common.Status.FAILURE

        
    # def terminate(self, new_status):
    #     self.logger.debug("  %s [FindPickupSpot::terminate().terminate()][%s->%s]" % (self.name, self.status, new_status))

    # def get_odom(self, odom):
    #     self.current_pose = [odom.pose.pose.position.x, odom.pose.pose.position.y]
