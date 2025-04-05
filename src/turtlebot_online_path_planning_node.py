#!/usr/bin/python3

import numpy as np
import rospy
import tf

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA 
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped

from online_planning import StateValidityChecker, move_to_point, compute_path

class OnlinePlanner:

    # OnlinePlanner Constructor
    def __init__(self, gridmap_topic, odom_topic, cmd_vel_topic, bounds, distance_threshold):

        # ATTRIBUTES
        # List of points which define the plan. None if there is no plan
        self.path = []
        # State Validity Checker object                                                 
        self.svc = StateValidityChecker(distance_threshold)
        # Current robot SE2 pose [x, y, yaw], None if unknown            
        self.current_pose = None
        # Goal where the robot has to move, None if it is not set                                                                   
        self.goal = None
        # Last time a map was received (to avoid map update too often)                                                
        self.last_map_time = rospy.Time.now()
        # Dominion [min_x_y, max_x_y] in which the path planner will sample configurations                           
        self.bounds = bounds                                        

        # CONTROLLER PARAMETERS
        # Proportional linear velocity controller gain
        self.Kv = 0.8
        # Proportional angular velocity controller gain                   
        self.Kw = 0.5
        # Maximum linear velocity control action                   
        self.v_max = 0.15
        # Maximum angular velocity control action               
        self.w_max = 1.0      

        # Planning stopping criteria
        self.max_retries = 5   # Maximum number of retries
        self.max_planning_time = 10 # Initial palnning time limit (s)
        self.reserved_time = 10   #  palnning time to increse after fail (s)
        self.goal_torlerance = 0.1

        # Recovery Behavior Parmeters 
        self.recovery_time = rospy.Duration(3)
        self.recovery_vel = -0.1

        # PUBLISHERS
        # Publisher for sending velocity commands to the robot
        self.cmd_pub = rospy.Publisher(cmd_vel_topic,Twist,queue_size=1) 
        # Publisher for visualizing the path to with rviz
        self.marker_pub = rospy.Publisher('/turtlebot_online_path_planning/path_marker', Marker, queue_size=1)
        self.waypoints_pub = rospy.Publisher('/turtlebot_online_path_planning/waypoints_marker', Marker, queue_size=1)

        
        # SUBSCRIBERS
        self.gridmap_sub = rospy.Subscriber(gridmap_topic,OccupancyGrid, self.get_gridmap) 
        self.odom_sub = rospy.Subscriber(odom_topic,Odometry, self.get_odom) 
        self.move_goal_sub = rospy.Subscriber('/move_base_simple/goal',PoseStamped, self.get_goal) 
        
        # TIMERS
        # Timer for velocity controller
        rospy.Timer(rospy.Duration(0.1), self.controller)
        rospy.Timer(rospy.Duration(3), self.publish_path)

    
    # Odometry callback: Gets current robot pose and stores it into self.current_pose
    def get_odom(self, odom):
        _, _, yaw = tf.transformations.euler_from_quaternion([odom.pose.pose.orientation.x, 
                                                              odom.pose.pose.orientation.y,
                                                              odom.pose.pose.orientation.z,
                                                              odom.pose.pose.orientation.w])
        x = odom.pose.pose.position.x 
        y = odom.pose.pose.position.y
        
        self.current_pose = np.array([x,y,yaw])
    
    # Goal callback: Get new goal from /move_base_simple/goal topic published by rviz 
    # and computes a plan to it using self.plan() method
    def get_goal(self, goal):
        rospy.logwarn("New goal received")
        if self.svc.there_is_map:
            rospy.logwarn("New goal received yes")
            x = goal.pose.position.x
            y = goal.pose.position.y
            self.goal = np.array([x,y])
            rospy.loginfo("New goal: {}".format(self.goal))
            rospy.loginfo("Current Pose: {}".format(self.current_pose[0:2]))
            
            test_path = [self.current_pose[0:2]- [1,0],self.current_pose[0:2],self.goal]
            if self.svc.check_path(test_path):
                rospy.logwarn("New path is valid")
            else:
                rospy.logerr("New path is not valid")

            if self.svc.is_valid(self.goal):
                # Plan a new path to self.goal
                self.path = []
                self.plan()
            else:
                rospy.logerr("New goal is not valid")

        
    # Map callback: Gets the latest occupancy map published by Octomap server and update 
    # the state validity checker
    def get_gridmap(self, gridmap):
        # To avoid map update too often (change value '1' if necessary)
        if (gridmap.header.stamp - self.last_map_time).to_sec() > 2:      
      
            self.last_map_time = gridmap.header.stamp

            # Update State Validity Checker
            env = np.array(gridmap.data).reshape(gridmap.info.height, gridmap.info.width).T
            origin = [gridmap.info.origin.position.x, gridmap.info.origin.position.y]
            self.svc.set(env, gridmap.info.resolution, origin)

            # If the robot is following a path, check if it is still valid
            if self.path is not None and len(self.path) > 0 and self.goal is not None:
                # create total_path adding the current position to the rest of waypoints in the path
                total_path = [self.current_pose[0:2]] + self.path
                
                rospy.loginfo("Checking current path validity ...")

                if not self.svc.check_path(total_path):
                    rospy.logerr("Current path is not valid -- plan a new path")
                    # if self.svc.is_valid(self.goal):
                    self.path = []
                    self.plan()
                    # else:
                    #     rospy.logerr("Current goal is stucked in obstacle")
                else:
                    rospy.logwarn("Current path is valid -- stay on same path")


    # Solve plan from current position to self.goal. 
    def plan(self):
        # Invalidate previous plan if available
        self.path = []

        rospy.loginfo("Check if robot is in the obstacle")

        # Check if robot stuck in the obstacle 
        while self.is_stuck_in_obst(): 
            rospy.logerr("Robot is stuck in an obstacle!")
            self.recover()
            
        rospy.loginfo("Compute new path")

        # TODO: plan a path from self.current_pose to self.goal
        self.path = compute_path(start_p= self.current_pose[0:2],goal_p=self.goal[0:2],
                                 state_validity_checker=self.svc, bounds= self.bounds,max_time=self.max_planning_time)
        
        # TODO: If planning fails, consider increasing the planning time, retry the planning a few times, etc.
        for i in range(self.max_retries):
            rospy.logwarn("Path not found! --> Retrying.. ({}/{} attempts)".format(i+1,self.max_retries))
            self.path = compute_path(start_p= self.current_pose[0:2],goal_p=self.goal[0:2],
                                 state_validity_checker=self.svc, bounds= self.bounds, max_time=self.max_planning_time+ self.reserved_time) 
            if self.path:
                break
            

        if len(self.path) == 0:
            rospy.logerr("Path not found!")
        else:
            rospy.loginfo("Path found")
            rospy.loginfo("Path: {}".format(self.path))
            # Publish plan marker to visualize in rviz
            self.publish_path()
            # remove initial waypoint in the path (current pose is already reached)
            del self.path[0]                 
        

    # This method is called every 0.1s. It computes the velocity comands in order to reach the 
    # next waypoint in the path. It also sends zero velocity commands if there is no active path.
    def controller(self, event): #CHEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEECK
        v = 0
        w = 0
        try:
            if len(self.path) > 0:
                #rospy.loginfo("Current Waypoint: {}".format(self.path[0])) 
                if self.near_waypoint() :# TODO: If current waypoint is reached with some tolerance move to the next waypoint. 
                    del self.path[0]
                    # If it was the last waypoint in the path show a message indicating it 
                    rospy.loginfo("Waypoint reached --> Move to the next waypoint")
                else: 
                    v,w = move_to_point(self.current_pose,self.path[0],self.Kv,self.Kw)
                    # rospy.loginfo("v: %f, w: %f", v, w)

            else:
                #rospy.logwarn("Path is empty")
                pass
        except:
            rospy.logerr("Error in controller")
        # Publish velocity commands
        self.__send_commnd__(v, -w)
    
    def near_waypoint(self):
        wp = self.path[0]
        dist = np.sqrt((wp[0]-self.current_pose[0])**2 + (wp[1]-self.current_pose[1])**2)
        return dist < self.goal_torlerance
    
        
    # PUBLISHER HELPERS
    # Transform linear and angular velocity (v, w) into a Twist message and publish it
    def __send_commnd__(self, v, w):
        cmd = Twist()
        cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
        self.cmd_pub.publish(cmd)

    # Publish a path as a series of line markers
    def publish_path(self, _):
        
        #print("Publish path!")
        m = Marker()
        m.header.frame_id = 'world_ned'
        m.header.stamp = rospy.Time.now()
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.ns = 'path'
        m.action = Marker.DELETE
        m.lifetime = rospy.Duration(0)
        self.marker_pub.publish(m)

        m.action = Marker.ADD
        m.scale.x = 0.1
        m.scale.y = 0.0
        m.scale.z = 0.0
        
        m.pose.orientation.x = 0
        m.pose.orientation.y = 0
        m.pose.orientation.z = 0
        m.pose.orientation.w = 1
        
        color_red = ColorRGBA()
        color_red.r = 1
        color_red.g = 0
        color_red.b = 0
        color_red.a = 1
        color_blue = ColorRGBA()
        color_blue.r = 0
        color_blue.g = 0
        color_blue.b = 1
        color_blue.a = 1

        c = Marker()
        c.header.frame_id = "world_ned"
        c.header.stamp = rospy.Time.now()
        c.ns = "waypoints_list"
        c.id = 0
        c.type = Marker.CUBE_LIST
        c.action = Marker.ADD
        c.scale.x = 0.1
        c.scale.y = 0.1
        c.scale.z = 0.1
        c.color.a = 1.0  # Alpha
        
        if len(self.path) > 0:
            p = Point()
            p.x = self.current_pose[0]
            p.y = self.current_pose[1]
            p.z = 0.0
            m.points.append(p)
            m.colors.append(color_blue)
            
            for n in self.path:
                p = Point()
                p.x = n[0]
                p.y = n[1]
                p.z = 0.0
                m.points.append(p)
                m.colors.append(color_red)
                c.points.append(p)   
        
        self.marker_pub.publish(m)           
        self.waypoints_pub.publish(c)

    def recover(self):
        """
        Recovery behavior by just moving the robot backward
        """
        rospy.logwarn("Robot is stuck in an obstacle! Recovering...")
        cmd = Twist()
        start_time = rospy.Time.now()
        while (rospy.Time.now() -start_time ) < self.recovery_time:
            v = self.recovery_vel
            w = 0
            cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
            cmd.linear.y = 0
            cmd.linear.z = 0
            cmd.angular.x = 0
            cmd.angular.y = 0
            cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
            self.cmd_pub.publish(cmd)
        rospy.loginfo("Finish recovering!")
        cmd.linear.x = 0
        cmd.linear.y = 0
        self.cmd_pub.publish(cmd)


    def is_stuck_in_obst(self):
        # print(self.current_pose[0:2])
        return not self.svc.is_valid(self.current_pose[0:2])
            
# MAIN FUNCTION
if __name__ == '__main__':
    rospy.init_node('turtlebot_online_path_planning_node')   
    node = OnlinePlanner('/projected_map', '/turtlebot/kobuki/odom_ground_truth', '/turtlebot/kobuki/commands/velocity', np.array([-10.0, 10.0, -10.0, 10.0]), 0.18)#0.18)
    rospy.logwarn("STARTING ONLINE PLANNIG NODE")
    # Run forever
    rospy.spin()
