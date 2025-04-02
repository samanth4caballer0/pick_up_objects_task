import rospy
from geometry_msgs.msg import Pose,PoseStamped,Twist
from nav_msgs.msg import Path,Odometry
import numpy as np
import tf
import py_trees
from visualization_msgs.msg import Marker


# Behavior for path following
class FollowPath(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(FollowPath, self).__init__(name)
        self.goal=PoseStamped()
        # controller class object
        self.controller=Controller("/turtlebot/kobuki/odom_ground_truth",
                                   "/turtlebot/kobuki/commands/velocity",0.15)
        self.path=Path()

    def setup(self):
        self.logger.debug("  %s [FollowPath::setup()]" % self.name)
        self.pathPub=rospy.Publisher("/move_base_simple/goal",PoseStamped,queue_size=1) ####### TODO should this be uncommented? 

    def initialise(self):
        self.logger.debug("  %s [FollowPath::initialise()]" % self.name)
        print("Getting path topic")
        self.path:Path=rospy.wait_for_message("/turtlebot/planned_path",Path)
        print("Following path")

        for wp in self.path.poses:
            point=[wp.pose.position.x,wp.pose.position.y]
            self.controller.path.append(point)

        # Timers
        self.timer=rospy.Timer(rospy.Duration(0.1), self.controller.controller)

    def update(self):
        try:
            # self.logger.debug("  {}: following path".format(self.name))
            if self.controller.goal_reached:
                self.timer.shutdown()
                return py_trees.common.Status.SUCCESS
                
            else:
                return py_trees.common.Status.RUNNING
        except:
            self.logger.debug("  {}: Error, something happened".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(
            "  %s [FollowPath::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )

class Controller:
    def __init__(self, odom_topic, cmd_vel_topic, distance_threshold):

        # Attributes
        self.distance_threshold = distance_threshold  # Distance threshold to way point
        self.current_pose = None  # Current robot SE2 pose
        self.goal = None  # A goal is set
        self.goal_reached=False
        self.path = []  # List of points which define the plan. None if there is no plan
        # Parameters
        self.Kv = 0.5  # Proportional linear velocity controller
        self.Kw = 0.5  # Proportional angular velocity controller
        self.v_max = 0.15  # Maximum linear velocity control action ##################### CHANGE THIS? #####################
        self.w_max = 0.3  # Maximum angular velocity control action

        # Publishers
        self.cmd_pub = rospy.Publisher(cmd_vel_topic, Twist, queue_size=1)
        self.marker_pub = rospy.Publisher('~path_marker', Marker, queue_size=1)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber(odom_topic, Odometry, self.get_odom)
        self.move_goal_sub = rospy.Subscriber('/move_base_simple/goal', PoseStamped, self.get_goal)


        # Wrap angle between -pi and pi
    def wrap_angle(self,angle):
        return angle + (2.0 * np.pi * np.floor((np.pi - angle) / (2.0 * np.pi)))


    # Controller
    def move_to_point(self,current, goal, Kv=0.5, Kw=0.5):
        """Computes the control command to move from current position to goal."""
        theta_d = np.arctan2(goal[1] - current[1], goal[0] - current[0])
        w = Kw * self.wrap_angle(theta_d - current[2])
        v = 0
        if abs(w) < 0.05:  # to avoid move while turning
            v = Kv * np.linalg.norm(goal - current[0:2])
        return v, w

    # Odometry callback
    def get_odom(self, odom:Odometry):
        _, _, yaw = tf.transformations.euler_from_quaternion(
            [
                odom.pose.pose.orientation.x,
                odom.pose.pose.orientation.y,
                odom.pose.pose.orientation.z,
                odom.pose.pose.orientation.w,
            ]
        )
        self.current_pose = np.array(
            [odom.pose.pose.position.x, odom.pose.pose.position.y, yaw]
        )

    # Iterate: check to which way point the robot has to face. Send zero velocity if there's no active path.
    def controller(self, event):
        v = 0
        w = 0
        if self.path is not None and len(self.path) > 0:
            # If current waypoint reached with some tolerance move to next point otherwise move to current point
            if (np.linalg.norm(self.path[0] - self.current_pose[0:2])< self.distance_threshold):
                print("Position {} reached".format(self.path[0]))
                del self.path[0]
                if len(self.path) == 0:
                    self.goal = None
                    self.goal_reached=True
                    print("Final position reached!")
            else:
                self.goal_reached=False
                v, w = self.move_to_point(self.current_pose, self.path[0], self.Kv, self.Kw)
        self.__send_commnd__(v, -w)

    # Publishers
    def __send_commnd__(self, v, w):
        cmd = Twist()
        cmd.linear.x = np.clip(v, -self.v_max, self.v_max)
        cmd.linear.y = 0
        cmd.linear.z = 0
        cmd.angular.x = 0
        cmd.angular.y = 0
        cmd.angular.z = np.clip(w, -self.w_max, self.w_max)
        self.cmd_pub.publish(cmd)



