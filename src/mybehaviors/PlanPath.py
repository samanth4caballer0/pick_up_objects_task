import rospy
from geometry_msgs.msg import Pose,PoseStamped
from nav_msgs.msg import Path
import py_trees


# Behavior for planning path
class PlanPath(py_trees.behaviour.Behaviour):
    def __init__(self, name):
        super(PlanPath, self).__init__(name)
        # self.blackboard = self.attach_blackboard_client(name=self.name)
        # self.blackboard.register_key("goal", access=py_trees.common.Access.WRITE)
        self.goal=PoseStamped()
        # planner class object
        self.planner=Planner()
        self.waypoints=[]
        
        # For map data
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None
        self.map_initialized = False

    def setup(self):
        self.logger.debug("  %s [PlanPath::setup()]" % self.name)
        self.pathPub=rospy.Publisher("/turtlebot/planned_path",Path,queue_size=1,latch=True)

    # Subscribe to map topic (adjust topic name as needed)
    # rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

    def initialise(self):
        self.logger.debug("  %s [PlanPath::initialise()]" % self.name)
        # print("Planning to goal:")
        # print(self.blackboard.get("goal"))
        # print(self.goal)
        self.planner.waypoints=self.waypoints

    def update(self):
        try:
            self.logger.debug("  {}: computing path".format(self.name))
            path:Path=self.planner.plan(self.goal)
            
            if len(path.poses) != 0:
                self.pathPub.publish(path)
                return py_trees.common.Status.SUCCESS
            else:
                self.logger.debug("  {}: Path not found".format(self.name))
                return py_trees.common.Status.FAILURE
        except:
            self.logger.debug("  {}: Error, path not computed".format(self.name))
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        self.logger.debug(
            "  %s [PlanPath::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )


# simple planner class, replace with your RRT path planner
class Planner:
    def __init__(self):
        # initialize planner params
        self.path=Path()
        self.path.header.frame_id="world_ned"
        self.waypoints=[]

    def plan(self,goal:PoseStamped):
        for wp in self.waypoints:
            poseS=PoseStamped()
            poseS.pose.orientation.w=1
            poseS.pose.position.x=wp[0]
            poseS.pose.position.y=wp[1]
            self.path.poses.append(poseS)

        # self.path.poses.append(goal)
        
        return self.path

