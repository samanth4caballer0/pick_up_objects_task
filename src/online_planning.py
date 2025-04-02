import numpy as np
import math
import time

def wrap_angle(angle):
    return (angle + ( 2.0 * np.pi * np.floor( ( np.pi - angle ) / ( 2.0 * np.pi ) ) ) )

class StateValidityChecker:
    """ Checks if a position or a path is valid given an occupancy map."""

    # Constructor
    def __init__(self, distance=0.1, is_unknown_valid=True):
        # map: 2D array of integers which categorizes world occupancy
        self.map = None 
        # map sampling resolution (size of a cell))                            
        self.resolution = None
        # world position of cell (0, 0) in self.map                      
        self.origin = None
        # set method has been called                          
        self.there_is_map = False
        # radius arround the robot used to check occupancy of a given position                 
        self.distance = distance                    
        # if True, unknown space is considered valid
        self.is_unknown_valid = is_unknown_valid    
    
    # Set occupancy map, its resolution and origin. 
    def set(self, data, resolution, origin):
        self.map = data # np.array already
        self.resolution = resolution
        self.origin = np.array(origin)
        self.there_is_map = True
        self.bound = [resolution*data.shape[0] ]

    # Given a pose, returs true if the pose is not in collision and false othewise.
    def is_valid(self, pose): 
        if self.there_is_map == False:
            return(False)
        # TODO: convert world robot position to map coordinates using method __position_to_map__
        for x in np.arange(pose[0]-self.distance,pose[0]+self.distance,self.resolution):
            for y in np.arange(pose[1]-self.distance,pose[1]+self.distance,self.resolution):
                m = self.__position_to_map__(np.array([x,y]))

        # TODO: check occupancy of the vicinity of a robot position (indicated by self.distance atribute). 
        # Return True if free, False if occupied and self.is_unknown_valid if unknown. 
        # If checked position is outside the map bounds consider it as unknown.

                if m is None:
                    if not self.is_unknown_valid:
                        return False
                elif self.map[m[0],m[1]]==-1:
                    if not self.is_unknown_valid:
                        return False
                elif self.map[m[0],m[1]]==100:
                    return(False)
                    
        return(True)

    # Given a path, returs true if the path is not in collision and false othewise.
    def check_path(self, path):
        # TODO: Discretize the positions between 2 waypoints with an step_size = 2*self.distance
        step_size = 2*self.distance
        for i in range(len(path)-1):
            dist = np.linalg.norm(path[i] - path[i+1])
            zeta = np.arctan2(path[i+1][1] - path[i][1], path[i+1][0] - path[i][0])

            step = int(dist//step_size)

        # TODO: for each point check if `is_valid``. If only one element is not valid return False, otherwise True.
        # For each point between 2 waypoints check if `is_valid`. 
            for j in range(step+1):
                inc_dist = step_size * (j)
                wp_check  = [path[i][0] + (inc_dist*math.cos(zeta)),path[i][1] + (inc_dist*math.sin(zeta))] 
                if not self.is_valid(wp_check):
                    #print("Path is not valid")
                    return False 
        # For the waypoint itself check if `is_valid`.
            wp_check = path[i+1]
            
            if not self.is_valid(wp_check):
               # print("Path is not valid")
                return False 
        return True
            
    def is_inside_map(self, p):
        return bool(self.__position_to_map__(p))

    # Transform position with respect the map origin to cell coordinates
    def __position_to_map__(self, p):

        # TODO: convert world position to map coordinates. 
        
        # find p in image plane
        p_img  = p - self.origin
        
        # find row and col
        m = [int(np.floor(p_img[0] / self.resolution)),int(np.floor(p_img[1] / self.resolution))]
        # check that the cell is inside the grid map
        if m[0] >= self.map.shape[0] or m[1] >= self.map.shape[1] or m[0] < 0 or m[1] < 0:
            return None
        
        return m
    

class Node:
    def __init__(self, pose, parent = None) -> None:
        self.pose = pose
        self.parent = parent

# Define RRT class (you can take code from Autonopmous Systems course!)
class RRT:
    def  __init__(self, state_validity_checker, max_time=10, delta_q=0.4, p_goal=0.6, dominion=[-10, 10, -10, 10]):
        # define constructor ...
        self.state_validity_checker = state_validity_checker
        self.max_time = max_time
        self.delta_q = delta_q
        self.p_goal = p_goal
        self.dominion = dominion
        
    def compute_path(self, q_start, q_goal):
        # Implement RRT algorithm.
        # Use the state_validity_checker object to see if a position is valid or not.q
        q_start =Node(q_start)
        q_goal = Node(q_goal)
        G = [] 
        E = []
        G.append(q_start)
        start_time = time.time()
        while (time.time()-start_time) < self.max_time:
            qrand = self.RAND_CONF(self.p_goal,q_goal)
            qnear = self.NEAREST_VERTEX(qrand,G)
            qnew = self.NEW_CONF(qnear, qrand, self.delta_q)
            if self.IS_SEGMENT_FREE(qnear, qnew):
                G.append(qnew)
                qnew.parent = qnear
                E.append([qnear,qnew])

                if (qnew.pose == q_goal.pose).all():
                    q_goal = qnew
                    path = self.FILL_PATH(q_start,q_goal)
                    smooth_path = self.SMOOTH_PATH(path)
                    return smooth_path
        return []
    
                             

    def RAND_CONF(self, p, qgoal):
        rand =  np.random.rand()
        if rand > p:
            qrand = (np.random.uniform(self.dominion[0],self.dominion[1]),np.random.randint(self.dominion[2],self.dominion[3]))# 
            while not self.state_validity_checker.is_valid(qrand):
                qrand = (np.random.uniform(self.dominion[0],self.dominion[1]),np.random.randint(self.dominion[2],self.dominion[3])) 

            return Node(pose=np.array(qrand)) 
        else:
            return qgoal

    def NEAREST_VERTEX(self, qrand, G):
        min_dist = 100000
               
        for i in range(len(G)):

            dist = self.dist(qrand,G[i])
            if dist < min_dist:
                min_dist = dist
                qnear = G[i]
        return qnear
    
    def NEW_CONF(self, qnear, qrand, delta_q):
        dist = self.dist(qnear,qrand)
        if delta_q > dist:
            return qrand 
        zeta = np.arctan2(qrand.pose[1] - qnear.pose[1], qrand.pose[0] - qnear.pose[0])
        new_pose  = (qnear.pose[0] + delta_q*math.cos(zeta),qnear.pose[1] + delta_q*math.sin(zeta))

        qnew = Node(np.array(new_pose))
        return qnew
    
    def IS_SEGMENT_FREE(self, qnear,qnew):
        if self.state_validity_checker.check_path([qnear.pose,qnew.pose]):  
            return True
        else:
            return False
    
    def FILL_PATH(self, qstart,qgoal):
        qcurrent = qgoal
        path = [qcurrent]
        
        while (qcurrent.pose != qstart.pose).any():   
            qcurrent = qcurrent.parent
            path.append(qcurrent) 
        path.reverse()
        return path
    
    def SMOOTH_PATH(self, path):
        current = path[-1]
        smooth_path = [current.pose]

        while (current.pose != path[0].pose).any():    
            for p in path:
                if self.IS_SEGMENT_FREE(current,p):
                    current = p
                    break
            smooth_path.append(p.pose)
        smooth_path.reverse()
        return smooth_path
    
    def dist(self, q1,q2):
        return ((q2.pose[0] - q1.pose[0])**2 + (q2.pose[1] - q1.pose[1])**2)**0.5


# Planner: This function has to plan a path from start_p to goal_p. To check if a position is valid the 
# StateValidityChecker class has to be used. The planning dominion must be specified as well as the maximum planning time.
# The planner returns a path that is a list of poses ([x, y]).
def compute_path(start_p, goal_p, state_validity_checker, bounds, max_time=1.0):

    # TODO: Plan a path from start_p to goal_p inside bounds using the RRT and the 
    # StateValidityChecker Objects previously defined.
    # TODO: if solved, return a list with the [x, y] points in the solution path.
    # example: [[x1, y1], [x2, y2], ...]
    # TODO: Ensure that the path brings the robot to the goal (with a small tolerance)!
    if goal_p[0] < bounds[0] or goal_p[0] > bounds[1] or goal_p[1] < bounds[2] or goal_p[1] > bounds[3] : 
        return []

    rrt = RRT(state_validity_checker, dominion=bounds, max_time=max_time)
    path = rrt.compute_path(start_p, goal_p)

    if path:
        return path
    else:
        return []
    

# Controller: Given the current position and the goal position, this function computes the desired 
# lineal velocity and angular velocity to be applied in order to reah the goal.
def move_to_point(current, goal, Kv=0.5, Kw=0.5):
    
    # TODO: Use a proportional controller which sets a velocity command to move from current position to goal (u = Ke)
    # To avoid strange curves, first correct the orientation and then the distance. 
    # Hint: use wrap_angle function to maintain yaw in [-pi, pi]
    # This function should return only  linear velocity (v) and angular velocity (w)
    d = math.sqrt((goal[0] - current[0])**2 + (goal[1] - current[1])**2)
    psi_d = math.atan2(goal[1] - current[1], goal[0] - current[0]) 
    if abs(wrap_angle(psi_d - current[2])) < 0.5:   
        w = Kw * wrap_angle(psi_d - current[2])
        v = Kv* d
    else:
        w = Kw * wrap_angle(psi_d - current[2])
        v = 0
    return v, w


    
