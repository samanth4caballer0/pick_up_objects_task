#!/usr/bin/env python
from nav_msgs.msg import Odometry
from std_srvs.srv import (
    Trigger,
    TriggerResponse,
    TriggerRequest,
    SetBool,
    SetBoolRequest,
)
import rospy


class ManageObject:
    def __init__(self):

        self.objects = [
            "redball",
            "greenball",
            "blueball",
        ]

        self.pick = False
        self.robot_pose = None

        server_check = rospy.Service("~check_object", Trigger, self.handle_check_object)
        server_take = rospy.Service("~get_object", Trigger, self.handle_pick_object)
        server_let = rospy.Service("~let_object", Trigger, self.handle_let_object)

        # remap odom topic
        subscriber = rospy.Subscriber("odom", Odometry, self.odom_callback)

    def handle_check_object(self, req: TriggerRequest):
        ret = TriggerResponse()
        print("----------------------")
        for ball in self.objects:
            ballpos: Odometry = rospy.wait_for_message(
                "/stonefish_simulator/" + ball + "/position", Odometry
            )
            pos = [ballpos.pose.pose.position.x, ballpos.pose.pose.position.y]
            distance = self.distance(pos, self.robot_pose)
            print(
                "robot distance to: "
                + ball
                + " = "
                + str(round(distance, 2))
                + " meters"
            )
            if distance < 0.43:
                print("able to pick: " + ball)
                ret.success = True
                ret.message = ball
                return ret
            else:
                ret.success = False
                ret.message = "No objects close"
        return ret

    def handle_pick_object(self, req: TriggerRequest):
        ret = TriggerResponse()
        print("----------------------")
        for ball in self.objects:
            ballpos: Odometry = rospy.wait_for_message(
                "/stonefish_simulator/" + ball + "/position", Odometry
            )
            pos = [ballpos.pose.pose.position.x, ballpos.pose.pose.position.y]
            distance = self.distance(pos, self.robot_pose)
            print(
                "robot distance to: "
                + ball
                + " = "
                + str(round(distance, 2))
                + " meters"
            )
            if distance < 0.43:
                print("picking up: " + ball)
                attachSrv = rospy.ServiceProxy(
                    "/turtlebot/stonefish_simulator/attach/" + ball, SetBool
                )
                attachSrv.call(data=True)

                ret.success = True
                ret.message = "Picking up " + ball
                return ret
            else:
                ret.success = False
                ret.message = "No objects close"
        return ret

    def handle_let_object(self, req: TriggerRequest):
        ret = TriggerResponse()
        print("----------------------")
        for ball in self.objects:
            attachSrv = rospy.ServiceProxy(
                "/turtlebot/stonefish_simulator/attach/" + ball, SetBool
            )
            attachSrv.call(data=False)

        ret.success = True
        ret.message = "Dropped all objects"
        return ret

    def odom_callback(self, data: Odometry):
        self.robot_pose = [data.pose.pose.position.x, data.pose.pose.position.y]

    def distance(self, p1, p2):
        return ((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2) ** 0.5


if __name__ == "__main__":
    rospy.init_node("sf_handle_objects")
    ManageObject()
    rospy.spin()
