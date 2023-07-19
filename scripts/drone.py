#!/usr/bin/env python3

import rospy
import sys
import select
import termios
import tty
import math

from tf.transformations import euler_from_quaternion, quaternion_from_euler

from geometry_msgs.msg import PoseStamped, Point, Quaternion
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

settings = termios.tcgetattr(sys.stdin)


# This function is responsible for getting keyboard input
def getKey(timeout):
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setcbreak(sys.stdin.fileno())

        if select.select([sys.stdin], [], [], timeout) == ([sys.stdin], [], []):
            return sys.stdin.read(1)
        else:
            return None
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


# This class is responsible for controlling the drone
class Drone:
    def __init__(self):
        rospy.init_node('px4_teleop_node')

        self.sub_state = rospy.Subscriber(
            "mavros/state", State, callback=self.CallbackState)
        self.sub_localPose = rospy.Subscriber(
            "/mavros/local_position/pose", PoseStamped, callback=self.CallbackLocalPose)

        self.pub_setpoint_local = rospy.Publisher(
            "mavros/setpoint_position/local", PoseStamped, queue_size=10)

        self.client_arming = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
        self.client_mode = rospy.ServiceProxy("mavros/set_mode", SetMode)

        self.current_state = State()
        self.current_yaw = 0  # Radian

        self.current_pose = PoseStamped()
        self.current_pose.pose.position = Point()
        self.current_pose.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)

        self.set_local_pose = PoseStamped()
        self.set_local_pose.pose.position = Point()
        self.set_local_pose.pose.orientation = Quaternion(x=0, y=0, z=0, w=1)

        self.takeoff_height = rospy.get_param('takeoff_height', 2.0)
        
        self.rate = rospy.Rate(10)


    # This function is responsible for updating the current state
    def CallbackState(self, msg):
        if self.current_state.mode != msg.mode :
            rospy.loginfo("Mode changed %s -> %s", self.current_state.mode, msg.mode)
        
        if self.current_state.armed != msg.armed :
            if msg.armed :
                rospy.loginfo("Now armed")
            else :
                rospy.loginfo("Now disarmed")

        self.current_state = msg


    # This function is responsible for updating the current pose
    def CallbackLocalPose(self, msg):
        self.current_pose = msg
        _, _, self.current_yaw = euler_from_quaternion([
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w
        ])


    # This function is responsible for connecting the drone
    def Connect(self):
        while(not rospy.is_shutdown() and not self.current_state.connected):
            rospy.loginfo_throttle(50, "Trying to connect MAVROS...")
            self.rate.sleep()
        rospy.loginfo("Connection successful")


    def ChangeMode(self, mode) :
        cmd_mode = SetModeRequest()
        cmd_mode.base_mode = 0
        cmd_mode.custom_mode = mode

        self.client_mode.call(cmd_mode)
        rospy.loginfo("Change mode to " + mode)

    
    def Arming(self) :
        if self.current_state.armed :
            rospy.logwarn("Already armed")
            return
        if self.current_state.system_status >= 6 :
            rospy.logwarn("Drone is not stable")
            return

        cmd_arm = CommandBoolRequest()
        cmd_arm.value = True

        self.client_arming.call(cmd_arm)
        rospy.loginfo("Try arming ...")

    
    
    # This function is responsible for taking off the drone
    def Takeoff(self, height):
        rospy.loginfo("Try takeoff ... [%lf]", height)

        self.Arming()

        self.set_local_pose = self.current_pose
        self.set_local_pose.pose.position.z = height

        self.ChangeMode("OFFBOARD")



    # This function is responsible for landing the drone
    def Land(self):
        self.ChangeMode("AUTO.LAND")

    def Hover(self):
        self.pub_setpoint_local.publish(self.set_local_pose)
        rospy.loginfo_throttle(5, "Hovering [%lf][%.2f, %.2f, %.2f]", math.degrees(self.current_yaw),
                self.current_pose.pose.position.x, self.current_pose.pose.position.y, self.current_pose.pose.position.z)


    def SetCurrentPose(self) :
        self.set_local_pose.pose.position = self.current_pose.pose.position

    def SetCurrentYaw(self) :
        self.set_local_pose.pose.orientation = self.current_pose.pose.orientation

    def MoveLinear(self, pose_x, pose_y, pose_z):
        if pose_x == 0 and pose_y == 0 and pose_z == 0:
            self.SetCurrentPose()

        else:
            goal_position = [pose_x, pose_y, pose_z]

            if pose_x > 0:
                goal_position[0] = pose_x * math.cos(self.current_yaw)
                goal_position[1] = pose_x * math.sin(self.current_yaw)
            elif pose_x < 0:
                goal_position[0] = abs(pose_x) * -math.cos(self.current_yaw)
                goal_position[1] = abs(pose_x) * -math.sin(self.current_yaw)

            if pose_y > 0:
                goal_position[0] = pose_y * math.sin(self.current_yaw)
                goal_position[1] = pose_y * -math.cos(self.current_yaw)
            elif pose_y < 0:
                goal_position[0] = abs(pose_y) * -math.sin(self.current_yaw)
                goal_position[1] = abs(pose_y) * math.cos(self.current_yaw)

            
            self.set_local_pose.pose.position.x += goal_position[0]
            self.set_local_pose.pose.position.y += goal_position[1]
            self.set_local_pose.pose.position.z += goal_position[2]

            rospy.loginfo("Move linear : %.2f, %.2f, %.2f", self.set_local_pose.pose.position.x,
                            self.set_local_pose.pose.position.y, self.set_local_pose.pose.position.z)


    def MoveAngular(self, degree):
        if degree == 0:
            self.SetCurrentYaw()

        else:
            goal_radian = self.current_yaw + math.radians(degree)

            if goal_radian >= math.pi * 2:
                goal_radian -= math.pi * 2
            elif goal_radian <= 0:
                goal_radian += math.pi * 2

            self.current_yaw = goal_radian

            quat = quaternion_from_euler(0, 0, self.current_yaw)

            self.set_local_pose.pose.orientation.x = quat[0]
            self.set_local_pose.pose.orientation.y = quat[1]
            self.set_local_pose.pose.orientation.z = quat[2]
            self.set_local_pose.pose.orientation.w = quat[3]

            rospy.loginfo("Move angular : [d: %lf][r: %lf]", math.degrees(self.current_yaw), self.current_yaw)
    

    # This function is responsible for running the controller
    def Run(self):
        rospy.loginfo("PX4 Teleop Node Start")

        self.Connect()

        rospy.loginfo("Keyboard Controll enabled")

        # Define dictionary mapping key inputs to actions
        actions = {
            'w': {"log": "[Input : w] Move foward",   "action": lambda: self.MoveLinear(1, 0, 0)},
            'a': {"log": "[Input : a] Move left",     "action": lambda: self.MoveLinear(0, -1, 0)},
            's': {"log": "[Input : s] Stop movement", "action": lambda: self.MoveLinear(0, 0, 0)},
            'd': {"log": "[Input : d] Move right",    "action": lambda: self.MoveLinear(0, 1, 0)},
            'x': {"log": "[Input : x] Move backward", "action": lambda: self.MoveLinear(-1, 0, 0)},

            'u': {"log": "[Input : u] Move up",   "action": lambda: self.MoveLinear(0, 0, 1)},
            'j': {"log": "[Input : j] Move down", "action": lambda: self.MoveLinear(0, 0, -1)},

            't': {"log": "[Input : t] Takeoff", "action": lambda: self.Takeoff(self.takeoff_height)},
            'l': {"log": "[Input : l] Land",    "action": lambda: self.Land()},

            '1': {"log": "[Input : 1] Trun left",   "action": lambda: self.MoveAngular(5)},
            '2': {"log": "[Input : 1] stop rotate", "action": lambda: self.MoveAngular(0)},
            '3': {"log": "[Input : 1] Trun Right",  "action": lambda: self.MoveAngular(-5)},

            'q': {"log": "Quit", "action": lambda: rospy.signal_shutdown("Quit")}

        }

        while not rospy.is_shutdown():
            key = getKey(0.1)

            # Check if key is in the dictionary
            if key in actions:
                rospy.loginfo(actions[key]["log"])
                actions[key]["action"]()

            if self.current_state.armed :
                self.Hover()

            self.rate.sleep()



if __name__ == "__main__":
    drone = Drone()
    drone.Run()
