#!/usr/bin/env python2

from __future__ import division

PKG = 'px4'

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Quaternion
from mavros_test_common import MavrosTestCommon
from pymavlink import mavutil
from six.moves import xrange
from std_msgs.msg import Header
from threading import Thread
from tf.transformations import quaternion_from_euler
from mavros_msgs.msg import Altitude, ExtendedState, HomePosition, State, \
                            WaypointList
from mavros_msgs.srv import CommandBool, ParamGet, SetMode, WaypointClear, \
                            WaypointPush
from sensor_msgs.msg import NavSatFix
import numpy as np
import time
import math
import random 
from sklearn.neighbors import KDTree
from mpl_toolkits import mplot3d
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import axes3d 
import matplotlib.pyplot as plt 
from matplotlib import style 
from scipy import spatial
import copy
from queue import PriorityQueue 
q = PriorityQueue() 
plt.ion()
length = 20
breadth = 15
height = 10
# fig = plt.figure()
# ax = plt.axes(projection='3d')

# ax.set_xlim3d(0, 20)
# ax.set_ylim3d(0,15)
# ax.set_zlim3d(0,10)

c = 1


# To check if a point is in obstacle space or not
def obstacle_check(i,j,k):
    a = b = c = d = e = f = g = 0
    if(5-c <= i <=6+c) and (5-c <= j <= 6+c) and (0 <= k <= 10):
        a = 1
    elif (8-c <= i <=9+c) and (11-c <= j <= 12+c) and (0 <= k <= 10):
        b = 1
    elif (12-c <= i <=13+c) and (14-c <= j <= 15+c) and (0 <= k <= 10):
        c = 1
    elif (10-c <= i <11+c) and (2-c <= j <= 3+c) and (0 <= k <= 10):
        d = 1
    elif (16-c <= i <=17+c) and (7-c <= j <= 8+c) and (0 <= k <= 10):
        e = 1
    elif (3-c <= i <=4+c) and (10-c <= j <= 11+c) and (0 <= k <= 10):
        f = 1
    elif (17-c <= i <=18+c) and (3-c <= j <= 4+c) and (0 <= k <= 10):
        g = 1
    # elif (11-c <= i <=12+c) and (7-c <= j <= 8+c) and (0 <= k <= 10):
    #     h = 1

    if  ((a == 1) or (b == 1) or (c == 1) or (d == 1) or (e == 1) or (f == 1) or (g == 1) ):
        return True
    else:
        return False

# Plots the osbstacles in the 3D map
def obstacle_map():
    # defining x, y, z co-ordinates for bar position 
    x = [5,8,12,10,16,3,17] 
    y = [5,11,14,2,7,10,3] 
    z = np.zeros(7)

    # size of bars 
    dx = np.ones(7)              # length along x-axis 
    dy = np.ones(7)              # length along y-axs 
    dz = [10,10,10,10,10,10,10]   # height of bar 

    # setting color scheme 
    color = [] 
    for h in dz: 
        if h > 5: 
            color.append('b') 
        else: 
            color.append('b') 

    ax.bar3d(x, y, z, dx, dy, dz, color = color) 


# Boundary check to avoid exploration outside the map space
def boundary_check(i, j, k):
    if (i < 0) or (j < 0) or (k < 0) or (i >= length) or (j >= breadth) or (k >= height):
        return True
    else:
        return False


# Returms the point 0.1m away from parent to child
def line_obstacle_check(j, i):
    k = (i[0] - j[0], i[1] - j[1], i[2] - j[2])
    k_mod = math.sqrt(k[0]**2 + k[1]**2 + k[2]**2)
    vec = (k[0]/k_mod, k[1]/k_mod, k[2]/k_mod)
    new_point = (j[0]+0.1*vec[0], j[1]+ 0.1*vec[1], j[2]+0.1*vec[2])
    return new_point

# Returns Euclidean distance between two 3D points
def cost2go(pt1, pt2):
    dist = math.sqrt((pt2[0] - pt1[0]) ** 2 + (pt2[1] - pt1[1]) ** 2 + (pt2[2] - pt1[2]) ** 2) 
    return dist

# Generates a random seed in 3D rounded off to 0.5
def generate_seed():
    x = round(random.uniform(0 , length)*2)/2
    y = round(random.uniform(0 , breadth)*2)/2
    z = round(random.uniform(0 , height)*2)/2
    return (x,y,z)
    
# Sphere to check Goal covergence 
def goalcheck_circle(x, y, z, goal_x, goal_y, goal_z):
    if ((x - goal_x) ** 2 + (y - goal_y) ** 2 + (z - goal_z) **2 <= (0.5 ** 2)):
        return True
    else:
        return False
    
# Returns the point after maximum step propagation disance of 2m
def max_step_prop(j, i):
    k = (i[0] - j[0], i[1] - j[1], i[2] - j[2])
    k_mod = math.sqrt(k[0]**2 + k[1]**2 + k[2]**2)
    vec = (k[0]/k_mod, k[1]/k_mod, k[2]/k_mod)
    new_point = (j[0]+4*vec[0], j[1]+ 4*vec[1], j[2]+4*vec[2])
    return new_point

# Returns the set if neighbours for a seed in a given radius
def neighbours(seed, r, tree):
    results = tree.query_ball_point((seed), r)
    nearby_points = X[results]
    return nearby_points


class missionfly():
    """
    Tests flying a path in offboard control by sending position setpoints
    via MAVROS.

    For the test to be successful it needs to reach all setpoints in a certain time.

    FIXME: add flight path assertion (needs transformation from ROS frame to NED)
    """
    
    def setUp(self):
        self.altitude = Altitude()
        self.extended_state = ExtendedState()
        self.global_position = NavSatFix()
        self.home_position = HomePosition()
        self.local_position = PoseStamped()
        self.mission_wp = WaypointList()
        self.state = State()
        self.mav_type = None

        self.sub_topics_ready = {
            key: False
            for key in [
                'alt', 'ext_state', 'global_pos', 'home_pos', 'local_pos',
                'mission_wp', 'state'
            ]
        }

        # ROS services
        service_timeout = 30
        rospy.loginfo("waiting for ROS services")
        try:
            rospy.wait_for_service('mavros/param/get', service_timeout)
            rospy.wait_for_service('mavros/cmd/arming', service_timeout)
            rospy.wait_for_service('mavros/mission/push', service_timeout)
            rospy.wait_for_service('mavros/mission/clear', service_timeout)
            rospy.wait_for_service('mavros/set_mode', service_timeout)
            rospy.loginfo("ROS services are up")
        except rospy.ROSException:
            self.fail("failed to connect to services")
        self.get_param_srv = rospy.ServiceProxy('mavros/param/get', ParamGet)
        self.set_arming_srv = rospy.ServiceProxy('mavros/cmd/arming',
                                                 CommandBool)
        self.set_mode_srv = rospy.ServiceProxy('mavros/set_mode', SetMode)
        self.wp_clear_srv = rospy.ServiceProxy('mavros/mission/clear',
                                               WaypointClear)
        self.wp_push_srv = rospy.ServiceProxy('mavros/mission/push',
                                              WaypointPush)

        # ROS subscribers
        self.alt_sub = rospy.Subscriber('mavros/altitude', Altitude,
                                        self.altitude_callback)
        self.ext_state_sub = rospy.Subscriber('mavros/extended_state',
                                              ExtendedState,
                                              self.extended_state_callback)
        self.global_pos_sub = rospy.Subscriber('mavros/global_position/global',
                                               NavSatFix,
                                               self.global_position_callback)
        self.home_pos_sub = rospy.Subscriber('mavros/home_position/home',
                                             HomePosition,
                                             self.home_position_callback)
        self.local_pos_sub = rospy.Subscriber('mavros/local_position/pose',
                                              PoseStamped,
                                              self.local_position_callback)
        self.mission_wp_sub = rospy.Subscriber(
            'mavros/mission/waypoints', WaypointList, self.mission_wp_callback)
        self.state_sub = rospy.Subscriber('mavros/state', State,
                                          self.state_callback)

    def tearDown(self):
        self.log_topic_vars()

    #
    # Callback functions
    #
    def altitude_callback(self, data):
        self.altitude = data

        # amsl has been observed to be nan while other fields are valid
        if not self.sub_topics_ready['alt'] and not math.isnan(data.amsl):
            self.sub_topics_ready['alt'] = True

    def extended_state_callback(self, data):
        if self.extended_state.vtol_state != data.vtol_state:
            rospy.loginfo("VTOL state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE']
                [self.extended_state.vtol_state].name, mavutil.mavlink.enums[
                    'MAV_VTOL_STATE'][data.vtol_state].name))

        if self.extended_state.landed_state != data.landed_state:
            rospy.loginfo("landed state changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_LANDED_STATE']
                [self.extended_state.landed_state].name, mavutil.mavlink.enums[
                    'MAV_LANDED_STATE'][data.landed_state].name))

        self.extended_state = data

        if not self.sub_topics_ready['ext_state']:
            self.sub_topics_ready['ext_state'] = True

    def global_position_callback(self, data):
        self.global_position = data

        if not self.sub_topics_ready['global_pos']:
            self.sub_topics_ready['global_pos'] = True

    def home_position_callback(self, data):
        self.home_position = data

        if not self.sub_topics_ready['home_pos']:
            self.sub_topics_ready['home_pos'] = True

    def local_position_callback(self, data):
        self.local_position = data

        if not self.sub_topics_ready['local_pos']:
            self.sub_topics_ready['local_pos'] = True

    def mission_wp_callback(self, data):
        if self.mission_wp.current_seq != data.current_seq:
            rospy.loginfo("current mission waypoint sequence updated: {0}".
                          format(data.current_seq))

        self.mission_wp = data

        if not self.sub_topics_ready['mission_wp']:
            self.sub_topics_ready['mission_wp'] = True

    def state_callback(self, data):
        if self.state.armed != data.armed:
            rospy.loginfo("armed state changed from {0} to {1}".format(
                self.state.armed, data.armed))

        if self.state.connected != data.connected:
            rospy.loginfo("connected changed from {0} to {1}".format(
                self.state.connected, data.connected))

        if self.state.mode != data.mode:
            rospy.loginfo("mode changed from {0} to {1}".format(
                self.state.mode, data.mode))

        if self.state.system_status != data.system_status:
            rospy.loginfo("system_status changed from {0} to {1}".format(
                mavutil.mavlink.enums['MAV_STATE'][
                    self.state.system_status].name, mavutil.mavlink.enums[
                        'MAV_STATE'][data.system_status].name))

        self.state = data

        # mavros publishes a disconnected state message on init
        if not self.sub_topics_ready['state'] and data.connected:
            self.sub_topics_ready['state'] = True

    #
    # Helper methods
    #
    def set_arm(self, arm, timeout):
        """arm: True to arm or False to disarm, timeout(int): seconds"""
        rospy.loginfo("setting FCU arm: {0}".format(arm))
        old_arm = self.state.armed
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        arm_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.armed == arm:
                arm_set = True
                rospy.loginfo("set arm success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_arming_srv(arm)
                    if not res.success:
                        rospy.logerr("failed to send arm command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # self.assertTrue(arm_set, (
        #     "failed to set arm | new arm: {0}, old arm: {1} | timeout(seconds): {2}".
        #     format(arm, old_arm, timeout)))

    def set_mode(self, mode, timeout):
        """mode: PX4 mode string, timeout(int): seconds"""
        rospy.loginfo("setting FCU mode: {0}".format(mode))
        old_mode = self.state.mode
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        mode_set = False
        for i in xrange(timeout * loop_freq):
            if self.state.mode == mode:
                mode_set = True
                rospy.loginfo("set mode success | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.set_mode_srv(0, mode)  # 0 is custom mode
                    if not res.mode_sent:
                        rospy.logerr("failed to send mode command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # self.assertTrue(mode_set, (
        #     "failed to set mode | new mode: {0}, old mode: {1} | timeout(seconds): {2}".
        #     format(mode, old_mode, timeout)))

    def wait_for_topics(self, timeout):
        """wait for simulation to be ready, make sure we're getting topic info
        from all topics by checking dictionary of flag values set in callbacks,
        timeout(int): seconds"""
        rospy.loginfo("waiting for subscribed topics to be ready")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        simulation_ready = False
        for i in xrange(timeout * loop_freq):
            if all(value for value in self.sub_topics_ready.values()):
                simulation_ready = True
                rospy.loginfo("simulation topics ready | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # self.assertTrue(simulation_ready, (
        #     "failed to hear from all subscribed simulation topics | topic ready flags: {0} | timeout(seconds): {1}".
        #     format(self.sub_topics_ready, timeout)))

    def wait_for_landed_state(self, desired_landed_state, timeout, index):
        rospy.loginfo("waiting for landed state | state: {0}, index: {1}".
                      format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
                          desired_landed_state].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        landed_state_confirmed = False
        for i in xrange(timeout * loop_freq):
            if self.extended_state.landed_state == desired_landed_state:
                landed_state_confirmed = True
                rospy.loginfo("landed state confirmed | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # self.assertTrue(landed_state_confirmed, (
        #     "landed state not detected | desired: {0}, current: {1} | index: {2}, timeout(seconds): {3}".
        #     format(mavutil.mavlink.enums['MAV_LANDED_STATE'][
        #         desired_landed_state].name, mavutil.mavlink.enums[
        #             'MAV_LANDED_STATE'][self.extended_state.landed_state].name,
        #            index, timeout)))

    def wait_for_vtol_state(self, transition, timeout, index):
        """Wait for VTOL transition, timeout(int): seconds"""
        rospy.loginfo(
            "waiting for VTOL transition | transition: {0}, index: {1}".format(
                mavutil.mavlink.enums['MAV_VTOL_STATE'][
                    transition].name, index))
        loop_freq = 10  # Hz
        rate = rospy.Rate(loop_freq)
        transitioned = False
        for i in xrange(timeout * loop_freq):
            if transition == self.extended_state.vtol_state:
                rospy.loginfo("transitioned | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                transitioned = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(transitioned, (
            "transition not detected | desired: {0}, current: {1} | index: {2} timeout(seconds): {3}".
            format(mavutil.mavlink.enums['MAV_VTOL_STATE'][transition].name,
                   mavutil.mavlink.enums['MAV_VTOL_STATE'][
                       self.extended_state.vtol_state].name, index, timeout)))

    def clear_wps(self, timeout):
        """timeout(int): seconds"""
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        wps_cleared = False
        for i in xrange(timeout * loop_freq):
            if not self.mission_wp.waypoints:
                wps_cleared = True
                rospy.loginfo("clear waypoints success | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break
            else:
                try:
                    res = self.wp_clear_srv()
                    if not res.success:
                        rospy.logerr("failed to send waypoint clear command")
                except rospy.ServiceException as e:
                    rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(wps_cleared, (
            "failed to clear waypoints | timeout(seconds): {0}".format(timeout)
        ))

    def send_wps(self, waypoints, timeout):
        """waypoints, timeout(int): seconds"""
        rospy.loginfo("sending mission waypoints")
        if self.mission_wp.waypoints:
            rospy.loginfo("FCU already has mission waypoints")

        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        wps_sent = False
        wps_verified = False
        for i in xrange(timeout * loop_freq):
            if not wps_sent:
                try:
                    res = self.wp_push_srv(start_index=0, waypoints=waypoints)
                    wps_sent = res.success
                    if wps_sent:
                        rospy.loginfo("waypoints successfully transferred")
                except rospy.ServiceException as e:
                    rospy.logerr(e)
            else:
                if len(waypoints) == len(self.mission_wp.waypoints):
                    rospy.loginfo("number of waypoints transferred: {0}".
                                  format(len(waypoints)))
                    wps_verified = True

            if wps_sent and wps_verified:
                rospy.loginfo("send waypoints success | seconds: {0} of {1}".
                              format(i / loop_freq, timeout))
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue((
            wps_sent and wps_verified
        ), "mission could not be transferred and verified | timeout(seconds): {0}".
                        format(timeout))

    def wait_for_mav_type(self, timeout):
        """Wait for MAV_TYPE parameter, timeout(int): seconds"""
        rospy.loginfo("waiting for MAV_TYPE")
        loop_freq = 1  # Hz
        rate = rospy.Rate(loop_freq)
        res = False
        for i in xrange(timeout * loop_freq):
            try:
                res = self.get_param_srv('MAV_TYPE')
                if res.success:
                    self.mav_type = res.value.integer
                    rospy.loginfo(
                        "MAV_TYPE received | type: {0} | seconds: {1} of {2}".
                        format(mavutil.mavlink.enums['MAV_TYPE'][self.mav_type]
                               .name, i / loop_freq, timeout))
                    break
            except rospy.ServiceException as e:
                rospy.logerr(e)

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        self.assertTrue(res.success, (
            "MAV_TYPE param get failed | timeout(seconds): {0}".format(timeout)
        ))

    def log_topic_vars(self):
        """log the state of topic variables"""
        rospy.loginfo("========================")
        rospy.loginfo("===== topic values =====")
        rospy.loginfo("========================")
        rospy.loginfo("altitude:\n{}".format(self.altitude))
        rospy.loginfo("========================")
        rospy.loginfo("extended_state:\n{}".format(self.extended_state))
        rospy.loginfo("========================")
        rospy.loginfo("global_position:\n{}".format(self.global_position))
        rospy.loginfo("========================")
        rospy.loginfo("home_position:\n{}".format(self.home_position))
        rospy.loginfo("========================")
        rospy.loginfo("local_position:\n{}".format(self.local_position))
        rospy.loginfo("========================")
        rospy.loginfo("mission_wp:\n{}".format(self.mission_wp))
        rospy.loginfo("========================")
        rospy.loginfo("state:\n{}".format(self.state))
        rospy.loginfo("========================")


    def setUp2(self,acc):

        self.pos = PoseStamped()
        self.radius = acc

        self.pos_setpoint_pub = rospy.Publisher(
            'mavros/setpoint_position/local', PoseStamped, queue_size=1)

        # send setpoints in seperate thread to better prevent failsafe
        self.pos_thread = Thread(target=self.send_pos, args=())
        self.pos_thread.daemon = True
        self.pos_thread.start()

    def tearDown(self):
        super(missionfly, self).tearDown()

    #
    # Helper methods
    #
    def send_pos(self):
        rate = rospy.Rate(10)  # Hz
        self.pos.header = Header()
        self.pos.header.frame_id = "base_footprint"

        while not rospy.is_shutdown():
            self.pos.header.stamp = rospy.Time.now()
            self.pos_setpoint_pub.publish(self.pos)
            try:  # prevent garbage in console output when thread is killed
                rate.sleep()
            except rospy.ROSInterruptException:
                pass

    def is_at_position(self, x, y, z, offset):
        """offset: meters"""
        rospy.logdebug(
            "current position | x:{0:.2f}, y:{1:.2f}, z:{2:.2f}".format(
                self.local_position.pose.position.x, self.local_position.pose.
                position.y, self.local_position.pose.position.z))

        desired = np.array((x, y, z))
        pos = np.array((self.local_position.pose.position.x,
                        self.local_position.pose.position.y,
                        self.local_position.pose.position.z))
        return np.linalg.norm(desired - pos) < offset

    def reach_position(self, x, y, z, timeout):
        """timeout(int): seconds"""
        # set a position setpoint
        self.pos.pose.position.x = x
        self.pos.pose.position.y = y
        self.pos.pose.position.z = z
        rospy.loginfo(
            "attempting to reach position | x: {0}, y: {1}, z: {2} | current position x: {3:.2f}, y: {4:.2f}, z: {5:.2f}".
            format(x, y, z, self.local_position.pose.position.x,
                   self.local_position.pose.position.y,
                   self.local_position.pose.position.z))

        # For demo purposes we will lock yaw/heading to north.
        yaw_degrees = 0  # North
        yaw = math.radians(yaw_degrees)
        quaternion = quaternion_from_euler(0, 0, yaw)
        self.pos.pose.orientation = Quaternion(*quaternion)

        # does it reach the position in 'timeout' seconds?
        loop_freq = 2  # Hz
        rate = rospy.Rate(loop_freq)
        reached = False
        for i in xrange(timeout * loop_freq):
            if self.is_at_position(self.pos.pose.position.x,
                                   self.pos.pose.position.y,
                                   self.pos.pose.position.z, self.radius):
                rospy.loginfo("position reached | seconds: {0} of {1}".format(
                    i / loop_freq, timeout))
                reached = True
                break

            try:
                rate.sleep()
            except rospy.ROSException as e:
                self.fail(e)

        # self.assertTrue(reached, (
        #     "took too long to get to position | current position x: {0:.2f}, y: {1:.2f}, z: {2:.2f} | timeout(seconds): {3}".
        #     format(self.local_position.pose.position.x,
        #            self.local_position.pose.position.y,
        #            self.local_position.pose.position.z, timeout)))

    #
    # Test method
    #
    def test_posctl(self,positions):
        """Test offboard position control"""

        # make sure the simulation is ready to start the mission
        self.wait_for_topics(60)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   10, -1)

        self.log_topic_vars()
        self.set_mode("OFFBOARD", 5)
        self.set_arm(True, 5)

        rospy.loginfo("run mission")
        # positions = ((0, 0, 0), (50, 50, 20), (50, -50, 20), (-50, -50, 20),
        # #              (0, 0, 20))
        # positions = ((0, 0, 0), (5, 5, 1), (5, -5, 1), (-5, -5, 1),
        #              (-5, 5, 1),(0, 0, 1),(0, 0, 0))


        for i in xrange(len(positions)):
            self.reach_position(positions[i][0], positions[i][1],
                                positions[i][2], 10)

        self.set_mode("AUTO.LAND", 2)
        self.wait_for_landed_state(mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND,
                                   15, 0)
        self.set_arm(False, 2)


# obstacle_map()
start = (0,0,0)
goal_x, goal_y, goal_z = (20,10,6)
visited_nodes = set()
all_nodes = []
parent_list = []
seed_list = []
parent_dict = {}
cost_dict = {}

visited_nodes.add(start)
all_nodes.append(start)
parent_dict[start] = "okay"
cost_dict[start] = 0

seed = (0,0,0)
print("\n")

i = 0
while(goalcheck_circle(goal_x, goal_y, goal_z, seed[0], seed[1], seed[2]) == False):
    seed = generate_seed()

    if ((seed not in visited_nodes) and not obstacle_check(seed[0], seed[1], seed[2])):
        
        X = np.asarray(all_nodes)
        tree = spatial.KDTree(X)
        
        r = 2
        n = (0,0,0)

        while(1):
            n = neighbours(seed, r,tree)
            if(n == seed).all():
                r = r + 1
            else:
                break        
        
        for pt in n:
            pt = tuple(pt)
            cost = cost_dict[pt]
            cost_new = cost + cost2go(pt, seed)
            q.put((cost_new, pt, cost))
        
        parent = q.get()
        q = PriorityQueue() 
        parent = parent[1] 
                     
        if (cost2go(parent,seed) > 4):
            seed = max_step_prop(parent, seed)
            seed = (round(seed[0], 1), round(seed[1], 1), round(seed[2], 1))
            
            
        par = seed
        s = parent
        a = 0

        while(cost2go(par,s)>=0.1):
            a = line_obstacle_check(s, par)
            if obstacle_check(a[0], a[1], a[2]):
                break
            s = a

        s = (round(s[0], 1), round(s[1], 1), round(s[2], 1))
         
        if s not in visited_nodes:
            neww_cost = cost2go(seed, parent) + cost_dict[parent]  
            all_nodes.insert(0, s)
            visited_nodes.add(s)
            parent_dict[s] = parent 
            cost_dict[s] = neww_cost
            parent_list.append((parent[0], parent[1], parent[2]))
            seed_list.append((s[0], s[1], s[2]))
            # ax.plot3D((parent[0],s[0]), (parent[1], s[1]), (parent[2], s[2]), 'black')

            for nei in n:
                nei = tuple(nei)
                if nei != parent:
                    if cost_dict[nei] > (cost_dict[s] + cost2go(s, nei)):
                        parent_dict[nei] = s
                        cost_dict[nei] = cost_dict[s] + cost2go(s, nei)
        else:
            all_nodes.pop(0)


path = []
path.append((s[0], s[1], s[2]))
while parent != 'okay':
    temp = parent_dict.get(parent)
    path.append(parent)
    parent = temp
    if parent == (start):
        break
path.append(start)
print("Backtracking done - shortest path found")

path = path[::-1]
# oroginal_path = path.copy()
oroginal_path = copy.deepcopy(path)


for i in range(100):
    choice1 = random.choice(path)
    choice2 = random.choice(path)
    ch1 = choice1
    ch2 = choice2
                    
    ind1 = path.index(choice1)
    ind2 = path.index(choice2)

    if (choice1 != choice2):
        
        while(cost2go(choice2, choice1)>=0.1):
            a = line_obstacle_check(choice1, choice2)
            if obstacle_check(a[0], a[1], a[2]):
                break
            choice1 = a
        choice2 = ch1
    if (cost2go(choice1, ch2)) < 0.2:
        if ind1< ind2:
            del path[ind1+1:ind2]

        if ind2 < ind1:
            del path[ind2+1:ind1]


    print("after_optimise")
    print("choice1", choice2)
    print("choice2", choice1)

    print("\n")

            
fig = plt.figure()
ax = plt.axes(projection='3d')

ax.set_xlim3d(0, 20)
ax.set_ylim3d(0,15)
ax.set_zlim3d(0,10)
obstacle_map()

    

x_path_original = [oroginal_path[i][0] for i in range(len(oroginal_path))]
y_path_original = [oroginal_path[i][1] for i in range(len(oroginal_path))]
z_path_original = [oroginal_path[i][2] for i in range(len(oroginal_path))]

x_path = [path[i][0] for i in range(len(path))]
y_path = [path[i][1] for i in range(len(path))]
z_path = [path[i][2] for i in range(len(path))]

ax.scatter(x_path, y_path, z_path, c='g', marker = 'o')
ax.plot3D(x_path, y_path, z_path, "-r")
# Uncomment next line to see the plotting of the path before optimisation
ax.plot3D(x_path_original, y_path_original, z_path_original, "-g")


# Uncomment below lines to see the real time exploration of the tree
# l = 0
# while l < len(seed_list):
#     ax.plot3D((parent_list[l][0], seed_list[l][0]), (parent_list[l][1], seed_list[l][1]), (parent_list[l][2], seed_list[l][2]),  'black')
#     l = l + 1
#     plt.show()
#     plt.pause(0.000000000000000000000000000000000001)


print("before", oroginal_path)
print("after", path)
ax.set_xlabel('x-axis') 
ax.set_ylabel('y-axis') 
ax.set_zlabel('z-axis') 
plt.show()
plt.pause(5)
plt.savefig("output.png")
plt.close()

rospy.init_node('test_node1', anonymous=True)

# rostest.rosrun('fly', 'missionfly',missionfly)


accuracy=0.3

# positions = [(0, 0, 0), (1.9, 0.4, 0.5), (3.7, 0.5, 1.4), (5.2, 1.6, 2.0), (6.4, 3.0, 1.9), (6.5, 3.5, 2.0), (7.0, 4.0, 3.0), (8.0, 5.0, 3.5), (8.5, 5.0, 3.5), (9.5, 5.5, 4.0), (10.0, 6.5, 4.5), (11.0, 7.0, 5.0), (12.0, 7.0, 5.0), (12.5, 7.5, 5.0), (13.0, 8.5, 4.5), (14.0, 9.0, 4.5), (15.0, 9.5, 5.0), (16.0, 10.0, 5.5), (16.4, 9.8, 5.9), (17.4, 11.1, 6.9), (17.5, 12.5, 7.0), (18.0, 12.5, 7.4), (18.9, 13.0, 8.0), (19.0, 14.0, 9.0), (19.5, 15.0, 10.0)]
# positions = [(0, 0, 0),(19.5, 15.0, 10.0)]



quadpos1=[]
for i in oroginal_path:
    quadpos1.append((-i[1],i[0],i[2]))
quadpos2=[]
for i in path:
    quadpos2.append((-i[1],i[0],i[2]))





drone1=missionfly()
drone1.setUp()
drone1.setUp2(accuracy)
drone1.set_arm(True,10)
drone1.set_mode("OFFBOARD",50)
drone1.test_posctl(quadpos1)#Un pruned path
#drone1.test_posctl(quadpos2)# pruned path

