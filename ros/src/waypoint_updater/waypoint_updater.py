#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32

import tf

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        #rospy.Subscriber('/obstacle_waypoint', Int32, self.obstacle_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
	self.current_pose = PoseStamped()
	self.base_waypoints = Lane()
	self.traffic_waypoints = Int32()


        rospy.spin()

    def closest_node(self):
	i_closest= -1
	dist_closest = 1000000000
	dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
	for i,waypoint in enumerate(self.base_waypoints.waypoints):
		distance = dl(waypoint.pose.pose.position, self.current_pose.pose.position)
		if distance < dist_closest:
			dist_closest = distance
			i_closest = i
	return i_closest


    def heading_tonode(self, waypoint):
	#rospy.logwarn('checking out node %s', waypoint)
	x_w = self.base_waypoints.waypoints[waypoint].pose.pose.position.x
	y_w = self.base_waypoints.waypoints[waypoint].pose.pose.position.y
	x_c = self.current_pose.pose.position.x
	y_c = self.current_pose.pose.position.y
	return math.atan2(y_w-y_c,x_w-x_c)

    def next_node(self):
	closestWaypoint = self.closest_node()
	heading = self.heading_tonode(closestWaypoint)
        (r, p, y) = tf.transformations.euler_from_quaternion([self.current_pose.pose.orientation.x, self.current_pose.pose.orientation.y, self.current_pose.pose.orientation.z, self.current_pose.pose.orientation.w])
	angle = abs(y - heading)
	if angle > math.pi/4.0:
		closestWaypoint += 1
	if closestWaypoint == len(self.base_waypoints.waypoints):
		closestWaypoint = 0
	return closestWaypoint

    def pose_cb(self, msg):
        # TODO: Implement
	self.current_pose = msg
	if len(self.base_waypoints.waypoints)>0:
		nextWaypoint = self.next_node()
		final_waypoints = Lane()
		for i in range(LOOKAHEAD_WPS):
			waypoint = nextWaypoint + i
			if waypoint >= len(self.base_waypoints.waypoints):
				waypoint -= len(self.base_waypoints.waypoints)
			waypoint_from_base = self.base_waypoints.waypoints[waypoint]
			waypoint_from_base.twist.twist.linear.x = 5
			final_waypoints.waypoints.extend([waypoint_from_base])
		self.final_waypoints_pub.publish(final_waypoints)
        pass

    def waypoints_cb(self, waypoints):
        # TODO: Implement
	#rospy.logwarn('Initial waypoints %s', len(waypoints.waypoints))
	self.base_waypoints = waypoints
	#rospy.logwarn('Copied waypoints %s', len(self.base_waypoints.waypoints))

        pass

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
	self.traffic_waypoints
        pass

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
