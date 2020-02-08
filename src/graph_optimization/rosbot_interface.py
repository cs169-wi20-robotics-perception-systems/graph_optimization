import rospy
import rospkg
import message_filters
import math
import numpy as np

import file_write_lib
import plot_lib
from pose_graph_optimization import PoseGraphOptmization

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan

class RosbotInterface:
    def __init__(self):
        self.g2o_graph_optimization = PoseGraphOptmization()
        self.vertex_id = 0

        # Sensor covariance values
        SCAN_COVARIANCE = 4.5  # Default
        POSE_COVARIANCE = 5.5  # Default
        # User may set covariance values
        if rospy.has_param('scan_covariance'):
            SCAN_COVARIANCE = rospy.get_param('scan_covariance')
            rospy.loginfo("Rosparam 'scan_covariance' set to: " + str(SCAN_COVARIANCE))
        if rospy.has_param('pose_covariance'):
            POSE_COVARIANCE = rospy.get_param('pose_covariance')
            rospy.loginfo("Rosparam 'pose_covariance' set to: " + str(POSE_COVARIANCE))
        # Set up sensor information matrices
        scan_covariance_matrix = np.identity(3)
        scan_covariance_matrix[0] = SCAN_COVARIANCE
        self.SCAN_INFORMATION_MATRIX = np.linalg.inv(scan_covariance_matrix)
        pose_covariance_matrix = np.identity(3)
        pose_covariance_matrix[0] = POSE_COVARIANCE
        self.POSE_INFORMATION_MATRIX = np.linalg.inv(pose_covariance_matrix)

        # Use measurements directly in front of the robot.
        self.SCAN_ANGLE_LEFT = math.pi / 12.0           # Default
        self.SCAN_ANGLE_RIGHT = 11.0 * math.pi / 12.0   # Default
        # User may set angle values
        if rospy.has_param('scan_angle_left'):
            self.SCAN_ANGLE_LEFT = rospy.get_param('scan_angle_left')
            rospy.loginfo("Rosparam 'scan_angle_left' set to: " + str(self.SCAN_ANGLE_LEFT))
        if rospy.has_param('scan_angle_right'):
            self.SCAN_ANGLE_RIGHT = rospy.get_param('scan_angle_right')
            rospy.loginfo("Rosparam 'scan_angle_right' set to: " + str(self.SCAN_ANGLE_RIGHT))

        # Parameters for calculating new vertices and edges.
        self.initial_vertex = True
        self.initial_pose = 0.36
        self.prev_scan_reading = 0.0
        self.prev_pose_reading = 0.0
        # User may set initial pose of the robot
        if rospy.has_param('initial_pose'):
            wait_for_initial_pose = rospy.get_param('initial_pose')
            if wait_for_initial_pose:
                rospy.Subscriber("initialpose", PoseWithCovarianceStamped, self.initial_pose_callback)
                rospy.wait_for_message("initialpose", PoseWithCovarianceStamped)

        # Set up file name paths for graphs and plots
        rospack = rospkg.RosPack()
        self.data_file_path = rospack.get_path('graph_optimization') + "/sample_data/"
        self.original_graph_file_name = self.data_file_path + "original_graph.g2o"
        self.optimized_graph_file_name = self.data_file_path + "optimized_graph.g2o"
        # Clear contents of file
        open(self.original_graph_file_name, 'w').close()

        # Arrays for storing pose estimates; Used for plotting.
        self.truth_x = np.array([0.0, 0.97])
        # User may set ground truth final value
        if rospy.has_param('ground_truth_end'):
            self.truth_x[1] = rospy.get_param('ground_truth_end')
            rospy.loginfo("Rosparam 'ground_truth_end' set to: " + str(self.truth_x[1]))
        self.odom_x = []
        self.odom_time = []
        self.optimized_x = []
        self.state_plot_file_name = self.data_file_path + "state_plot.svg"
        self.error_plot_file_name = self.data_file_path + "error_plot.svg"

        rospy.on_shutdown(self.shutdown)

        # Required as messages in pose topic have no time stamp
        rospy.set_param('use_sim_time', True)
        rospy.Subscriber('pose', PoseStamped, self.pose_callback)
        rospy.wait_for_message('pose', PoseStamped)

        scan_sub = message_filters.Subscriber('scan', LaserScan)
        pose_sub = message_filters.Subscriber('pose', PoseStamped)
        ts = message_filters.ApproximateTimeSynchronizer([scan_sub, pose_sub], 15, 0.1)
        ts.registerCallback(self.scan_pose_callback)


    def pose_callback(self, msg):
        """
        Callback for setting time stamps on pose messages.

        Args:
            msg: PoseStamped msg that has the current robot's position and orientation.
        """
        msg.header.stamp.secs = rospy.Time.now().secs


    def scan_pose_callback(self, scan_msg, pose_msg):
        """
        Callback for retrieving the laser scan and odometry data. Calls functions to create
        new vertices and edges according to the information from the messages.

        Args:
            scan_msg: LaserScan msg that has a list of scan measurements.
            pose_msg: PoseStamped msg that has the current robot's position and orientation.
        """
        # Calculate average measurement in scan readings
        avg_scan_distance = 0.0
        num_scan_points = 0.0
        cur_angle = 0.0

        # Use data between SCAN_ANGLE_LEFT and SCAN_ANGLE_RIGHT
        for index in range(len(scan_msg.ranges)):
            if ((cur_angle <= self.SCAN_ANGLE_LEFT) or (cur_angle >= self.SCAN_ANGLE_RIGHT)) and (scan_msg.ranges[index] < scan_msg.range_max):
                avg_scan_distance = avg_scan_distance + scan_msg.ranges[index]
                num_scan_points = num_scan_points + 1
            cur_angle = cur_angle + scan_msg.angle_increment
        avg_scan_distance = avg_scan_distance / num_scan_points

        # Add information to array, which will be used later for plotting
        self.odom_x.append(pose_msg.pose.position.x - self.initial_pose)
        self.odom_time.append(rospy.Time.now().to_sec())

        # Create vertex
        cur_vertex = np.array([pose_msg.pose.position.x - self.initial_pose, 0.0, 0.0])
        self.g2o_graph_optimization.add_vertex(self.vertex_id, cur_vertex)

        # Write vertex of original graph to file
        file_write_lib.append_vertex(cur_vertex, self.vertex_id, self.original_graph_file_name)

        if self.initial_vertex:
            self.initial_vertex = False
        else:
            # Calculate change in scan and in pose
            change_in_scan = np.array([self.prev_scan_reading - avg_scan_distance, 0.0, 0.0])
            change_in_pose = np.array([pose_msg.pose.position.x - self.prev_pose_reading, 0.0, 0.0])

            # Create edges between current vertex and previous vertex
            self.g2o_graph_optimization.add_edge((self.vertex_id-1, self.vertex_id), change_in_scan, self.SCAN_INFORMATION_MATRIX)
            self.g2o_graph_optimization.add_edge((self.vertex_id-1, self.vertex_id), change_in_pose, self.POSE_INFORMATION_MATRIX)

            # Write edges of original graph to file
            file_write_lib.append_edge(self.vertex_id-1, self.vertex_id, change_in_scan, self.SCAN_INFORMATION_MATRIX, self.original_graph_file_name)
            file_write_lib.append_edge(self.vertex_id-1, self.vertex_id, change_in_pose, self.POSE_INFORMATION_MATRIX, self.original_graph_file_name)

        # Update parameters
        self.prev_scan_reading = avg_scan_distance
        self.prev_pose_reading = pose_msg.pose.position.x
        self.vertex_id = self.vertex_id + 1


    def initial_pose_callback(self, msg):
        """
        Callback for retrieving the robot's initial state.

        Args:
            msg: PoseWithCovarianceStamped msg that has the current pose, orientation, and covariance matrix.
        """
        self.initial_pose = msg.pose.pose.position.x

        rospy.loginfo("Rosparam 'initial_pose' set to: " + str(self.initial_pose))


    def shutdown(self):
        """
        Shutdown function that optimizes the graph, writes the graph to file,
        and plots the results before killing the node.
        """
        # Must have at least 2 vertices in the graph
        if 1 <= self.vertex_id:
            rospy.loginfo("Optimize graph and write data to file.")
            self.g2o_graph_optimization.optimize()

            file_write_lib.write_optimized_graph(self.g2o_graph_optimization, self.vertex_id - 1, self.optimized_graph_file_name)

            for i in range(self.vertex_id):
                self.optimized_x.append(self.g2o_graph_optimization.get_pose(i)[0])

            plot_lib.plot_state(self.truth_x, self.odom_x, self.optimized_x, self.odom_time, self.state_plot_file_name)
            plot_lib.plot_error(self.truth_x[-1], self.odom_x[-1], self.optimized_x[-1], self.error_plot_file_name)

        rospy.loginfo("Shutting down.")
