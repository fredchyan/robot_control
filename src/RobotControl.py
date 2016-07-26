#!/usr/bin/env python
"""
ROS based interface for the Course Robotics Specialization Capstone Autonomous Rover.
Updated June 15 2016.
"""
import rospy

import yaml
import numpy as np
import math
from numpy.linalg import inv

import sys

from RosInterface import ROSInterface

# User files, uncomment as completed
from ShortestPath import dijkstras
from KalmanFilter import KalmanFilter
from DiffDriveController import DiffDriveController

class RobotControl(object):
    """
    Class used to interface with the rover. Gets sensor measurements through ROS subscribers,
    and transforms them into the 2D plane, and publishes velocity commands.
    """
    def __init__(self, world_map,occupancy_map, pos_init, pos_goal, max_speed, max_omega, x_spacing, y_spacing, t_cam_to_body):
        """
        Initialize the class
        """

        # Handles all the ROS related items
        self.ros_interface = ROSInterface(t_cam_to_body)

        # YOUR CODE AFTER THIS
        
        # Uncomment as completed
        self.markers = world_map
        self.vel = np.array([0, 0])
        self.imu_meas = np.array([])
        self.meas = []
        
        # TODO for student: Use this when transferring code to robot
        # Handles all the ROS related items
        #self.ros_interface = ROSInterface(t_cam_to_body)

        # YOUR CODE AFTER THIS
        
        # Uncomment as completed
        self.goals = dijkstras(occupancy_map, x_spacing, y_spacing, pos_init, pos_goal)
        # self.total_goals = self.goals.shape[0]
        self.cur_goal = 2
        self.end_goal = self.goals.shape[0] - 1
        self.kalman_filter = KalmanFilter(world_map)
        self.diff_drive_controller = DiffDriveController(max_speed, max_omega)

    def process_measurements(self):
        """ 
        YOUR CODE HERE
        This function is called at 60Hz
        """
        meas = self.ros_interface.get_measurements()
        self.meas = meas;
        print 'tag output'
        print meas
        # imu_meas: measurment comig from the imu
        imu_meas = self.ros_interface.get_imu()
        self.imu_meas = imu_meas
        print 'imu measurement'
        print imu_meas
        pose = self.kalman_filter.step_filter(self.vel, self.imu_meas, np.asarray(self.meas))
        

        # Code to follow AprilTags
        '''
        if(meas != None and meas):
            cur_meas = meas[0]
            tag_robot_pose = cur_meas[0:3]
            tag_world_pose = self.tag_pos(cur_meas[3])
            state = self.robot_pos(tag_world_pose, tag_robot_pose)
            goal = tag_world_pose
            vel = self.diff_drive_controller.compute_vel(state, goal)        
            self.vel = vel[0:2];
            print vel
            if(not vel[2]):
                self.ros_interface.command_velocity(vel[0], vel[1])
            else:
                vel = (0.01, 0.1) 
                self.vel = vel
                self.ros_interface.command_velocity(vel[0], vel[1])
        '''


        # Code to move autonomously
        goal = self.goals[self.cur_goal]
        print 'pose'
        print pose
        print 'goal'
        print goal
        vel = self.diff_drive_controller.compute_vel(pose, goal)        
        self.vel = vel[0:2];
        print 'speed'
        print vel
        if(not vel[2]):
            self.ros_interface.command_velocity(vel[0], vel[1])
        else:
            vel = (0, 0) 
            if self.cur_goal < self.end_goal:
                self.cur_goal = self.cur_goal + 1
            self.ros_interface.command_velocity(vel[0], vel[1])
            self.vel = vel
        return

    def tag_pos(self, marker_id):
        for i in range(len(self.markers)):
            marker_i = np.copy(self.markers[i])
            if marker_i[3] == marker_id:
                return marker_i[0:3]
        return None

    def robot_pos(self, w_pos, r_pos):
        H_W = np.array([[math.cos(w_pos[2]), -math.sin(w_pos[2]), w_pos[0]],
                        [math.sin(w_pos[2]), math.cos(w_pos[2]), w_pos[1]],
                        [0, 0, 1]])
        H_R = np.array([[math.cos(r_pos[2]), -math.sin(r_pos[2]), r_pos[0]],
                        [math.sin(r_pos[2]), math.cos(r_pos[2]), r_pos[1]],
                        [0, 0, 1]])
        w_r = H_W.dot(inv(H_R))
        robot_pose =  np.array([[w_r[0,2]], [w_r[1,2]], [math.atan2(w_r[1,0], w_r[0, 0])]])
        return robot_pose

    
def main(args):
    rospy.init_node('robot_control')

    # Load parameters from yaml
    param_path = rospy.get_param("~param_path")
    f = open(param_path,'r')
    params_raw = f.read()
    f.close()
    params = yaml.load(params_raw)
    occupancy_map = np.array(params['occupancy_map'])
    world_map = np.array(params['world_map'])
    pos_init = np.array(params['pos_init'])
    pos_goal = np.array(params['pos_goal'])
    max_vel = params['max_vel']
    max_omega = params['max_omega']
    t_cam_to_body = np.array(params['t_cam_to_body'])
    x_spacing = params['x_spacing']
    y_spacing = params['y_spacing']

    # Intialize the RobotControl object
    robotControl = RobotControl(world_map,occupancy_map, pos_init, pos_goal, max_vel, max_omega, x_spacing, y_spacing, t_cam_to_body)

    # Call process_measurements at 60Hz
    r = rospy.Rate(60)
    #start_time = rospy.get_time()
    #while not (rospy.is_shutdown() or (rospy.get_time() - start_time) > 1):
    while not (rospy.is_shutdown()):
        robotControl.process_measurements()
        r.sleep()
    # Done, stop robot
    robotControl.ros_interface.command_velocity(0,0)

if __name__ == "__main__":
    try:
        main(sys.argv)
    except rospy.ROSInterruptException: pass


