#!/usr/bin/env python

# Baxter Pipetting 

# Bruce Stracener - University of Arkansas for Medical Sciences
# started 01/22/18

# IK service code usage based upon IK_Pick_and_Place_Demo from Rethink Robotics

import rospy                # ROS python API
import baxter_interface     # Baxter Python API
import argparse             # Parse command-line arguments
import sys                  # System-specific parameters and functions
import copy                 # Allows deep & shallow copying of mutable objects
import struct               # Convert between strings and binary data

from geometry_msgs.msg import (   
    PoseStamped,                  
    Pose,                   # Allows use of the message types which
    Point,                  # Baxter uses to publish his pose information 
    Quaternion,
)
from std_msgs.msg import (
    Header,                 # Used to communicate timestamped data
    Empty,                  # Allows for null msgs (void in C/C++ terms)
)
from baxter_core_msgs.srv import (
    SolvePositionIK,        # Msg type returned from IK service w/ joint angles
    SolvePositionIKRequest, # Msg type sent to IK service w/ cartesian coordinates
)

class PickAndPlace(object):
    def __init__(self, limb, speed = 1.5, hover_distance = 0.15, verbose=True):   # fix the damned speed
        self._limb_name = limb                          # string
        self._hover_distance = hover_distance           # in meters above
        self._verbose = verbose                         # bool
        self._limb = baxter_interface.Limb(limb)        # selects limb
        self._limb.set_joint_position_speed(speed)      # sets overall speed
        self._gripper = baxter_interface.Gripper(limb)  # sets gripper left/right
        self._gripper.set_moving_force(100)             # sets max gripper moving force 
        ns = "ExternalTools/" + limb + "/PositionKinematicsNode/IKService"
        self._iksvc = rospy.ServiceProxy(ns, SolvePositionIK)
        rospy.wait_for_service(ns, 5.0)
        # verify robot is enabled
        print("Getting robot state... ")
        self._rs = baxter_interface.RobotEnable(baxter_interface.CHECK_VERSION)
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def move_to_start(self, start_angles=None):
        print 'move_to_start'   # for debugging
        print("Moving the {0} arm to start pose...".format(self._limb_name))
        if not start_angles:
            start_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(start_angles)
        self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def move_to_rest(self, rest_angles=None):
        print 'move_to_rest'   # for debugging
        print("Moving the {0} arm to resting pose...".format(self._limb_name))
        if not rest_angles:
            rest_angles = dict(zip(self._joint_names, [0]*7))
        self._guarded_move_to_joint_position(rest_angles)
        #self.gripper_open()
        rospy.sleep(1.0)
        print("Running. Ctrl-c to quit")

    def ik_request(self, pose):
        print 'ik_request'      # for debugging
        hdr = Header(stamp=rospy.Time.now(), frame_id='base')
        ikreq = SolvePositionIKRequest()
        ikreq.pose_stamp.append(PoseStamped(header=hdr, pose=pose))
        try:
            resp = self._iksvc(ikreq)
        except (rospy.ServiceException, rospy.ROSException), e:
            rospy.logerr("Service call failed: %s" % (e,))
            return False
        # Check if result valid, and type of seed ultimately used to get solution
        # convert rospy's string representation of uint8[]'s to int's
        resp_seeds = struct.unpack('<%dB' % len(resp.result_type), resp.result_type)
        limb_joints = {}
        if (resp_seeds[0] != resp.RESULT_INVALID):
            seed_str = {
                        ikreq.SEED_USER: 'User Provided Seed',
                        ikreq.SEED_CURRENT: 'Current Joint Angles',
                        ikreq.SEED_NS_MAP: 'Nullspace Setpoints',
                       }.get(resp_seeds[0], 'None')
            if self._verbose:       # if TRUE
                print("IK Solution SUCCESS - Valid Joint Solution Found from Seed Type: {0}".format(
                         (seed_str)))
            # Format solution into Limb API-compatible dictionary
            limb_joints = dict(zip(resp.joints[0].name, resp.joints[0].position))
            if self._verbose:
                print("IK Joint Solution:\n{0}".format(limb_joints))
                print("------------------")
        else:
            rospy.logerr("INVALID POSE - No Valid Joint Solution Found.")
            return False
        return limb_joints

    def _guarded_move_to_joint_position(self, joint_angles):
        print '_guarded_move_to_joint_position'     # for debugging
        if joint_angles:
            self._limb.move_to_joint_positions(joint_angles)
        else:
            rospy.logerr("No Joint Angles provided for move_to_joint_positions. Staying put.")
    def gripper_open(self):
        print 'gripper_open'    # for debugging
        rospy.sleep(1.0)        # Pause before opening gripper
        self._gripper.open()
        rospy.sleep(1.0)
 
    def gripper_close(self):
        print 'gripper_close'   # for debugging
        rospy.sleep(1.0)        # Pause before closing gripper
        self._gripper.close()
        rospy.sleep(1.0)

    def _approach(self, pose):
        print '_approach'       # for debugging
        approach = copy.deepcopy(pose)
        # approach with a pose the hover-distance above the requested pose
        # by making a deep copy of the pose object (so as not to alter the
        # original) and adding arbitrarily to the z coordinate
        approach.position.z = approach.position.z + self._hover_distance
        joint_angles = self.ik_request(approach)
        self._guarded_move_to_joint_position(joint_angles)
        
    def _retract(self):
        print '_retract'
        # retrieve current pose from endpoint
        current_pose = self._limb.endpoint_pose()
        ik_pose = Pose()
        ik_pose.position.x = current_pose['position'].x
        ik_pose.position.y = current_pose['position'].y
        ik_pose.position.z = current_pose['position'].z + self._hover_distance
        ik_pose.orientation.x = current_pose['orientation'].x
        ik_pose.orientation.y = current_pose['orientation'].y
        ik_pose.orientation.z = current_pose['orientation'].z
        ik_pose.orientation.w = current_pose['orientation'].w
        joint_angles = self.ik_request(ik_pose)
        # servo up from current pose
        self._guarded_move_to_joint_position(joint_angles)
        # test this later using deepcopy as above
 
    def _servo_to_pose(self, pose):
        print '_servo_to_pose'
        # servo down to release
        joint_angles = self.ik_request(pose)
        self._guarded_move_to_joint_position(joint_angles)
 
    def pick(self, pose):       # generic code to pick up an object
        print 'pick'
        # open the gripper
        self.gripper_open()
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # close gripper
        self.gripper_close()
        # retract to clear object
        self._retract()             
 
    def place(self, pose):      # generic code to place an object
        print 'place'
        # servo above pose
        self._approach(pose)
        # servo to pose
        self._servo_to_pose(pose)
        # open the gripper
        self.gripper_open()
        # retract to clear object
        self._retract()

    def get_solution(self,pose):
        print '*********************************************get_solution***********************'
        self.gripper_close()
        self._approach(pose)
        self._limb.set_joint_position_speed(0.10)  # don't hardcode
        rospy.sleep(0.5)
        self._servo_to_pose(pose)
        self.gripper_open()
        self._limb.set_joint_position_speed(0.05)  # don't hardcode
        self._retract()
        self._limb.set_joint_position_speed(1.00)

    def leave_solution(self, pose):
        print '*********************************************leave_solution*********************'
        self._approach(pose)
        self._limb.set_joint_position_speed(0.10)  # don't hardcode
        self._servo_to_pose(pose)
        self.gripper_close()
        self._limb.set_joint_position_speed(0.05)  # don't hardcode                
        self._retract()
        self._limb.set_joint_position_speed(1.00)

    def remove_lid(self,pose):
        print '*********************************************remove_lid************************'
        self._limb.set_joint_position_speed(0.50)
        rospy.sleep(0.5)
        self._approach(pose)
        self._limb.set_joint_position_speed(0.05)  # don't hardcode
        rospy.sleep(0.5)
        self._servo_to_pose(pose)
        self.gripper_close()
        self._limb.set_joint_position_speed(0.05)  # don't hardcode                
        rospy.sleep(0.5)
        self._retract()
        self._limb.set_joint_position_speed(0.45)

    def bye_lid(self,pose):
        rospy.sleep(0.5)
        self._limb.set_joint_position_speed(0.50)
        self._approach(pose)
        self._limb.set_joint_position_speed(0.15)  # don't hardcode
        rospy.sleep(0.5)
        self._servo_to_pose(pose)
        self.gripper_open()
        self._retract()
        rospy.sleep(3.0)


def main():
    # Uses IK service to return joint angles for a requested Cartesian Pose.
    # Currently open loop. Improve later with perception and feedback to close.
    rospy.init_node("pick_and_place", anonymous=True)
 
    limb = 'left'

    hover_distance = 0.025

    rospy.sleep(0.5)

    resting_joint_angles = {'left_w0': 1.4097283440666952,
                            'left_w1': 0.9767622666860372,
                            'left_w2': -0.24735440204652295,
                            'left_e0': -1.4534467965214295,
                            'left_e1': 2.603165397041547,
                            'left_s0': 1.1408982109897765,
                            'left_s1': -0.6841554313968945}

    pnp = PickAndPlace(limb, hover_distance)
     
    vacuum_orientation = Quaternion(
                             x=0.0,    
                             y=1.0,   
                             z=0.0,    
                             w=0.0)  

    lid_pose = Pose(
        position=Point(x=0.84, y=0.06, z=0.016),
        orientation=vacuum_orientation)

    no_lid_pose = Pose(
        position=Point(x=0.67, y=0.50, z=0.014),
        orientation=vacuum_orientation)
 
    pnp.remove_lid(lid_pose)
    pnp.bye_lid(no_lid_pose)
    pnp.move_to_rest(resting_joint_angles)

    limb = 'right'  #######################################
    
    hover_distance = 0.075    

    starting_joint_angles = {'right_w0': -0.8429224429430349,
                             'right_w1': -0.8973787609129671,
                             'right_w2': 1.0684176187621908,
                             'right_e0': 1.2302525918841019,
                             'right_e1': 2.593194521920292,
                             'right_s0': -1.230636087081073,
                             'right_s1': -0.6768690226544388}
    pnp = PickAndPlace(limb, hover_distance)
    
    overhead_orientation = Quaternion(
                             x=0.5,    
                             y=0.5,   
                             z=0.5,    
                             w=0.52)   

    solution_pose = Pose(
        position=Point(x=.95, y=-0.27, z=0.38),
        orientation=overhead_orientation)

    block_poses = list()

    block_poses.append(Pose(
        position=Point(x=0.9, y=0.100, z=0.31),
        orientation=overhead_orientation))
 
    block_poses.append(Pose(
        position=Point(x=0.9, y=0.075, z=0.31),
        orientation=overhead_orientation))

    block_poses.append(Pose(
        position=Point(x=0.9, y=0.050, z=0.31),
        orientation=overhead_orientation))

    block_poses.append(Pose(
        position=Point(x=0.9, y=0.025, z=0.31),
        orientation=overhead_orientation))


    block_poses.append(Pose(
        position=Point(x=0.875, y=0.100, z=0.31),
        orientation=overhead_orientation))
 
    block_poses.append(Pose(
        position=Point(x=0.875, y=0.075, z=0.31),
        orientation=overhead_orientation))

    block_poses.append(Pose(
        position=Point(x=0.875, y=0.050, z=0.31),
        orientation=overhead_orientation))

    block_poses.append(Pose(
        position=Point(x=0.875, y=0.025, z=0.31),
        orientation=overhead_orientation))

    # right move to start position
    pnp.move_to_start(starting_joint_angles)
 


    idx = 0
    while not rospy.is_shutdown():
#       print("\nPicking...")
        pnp.get_solution(solution_pose)
#       pnp.pick(block_poses[idx])
#       print("\nPlacing...")
        pnp.leave_solution(block_poses[idx])
        idx = (idx+1) % len(block_poses)
#       pnp.place(block_poses[idx])
    return 0

if __name__ == '__main__':
    sys.exit(main())
