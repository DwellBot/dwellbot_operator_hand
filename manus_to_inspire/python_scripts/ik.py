#!/usr/bin/env python3
import pybullet as p
import math
import numpy as np
import rclpy
import os
import time
import threading
from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Point, Pose, Quaternion
from sensor_msgs.msg import JointState
import sys
from ament_index_python.packages import get_package_share_directory
'''
This takes the glove data, and runs inverse kinematics and then publishes onto LEAP Hand.

Note how the fingertip positions are matching, but the joint angles between the two hands are not.  :) 

Inspired by Dexcap https://dex-cap.github.io/ by Wang et. al. and Robotic Telekinesis by Shaw et. al.
'''
class InspirePybulletIK(Node):
    def __init__(self):
        super().__init__('leap_pyb_ik')
        # Start PyBullet
        p.connect(p.GUI)
        
        self.is_left = self.declare_parameter('isLeft', False).get_parameter_value().bool_value
        self.glove_to_leap_mapping_scale = 0.8
        self.leapEndEffectorIndex = [2, 3, 4, 7, 8, 9, 13, 14, 18, 19]
        
        # Load the URDF for the hand model
        path_src = os.path.dirname(os.path.abspath(__file__))
        if self.is_left:
            path_src = os.path.join(path_src, "../../inspire_hand/robot_pybullet.urdf")
            self.urdf_id = p.loadURDF(path_src, [0, 0, 0], p.getQuaternionFromEuler([0, 0, 0]), useFixedBase=True)
            self.pub_hand = self.create_publisher(JointState, '/leaphand_node/cmd_allegro_left', 10)
            self.sub_skeleton = self.create_subscription(PoseArray, "/glove/l_short", self.get_glove_data, 10)
        else:
            self.initial_offset = np.array([0.0, 0.0, 0.0])  # initial offset
            self.initial_orientation = p.getQuaternionFromEuler([math.pi, -math.pi / 2, -math.pi / 2])
            path_src = os.path.join(path_src, "../../inspire_hand/inspire_hand_right.urdf")
            self.urdf_id = p.loadURDF(path_src, 
                                     self.initial_offset.tolist(), 
                                     self.initial_orientation, 
                                     useFixedBase=True,
            )
            self.pub_hand = self.create_publisher(JointState, '/inspire_hand_right/joint_positions', 10)
            self.sub_skeleton = self.create_subscription(PoseArray, "/manus_right", self.get_glove_data, 1)

        # Initialize other PyBullet settings
        self.numJoints = p.getNumJoints(self.urdf_id)
        for i in range(self.numJoints):
            print(p.getJointInfo(self.urdf_id, i))
        for link1 in range(self.numJoints):
            for link2 in range(link1 + 1, self.numJoints):
                p.setCollisionFilterPair(self.urdf_id, self.urdf_id, link1, link2, enableCollision=False)

        p.setGravity(0, 0, 0)
        p.setRealTimeSimulation(0)
        self.create_target_vis()

        # Thread-safe shared variable for glove data
        self.data_lock = threading.Lock()
        self.latest_hand_pos = None
        self.latest_hand_angles = None
        self.data_updated = False  # Flag to indicate new data availability


    def update_camera_view(self):
        # Adjust these values based on how you want the view to look
        camera_distance = 1.4  # Distance from the object
        camera_yaw = 0  # Horizontal rotation angle
        camera_pitch = 60  # Vertical rotation angle
        camera_target_position = [0, -2, -1]  # Target position in the scene
        
        # Apply the new camera settings
        p.resetDebugVisualizerCamera(
            cameraDistance=camera_distance,
            cameraYaw=camera_yaw,
            cameraPitch=camera_pitch,
            cameraTargetPosition=camera_target_position
        )


    def create_target_vis(self):
        # load balls
        small_ball_radius = 0.01
        small_ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=small_ball_radius)
        ball_radius = 0.01
        ball_shape = p.createCollisionShape(p.GEOM_SPHERE, radius=ball_radius)
        baseMass = 0.001
        basePosition = [-0.0, -0.0, 0]
        
        self.ballMbt = []
        for i in range(0,21):
            self.ballMbt.append(p.createMultiBody(baseMass=baseMass, baseCollisionShapeIndex=ball_shape, basePosition=basePosition)) # for base and finger tip joints    
            no_collision_group = 0
            no_collision_mask = 0
            p.setCollisionFilterGroupMask(self.ballMbt[i], -1, no_collision_group, no_collision_mask)
        
        #wrist
        p.changeVisualShape(self.ballMbt[0], -1, rgbaColor=[1, 1, 1, 1]) 

        #thumb
        p.changeVisualShape(self.ballMbt[1], -1, rgbaColor=[0, 0.2, 0, 1]) 
        p.changeVisualShape(self.ballMbt[2], -1, rgbaColor=[0, 0.4, 0, 1]) 
        p.changeVisualShape(self.ballMbt[3], -1, rgbaColor=[0, 0.6, 0, 1]) 
        p.changeVisualShape(self.ballMbt[4], -1, rgbaColor=[0, 0.8, 0, 1]) 
        p.changeVisualShape(self.ballMbt[5], -1, rgbaColor=[0, 1.0, 0, 1])  
 
        #index
        
        p.changeVisualShape(self.ballMbt[6], -1, rgbaColor=[0, 0, 0.2, 1])  
        p.changeVisualShape(self.ballMbt[7], -1, rgbaColor=[0, 0, 0.4, 1])  
        p.changeVisualShape(self.ballMbt[8], -1, rgbaColor=[0, 0, 0.6, 1])  
        p.changeVisualShape(self.ballMbt[9], -1, rgbaColor=[0, 0, 0.8, 1])
        p.changeVisualShape(self.ballMbt[10], -1, rgbaColor=[0, 0, 1, 1])

        #middle
        p.changeVisualShape(self.ballMbt[11], -1, rgbaColor=[0.2, 0, 0, 1])
        p.changeVisualShape(self.ballMbt[12], -1, rgbaColor=[0.4, 0, 0, 1])
        p.changeVisualShape(self.ballMbt[13], -1, rgbaColor=[0.6, 0, 0.4, 1])
        p.changeVisualShape(self.ballMbt[14], -1, rgbaColor=[0.8, 0, 0.6, 1])
        p.changeVisualShape(self.ballMbt[15], -1, rgbaColor=[1, 0, 0.8, 1])

        #ring

        p.changeVisualShape(self.ballMbt[16], -1, rgbaColor=[1, 0.5, 0.2, 1])
        p.changeVisualShape(self.ballMbt[17], -1, rgbaColor=[1, 0.5, 0.4, 1])
        p.changeVisualShape(self.ballMbt[18], -1, rgbaColor=[1, 0.5, 0.6, 1])
        p.changeVisualShape(self.ballMbt[19], -1, rgbaColor=[1, 0.5, 0.8, 1])
        p.changeVisualShape(self.ballMbt[20], -1, rgbaColor=[1, 0.5, 1, 1])

        #pinky
        #p.changeVisualShape(self.ballMbt[21], -1, rgbaColor=[1, 1, 0.2, 1])
        #p.changeVisualShape(self.ballMbt[22], -1, rgbaColor=[1, 1, 0.4, 1])
        #p.changeVisualShape(self.ballMbt[23], -1, rgbaColor=[1, 1, 0.6, 1])
        #p.changeVisualShape(self.ballMbt[24], -1, rgbaColor=[1, 1, 0.8, 1])

    # Function to update orientation dynamically
    def update_hand_orientation(self, glove_quaternion):
        # Get the current position (to keep it unchanged)
        current_position, _ = p.getBasePositionAndOrientation(self.urdf_id)
        
        # Convert the glove's quaternion to a rotation matrix
        glove_rotation = R.from_quat(glove_quaternion)

        # Apply the initial offset in the glove's rotated frame
        adjusted_position = glove_rotation.apply(self.initial_offset)

        intial_rotation = R.from_quat(self.initial_orientation)
        adjusted_orientation = glove_rotation * intial_rotation
        new_orientation_quat = adjusted_orientation.as_quat()  # Returns [x, y, z, w]

        # Apply the updated quaternion to the URDF model
        p.resetBasePositionAndOrientation(self.urdf_id, adjusted_position, new_orientation_quat)


    def update_target_vis(self, hand_pos):
        
        for i in range(21):
            if i == 44:
                self.get_logger().info("ball #4" + str(i) + ": " + str(hand_pos[i]))
            _, current_orientation = p.getBasePositionAndOrientation( self.ballMbt[i])
            p.resetBasePositionAndOrientation(self.ballMbt[i], hand_pos[i], current_orientation)

    def get_glove_data(self, pose):
            """Callback to receive glove data and update hand positions."""
            hand_pos = [
                [1.0 * pose.poses[i].position.x * self.glove_to_leap_mapping_scale,
                 1.0 * pose.poses[i].position.y * self.glove_to_leap_mapping_scale,
                 1.0 * pose.poses[i].position.z * self.glove_to_leap_mapping_scale]
                for i in range(25)
            ]


            self.wrist_quat = [pose.poses[0].orientation.x,
                               pose.poses[0].orientation.y,
                               pose.poses[0].orientation.z,
                               pose.poses[0].orientation.w]
            
            with self.data_lock:
                #print(str(hand_pos))
                self.latest_hand_pos = hand_pos
                self.data_updated = True  # Mark data as updated
            #self.get_logger().info("New glove data received and processed.")



    def normalize_to_local(self, world_positions, base_quaternion, base_position=None):
        """
        Normalize positions from world coordinates to a local coordinate system.

        Args:
        - world_positions: np.array of shape (N, 3), list of XYZ positions in world coordinates.
        - base_quaternion: list or np.array of shape (4), quaternion (w, x, y, z) specifying the base rotation.
        - base_position: list or np.array of shape (3), optional, specifies the base position for translation.

        Returns:
        - local_positions: np.array of shape (N, 3), positions in the local coordinate system.
        """
        # Convert the quaternion to a rotation matrix
        base_rotation = R.from_quat(base_quaternion)  # Base rotation as a scipy Rotation object

        # Calculate the inverse rotation (to move into the local frame)
        inverse_rotation = base_rotation.inv()

        # Initialize local positions array
        local_positions = np.zeros_like(world_positions)

        # Translate positions to be relative to the base position
        if base_position is not None:
            translated_positions = world_positions - np.array(base_position)
        else:
            translated_positions = world_positions

        # Apply the inverse rotation to each position
        local_positions = inverse_rotation.apply(translated_positions)

        return local_positions

    # Convert absolute points to relative
    def convert_to_relative(self, pose_array):
        # Extract the points into a numpy array for easy manipulation
        points = np.array([[pose.position.x, pose.position.y, pose.position.z] for pose in pose_array.poses])

        # Define the parent-child relationships
        parent_indices = [1, 5, 9, 13]  # Parent points
        child_groups = [[2, 3, 4], [6, 7, 8], [10, 11, 12], [14, 15, 16]]  # Child points for each parent

        for parent_index, child_indices in zip(parent_indices, child_groups):
            # Get the parent point
            parent_point = points[parent_index]
            
            # Convert child points to be relative to the parent
            for child_index in child_indices:
                points[child_index] -= parent_point

        # Update the pose array with the new relative points
        for i, point in enumerate(points):
            pose_array.poses[i].position.x = point[0]
            pose_array.poses[i].position.y = point[1]
            pose_array.poses[i].position.z = point[2]

        return pose_array

    def update_hand(self):
        """Main PyBullet simulation loop to update the hand model."""
        while rclpy.ok():
            with self.data_lock:
                if self.data_updated and self.latest_hand_angles is not None and self.latest_hand_pos is not None:
                    hand_angles = self.latest_hand_angles
                    hand_pos = self.latest_hand_pos
                    #print(str(hand_pos))
                    self.data_updated = False  # Reset the update flag
                else:
                    hand_pos = None
                    hand_angles = None

            if hand_pos is not None and hand_angles is not None :
                self.update_hand_orientation(self.wrist_quat)
                #self.compute_IK2(hand_pos)
                self.update_target_vis(hand_pos)
                #self.get_logger().info("Hand model updated with new data.")

            #p.stepSimulation()
            time.sleep(1 / 30)  # Run the simulation at approximately 30 FPS


    def rotate_position_with_quaternion(self, position, quaternion):
        """
        Rotate a 3D position using a quaternion.

        Args:
        - position: List or np.array of shape (3,), the 3D position to rotate [x, y, z].
        - quaternion: List or np.array of shape (4,), the quaternion [x, y, z, w].

        Returns:
        - rotated_position: np.array of shape (3,), the rotated position.
        """
        # Create a Rotation object from the quaternion
        rotation = R.from_quat(quaternion)

        # Apply the rotation to the position
        rotated_position = rotation.apply(position)

        return rotated_position


    def compute_IK2(self, hand_pos):
        p.stepSimulation()     

        rightHandIndex_low_pos = hand_pos[7]
        rightHandIndex_middle_pos = hand_pos[8]
        rightHandIndex_pos = hand_pos[9]
        
        rightHandMiddle_low_pos = hand_pos[12]
        rightHandMiddle_middle_pos = hand_pos[13]
        rightHandMiddle_pos = hand_pos[14]
        
        rightHandRing_middle_pos = hand_pos[18]
        rightHandRing_pos = hand_pos[19]
        
        rightHandThumb_middle_pos = hand_pos[3]
        rightHandThumb_pos = hand_pos[4]
        
        leapEndEffectorPos = [
            rightHandIndex_low_pos,
            rightHandIndex_middle_pos,
            rightHandIndex_pos,
            rightHandMiddle_low_pos,
            rightHandMiddle_middle_pos,
            rightHandMiddle_pos,
            rightHandRing_middle_pos,
            rightHandRing_pos,
            rightHandThumb_middle_pos,
            rightHandThumb_pos
        ]

        targetOrns = [p.getQuaternionFromEuler([0, 0, 0]) for _ in leapEndEffectorPos]

        jointPoses = p.calculateInverseKinematics2(
            self.LeapId,
            self.leapEndEffectorIndex,
            leapEndEffectorPos,
            #targetOrientations=targetOrns,  # Specify orientations for end effectors
            solver=p.IK_DLS,
            maxNumIterations=200,
            residualThreshold=01e-5,
        )
        
        combined_jointPoses = (jointPoses[0:4] + (0.0,) + jointPoses[4:8] + (0.0,) + jointPoses[8:12] + (0.0,) + jointPoses[12:16] + (0.0,))
        combined_jointPoses = list(combined_jointPoses)

        # update the hand joints
        for i in range(20):
            p.setJointMotorControl2(
                bodyIndex=self.LeapId,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=combined_jointPoses[i],
                targetVelocity=0,
                force=500,
                positionGain=0.3,
                velocityGain=1,
            )

        # map results to real robot
        real_robot_hand_q = np.array([float(0.0) for _ in range(16)])
        #real_left_robot_hand_q = np.array([0.0 for _ in range(16)])

        real_robot_hand_q[0:4] = jointPoses[0:4]
        real_robot_hand_q[4:8] = jointPoses[4:8]
        real_robot_hand_q[8:12] = jointPoses[8:12]
        real_robot_hand_q[12:16] = jointPoses[12:16]
        real_robot_hand_q[13] = jointPoses[13] - math.pi/2
        #real_robot_hand_q[0:2] = real_robot_hand_q[0:2][::-1]
        #real_robot_hand_q[4:6] = real_robot_hand_q[4:6][::-1]
        #real_robot_hand_q[8:10] = real_robot_hand_q[8:10][::-1]
        stater = JointState()
        stater.position = [float(i) for i in real_robot_hand_q]
        self.pub_hand.publish(stater)

def ros2_thread(node):
    """ROS2 processing thread."""
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    inspire_pybullet_ik = InspirePybulletIK()

    # Start the ROS2 thread
    ros_thread = threading.Thread(target=ros2_thread, args=(inspire_pybullet_ik,))
    ros_thread.start()

    # Start the PyBullet simulation loop
    inspire_pybullet_ik.update_hand()

    # Shutdown ROS and clean up
    inspire_pybullet_ik.destroy_node()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == "__main__":
    main()