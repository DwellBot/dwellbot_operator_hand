from typing import List
import serial
import time
import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation as R
import math
import rclpy
import os
import time
import threading
#from scipy.spatial.transform import Rotation as R
from rclpy.node import Node
from sensor_msgs.msg import JointState
import sys
from geometry_msgs.msg import PoseArray, Point, Pose, Quaternion

regdict = {
    'ID' : 1000,
    'baudrate' : 1001,
    'clearErr' : 1004,
    'forceClb' : 1009,
    'angleSet' : 1486,
    'forceSet' : 1498,
    'speedSet' : 1522,
    'angleAct' : 1546,
    'forceAct' : 1582,
    'errCode' : 1606,
    'statusCode' : 1612,
    'temp' : 1618,
    'actionSeq' : 2320,
    'actionRun' : 2322
}


def openSerial(port, baudrate):
    ser = serial.Serial()
    ser.port = port
    ser.baudrate = baudrate
    ser.open()
    return ser


def writeRegister(ser, id, add, num, val):
    bytes = [0xEB, 0x90]
    bytes.append(id) # id
    bytes.append(num + 3) # len
    bytes.append(0x12) # cmd
    bytes.append(add & 0xFF)
    bytes.append((add >> 8) & 0xFF) # add
    for i in range(num):
        bytes.append(val[i])
    checksum = 0x00
    for i in range(2, len(bytes)):
        checksum += bytes[i]
    checksum &= 0xFF
    bytes.append(checksum)
    ser.write(bytes)
    time.sleep(0.01)
    ser.flush() 
    # ser.read_all() # 把返回帧读掉，不处理


def readRegister(ser, id, add, num, mute=False):
    bytes = [0xEB, 0x90]
    bytes.append(id) # id
    bytes.append(0x04) # len
    bytes.append(0x11) # cmd
    bytes.append(add & 0xFF)
    bytes.append((add >> 8) & 0xFF) # add
    bytes.append(num)
    checksum = 0x00
    for i in range(2, len(bytes)):
        checksum += bytes[i]
    checksum &= 0xFF
    bytes.append(checksum)
    ser.write(bytes)
    time.sleep(0.01)
    recv = ser.read_all()
    if len(recv) == 0:
        return []
    num = (recv[3] & 0xFF) - 3
    val = []
    for i in range(num):
        val.append(recv[7 + i])
    if not mute:
        print('读到的寄存器值依次为：', end='')
        for i in range(num):
            print(val[i], end=' ')
        print()
    return val


def write6(ser, id, str, val):
    if str == 'angleSet' or str == 'forceSet' or str == 'speedSet':
        val_reg = []
        for i in range(6):
            val_reg.append(val[i] & 0xFF)
            val_reg.append((val[i] >> 8) & 0xFF)
        writeRegister(ser, id, regdict[str], 12, val_reg)
    else:
        print('函数调用错误，正确方式：str的值为\'angleSet\'/\'forceSet\'/\'speedSet\'，val为长度为6的list，值为0~1000，允许使用-1作为占位符')


def read6(ser, id, str):
    if str == 'angleSet' or str == 'forceSet' or str == 'speedSet' or str == 'angleAct' or str == 'forceAct':
        val = readRegister(ser, id, regdict[str], 12, True)
        if len(val) < 12:
            print('没有读到数据')
            return
        val_act = []
        for i in range(6):
            val_act.append((val[2*i] & 0xFF) + (val[1 + 2*i] << 8))
        print('读到的值依次为：', end='')
        for i in range(6):
            print(val_act[i], end=' ')
        print()
    elif str == 'errCode' or str == 'statusCode' or str == 'temp':
        val_act = readRegister(ser, id, regdict[str], 6, True)
        if len(val_act) < 6:
            print('没有读到数据')
            return
        print('读到的值依次为：', end='')
        for i in range(6):
            print(val_act[i], end=' ')
        print()
    else:
        print('函数调用错误，正确方式：str的值为\'angleSet\'/\'forceSet\'/\'speedSet\'/\'angleAct\'/\'forceAct\'/\'errCode\'/\'statusCode\'/\'temp\'')

class InspireHand(Node):
    def __init__(self):
        super().__init__('leap_pyb_ik')

        # Start PyBullet
        p.connect(p.GUI)

        self.sub_skeleton = self.create_subscription(JointState, "/manus/right/joint_states", self.get_glove_joints, 10)
        self.data_lock_poses = threading.Lock()
        self.data_lock_angles = threading.Lock()
        self.latest_hand_pos = None
        self.latest_hand_angles : List[float] = []
        self.data_updated = False  # Flag to indicate new data availability
        self.glove_to_leap_mapping_scale = 1.0

        path_src = os.path.dirname(os.path.abspath(__file__))
        self.initial_offset = np.array([0.0, 0.0, 0.0])  # initial offset
        self.initial_orientation = p.getQuaternionFromEuler([0, -math.pi / 2, math.pi / 2])
        path_src = os.path.join(path_src, "../../inspire_hand/inspire_hand_right.urdf")
        self.urdf_id = p.loadURDF(path_src, 
                                    self.initial_offset.tolist(), 
                                    self.initial_orientation, 
                                    useFixedBase=True,
        )
        self.sub_skeleton = self.create_subscription(PoseArray, "/manus/right/poses", self.get_glove_poses, 1)

        # Initialize other PyBullet settings
        self.numJoints = p.getNumJoints(self.urdf_id)
        #for i in range(self.numJoints):
        #    print(p.getJointInfo(self.urdf_id, i))
        for link1 in range(self.numJoints):
            for link2 in range(link1 + 1, self.numJoints):
                p.setCollisionFilterPair(self.urdf_id, self.urdf_id, link1, link2, enableCollision=False)

        # Change the color of all links
        for i in range(-1, self.numJoints):  # -1 is for the base link
            p.changeVisualShape(
                self.urdf_id,
                linkIndex=i,
                rgbaColor=[1, 0.5, 0.5, 1]  # Light pink color
            )

        p.setGravity(0, 0, 0)
        p.setRealTimeSimulation(0)
        self.create_target_vis()
        self.update_camera_view()

        
        print('open port')
        self.inspire_serial_right = openSerial('/dev/ttyUSB0', 115200)
        write6(self.inspire_serial_right, 1, 'speedSet', [1000, 1000, 1000, 1000, 1000, 1000])
        write6(self.inspire_serial_right, 1, 'forceSet', [500, 500, 500, 500, 500, 500])
        write6(self.inspire_serial_right, 1, 'angleSet', [1000, 1000, 1000, 1000, 1000, 1000])
        


    def update_camera_view(self):
        # Adjust these values based on how you want the view to look
        camera_distance = 1.4  # Distance from the object
        camera_yaw = 15  # Horizontal rotation angle
        camera_pitch = 60  # Vertical rotation angle
        camera_target_position = [0, 0, 0]  # Target position in the scene
        
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
        for i in range(0,25):
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
        p.changeVisualShape(self.ballMbt[21], -1, rgbaColor=[1, 1, 0.2, 1])
        p.changeVisualShape(self.ballMbt[22], -1, rgbaColor=[1, 1, 0.4, 1])
        p.changeVisualShape(self.ballMbt[23], -1, rgbaColor=[1, 1, 0.6, 1])
        p.changeVisualShape(self.ballMbt[24], -1, rgbaColor=[1, 1, 0.8, 1])

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

    def get_glove_poses(self, pose):
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
            
            with self.data_lock_poses:
                #print(str(hand_pos))
                self.latest_hand_pos = hand_pos
                self.data_updated = True  # Mark data as updated
            #self.get_logger().info("New glove data received and processed.")


    def get_glove_joints(self, joint_state):

        #print(str(joint_state))
        #return

        with self.data_lock_angles:
            #print(str(hand_pos))
            self.latest_hand_angles = joint_state.position[:]
            self.data_updated = True  # Mark data as updated


    def compute_joints(self, hand_pos : List[float]) -> np.ndarray:

        #percent_thumb1 = (hand_pos[0] + hand_pos[1] ) / 200.0
        #percent_thumb2 = (hand_pos[1] + hand_pos[2] + hand_pos[3]) / 200.0
        percent_index = (hand_pos[5] + hand_pos[6] + hand_pos[7]) / 250.0
        percent_middle = (hand_pos[9] + hand_pos[10] + hand_pos[11]) / 250.0
        percent_ring = (hand_pos[13] + hand_pos[14] + hand_pos[15]) / 250.0
        percent_pinky = (hand_pos[17] + hand_pos[18] + hand_pos[19]) / 250.0
        
        #angles = [percent_index, percent_middle, percent_ring, percent_pinky, percent_thumb1, percent_thumb2]
        angles = [percent_pinky, percent_ring, percent_middle, percent_index, 0, 0]


        angles = np.array(angles)
        angles = np.clip(np.array(angles), 0.0, 1.0).astype(float) #clip to 0 - 1
        inverted_angles = 1 - angles

        return inverted_angles

    def compute_IK(self, hand_pos, hand_angles):
        p.stepSimulation()     

        rightHandThumb_pos = hand_pos[4]
        endEffectorIndex = 4

        jointPoses = p.calculateInverseKinematics(
            bodyUniqueId=self.urdf_id,
            endEffectorLinkIndex=endEffectorIndex,
            targetPosition=rightHandThumb_pos,
        )

       
        #print(str(jointPoses))
        hand_angles[5] = 1.0 - (jointPoses[0] / 1.3)
        hand_angles[4] = 1.0 - ((jointPoses[1] + jointPoses[2] + jointPoses[3])  / 1.5)
        hand_angles[4:6] = np.clip(np.array(hand_angles[4:6]), 0.0, 1.0).astype(float) #clip to 0 - 1

        # update the hand joints
        for i in range(5):
            p.setJointMotorControl2(
                bodyIndex=self.urdf_id,
                jointIndex=i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=jointPoses[i],
                targetVelocity=0,
                force=500,
                positionGain=0.3,
                velocityGain=1,
            )

    def update_urdf_fingers(self, hand_angles):
        if hand_angles is None or len(hand_angles) < 6:
            return
        
        # update the index finger
        joint_offset = 5
        index_finger = [0, 0, 0]
        index_finger[0] = (1.0 - hand_angles[0]) * 1.7
        index_finger[1] = (1.0 - hand_angles[0]) * 1.7
        index_finger[2] = (1.0 - hand_angles[0]) * 1.7
        for i in range(3):
            p.setJointMotorControl2(
                bodyIndex=self.urdf_id,
                jointIndex=i + joint_offset,
                controlMode=p.POSITION_CONTROL,
                targetPosition=index_finger[i],
                targetVelocity=0,
                force=500,
                positionGain=0.3,
                velocityGain=1,
            )

        # update the middle finger
        joint_offset = 8
        index_finger = [0, 0, 0]
        index_finger[0] = (1.0 - hand_angles[1]) * 1.7
        index_finger[1] = (1.0 - hand_angles[1]) * 1.7
        index_finger[2] = (1.0 - hand_angles[1]) * 1.7
        for i in range(3):
            p.setJointMotorControl2(
                bodyIndex=self.urdf_id,
                jointIndex=i + joint_offset,
                controlMode=p.POSITION_CONTROL,
                targetPosition=index_finger[i],
                targetVelocity=0,
                force=500,
                positionGain=0.3,
                velocityGain=1,
            )

        # update the ring finger
        joint_offset = 11
        index_finger = [0, 0, 0]
        index_finger[0] = (1.0 - hand_angles[2]) * 1.7
        index_finger[1] = (1.0 - hand_angles[2]) * 1.7
        index_finger[2] = (1.0 - hand_angles[2]) * 1.7
        for i in range(3):
            p.setJointMotorControl2(
                bodyIndex=self.urdf_id,
                jointIndex=i + joint_offset,
                controlMode=p.POSITION_CONTROL,
                targetPosition=index_finger[i],
                targetVelocity=0,
                force=500,
                positionGain=0.3,
                velocityGain=1,
            )

        # update the pinky finger
        joint_offset = 14
        index_finger = [0, 0, 0]
        index_finger[0] = (1.0 - hand_angles[3]) * 1.7
        index_finger[1] = (1.0 - hand_angles[3]) * 1.7
        index_finger[2] = (1.0 - hand_angles[3]) * 1.7
        for i in range(3):
            p.setJointMotorControl2(
                bodyIndex=self.urdf_id,
                jointIndex=i + joint_offset,
                controlMode=p.POSITION_CONTROL,
                targetPosition=index_finger[i],
                targetVelocity=0,
                force=500,
                positionGain=0.3,
                velocityGain=1,
            )


    def set_hand(self, finger_angles : np.ndarray):
        start_time = time.time()
        finger_cmd_r = np.array([-1]*6)

        print(f"finger_angles: {finger_angles}")
        np_angles = np.array(finger_angles)
        print(f"finger_angles shape: {np_angles.shape}")

        finger_cmd_r[:6] = np.clip(np.array(finger_angles[:6]) * 1000, 0, 1000).astype(int)

        #finger_cmd_r[:4] = (np.clip((finger_angles[:4])*1000,0,1000)).astype(int)  # range 30-160 30-close-cmd:0
        #finger_cmd_r[4] = (np.clip((finger_angles[4])*1000,0,1000)).astype(int)  # thumb close: 90 open: 140
        #finger_cmd_r[5] = (np.clip((finger_angles[5])*1000,0,1000)).astype(int)  # thumb lateral close:60 open:80 range 10-30 30-close-cmd:0 
        
        write6(self.inspire_serial_right, 1, 'angleSet', finger_cmd_r)
        print('cmd: ' + str(finger_cmd_r))
        #print('loop time:', time.time() - start_time)  #loop time: 0.0315 if read6 loop time: 0.0156 if write only

    def update_hand(self):
        while rclpy.ok():
            hand_pos = None
            hand_angles = None

            with self.data_lock_poses:
                if self.data_updated and self.latest_hand_pos is not None:
                    hand_pos = self.latest_hand_pos
                    hand_angles = self.latest_hand_angles
                    #print(str(hand_pos))
                    self.data_updated = False  # Reset the update flag

            with self.data_lock_angles:
                if self.data_updated and self.latest_hand_angles is not None:
                    hand_angles = self.latest_hand_angles
                    hand_pos = self.latest_hand_pos
                    #print(str(hand_pos))
                    self.data_updated = False  # Reset the update flag


            if hand_pos is not None and hand_angles is not None:
                self.update_hand_orientation(self.wrist_quat)
                self.update_target_vis(hand_pos)
                inspire_joints = self.compute_joints(hand_angles)
                self.update_urdf_fingers(inspire_joints)
                self.compute_IK(hand_pos, inspire_joints)
                print('percent angles: ' + str(inspire_joints))

                self.set_hand(inspire_joints)
                #self.get_logger().info("Hand model updated with new data.")
            
            p.stepSimulation()
            time.sleep(1 / 60)  # Run the simulation at approximately 30 FPS

def ros2_thread(node):
    """ROS2 processing thread."""
    rclpy.spin(node)


def main(args=None):
    rclpy.init(args=args)
    inspire = InspireHand()

    # Start the ROS2 thread
    ros_thread = threading.Thread(target=ros2_thread, args=(inspire,))
    ros_thread.start()

    # Start the PyBullet simulation loop
    inspire.update_hand()

    # Shutdown ROS and clean up
    inspire.destroy_node()
    rclpy.shutdown()
    ros_thread.join()

if __name__ == "__main__":
    main()