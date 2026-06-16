import time
import os
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import mujoco
import mujoco.viewer
from yolo_msgs.msg import DetectionArray
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo

# --- ROS 2 COMMUNICATION IMPORTS ---
from rclpy.action import ActionServer, ActionClient
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped

# --- MOVEIT 2 IMPORTS ---
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint, BoundingVolume
from shape_msgs.msg import SolidPrimitive


# Path to the XML file
xml_path = 'scene_cup_combined.xml'

with open(xml_path, 'r') as file:
    xml_template = file.read()

# Replace the placeholder with the actual home directory where the assets are located
home_dir = os.path.expanduser('~/code')

xml_string = xml_template.replace('{basefolder}', home_dir)

# MODEL_PATH = "scene_cup.xml"


class TiagoBrain(Node):
    def __init__(self):
        super().__init__('tiago_mujoco_brain')
        self.bridge = CvBridge()
        
        # Publisher for YOLO
        self.image_pub = self.create_publisher(Image, '/camera/rgb/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info', 10)

        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)

        # YOLO Subscriber
        self.yolo_sub = self.create_subscription(
            DetectionArray, 
            '/yolo/detections_3d',
            self.yolo_callback, 
            qos_profile_sensor_data
        )

# RECEIVING trajectories from MoveIt
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory', 
            self.execute_trajectory_callback
        )
        
# SENDING goals to MoveIt
        self.move_action_client = ActionClient(self, MoveGroup, '/move_action')
        
        # Variables
        self.active_trajectory = None
        self.trajectory_start_time = 0.0

        self.cup_move_threshold = 0.05  
        self.candidate_cup_pos = None   
        self.last_movement_time = 0.0   
        self.cup_above_height = 0.15    
        self.goal_sent = False
        self.ready_to_move = False 
        
        self.get_logger().info("TIAGo Brain started! Locked for 10s warmup...")

    def yolo_callback(self, msg):
        if not self.ready_to_move or self.active_trajectory is not None:
            return

        if not msg.detections:
            return

        found_cup = False
        for detection in msg.detections:
            if detection.class_id == 41 or detection.class_name == 'cup':
                found_cup = True

                raw_x = detection.bbox3d.center.position.x * 10.0
                raw_y = detection.bbox3d.center.position.y * 10.0
                raw_z = detection.bbox3d.center.position.z * 10.0

                robot_x = raw_z
                robot_y = -raw_x
                robot_z = 0.75
                
                current_cup_pos = np.array([robot_x, robot_y, robot_z])
                break 

        if not found_cup:
            return

        current_time = time.time()

        if self.candidate_cup_pos is None:
            self.get_logger().info("Cup detected. Starting stability countdown...")
            self.candidate_cup_pos = current_cup_pos
            self.last_movement_time = current_time
        else:
            dist = np.linalg.norm(current_cup_pos - self.candidate_cup_pos)

            if dist > self.cup_move_threshold:
                self.get_logger().info(f"Cup position changed by {dist:.3f}m (> 5cm). Resetting timer.")
                self.candidate_cup_pos = current_cup_pos
                self.last_movement_time = current_time
            else:
                elapsed_stable = current_time - self.last_movement_time
                if elapsed_stable >= 5.0: 
                    if not self.goal_sent:
                        self.get_logger().info(f"Cup stable! Composing MoveGroup Action request to MoveIt...")
                        self.send_goal_to_moveit()
                else:
                    self.get_logger().info(f"Waiting for stability... {elapsed_stable:.1f} / 5.0 s")

    def send_goal_to_moveit(self):
        # Wait for MoveIt to wake up
        if not self.move_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Error: MoveIt /move_action action server not responding!")
            return

        goal_msg = MoveGroup.Goal()
        req = MotionPlanRequest()
        req.workspace_parameters.header.frame_id = "base_link"
        req.group_name = 'arm'
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        
        constraint = Constraints()
        
        # 1. Define Goal (Increasing tolerance to 5 cm!)
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "arm_7_link"
        
        bv = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.05]  # <--- Changed from 0.01 to 0.05
        
        pose_goal = PoseStamped()
        pose_goal.header.frame_id = "base_link"
        pose_goal.pose.position.x = float(self.candidate_cup_pos[0])
        pose_goal.pose.position.y = float(self.candidate_cup_pos[1])
        pose_goal.pose.position.z = float(self.candidate_cup_pos[2] + self.cup_above_height)
        
        bv.primitives.append(sphere)
        bv.primitive_poses.append(pose_goal.pose)
        pos_constraint.constraint_region = bv
        pos_constraint.weight = 1.0
        
        constraint.position_constraints.append(pos_constraint)
        
        # NOTE: WE REMOVED orientation restrictions (OrientationConstraint).
        # From now on, MoveIt has free rein in choosing the gripper angle!
        
        req.goal_constraints.append(constraint)
        goal_msg.request = req
        goal_msg.planning_options.plan_only = False
        
        self.move_action_client.send_goal_async(goal_msg)
        self.goal_sent = True
        self.get_logger().info(f"Goal sent to MoveIt. Waiting for trajectory... {goal_msg.request.goal_constraints[0].position_constraints[0].constraint_region.primitives[0].dimensions[0]:.3f}m tolerance.")
        self.get_logger().info(f"Target cup position (with offset): x={pose_goal.pose.position.x:.3f}, y={pose_goal.pose.position.y:.3f}, z={pose_goal.pose.position.z:.3f}")

    def execute_trajectory_callback(self, goal_handle):
        self.get_logger().info("==========================================")
        self.get_logger().info("SUCCESS! MoveIt sent a trajectory to execute!")
        self.get_logger().info("==========================================")
        self.goal_sent = False

        self.active_trajectory = goal_handle.request.trajectory.points
        self.trajectory_start_time = time.time()
        
        goal_handle.succeed()
        result = FollowJointTrajectory.Result()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        return result

def apply_joint_pd_control(m, d, target_q, arm_qpos_ids, arm_dof_ids, arm_actuator_ids):
    Kp_arm = 60.0
    Kd_arm = 25.0

    for idx, actuator_id in enumerate(arm_actuator_ids):
        qpos_idx = arm_qpos_ids[idx]
        dof_idx = arm_dof_ids[idx]

        error_q = target_q[idx] - d.qpos[qpos_idx]
        current_vel = d.qvel[dof_idx]
        gravity_compensation = d.qfrc_bias[dof_idx]

        torque = (Kp_arm * error_q) - (Kd_arm * current_vel) + gravity_compensation

        lo, hi = m.actuator_forcerange[actuator_id]
        if lo == hi:                       # forcerange unset -> try ctrlrange
            lo, hi = m.actuator_ctrlrange[actuator_id]
        if lo == hi:                       # nothing defined -> don't artificially clip
            lo, hi = -1e6, 1e6
        d.ctrl[actuator_id] = np.clip(torque, lo, hi)

    Kd_base = 40.0
    for act_id in range(m.nu):
        if act_id not in arm_actuator_ids:
            dof_adr = m.jnt_dofadr[m.actuator_trnid[act_id, 0]]
            torque = d.qfrc_bias[dof_adr] - (Kd_base * d.qvel[dof_adr])

            lo, hi = m.actuator_forcerange[act_id]
            if lo == hi:
                lo, hi = m.actuator_ctrlrange[act_id]
            if lo == hi:
                lo, hi = -50.0, 50.0
            d.ctrl[act_id] = np.clip(0, lo, hi)

def get_target_positions(tiago_brain, home_angles):
    if tiago_brain.active_trajectory is None:
        return home_angles

    traj = tiago_brain.active_trajectory
    elapsed_time = time.time() - tiago_brain.trajectory_start_time

    if elapsed_time <= 0:
        return np.array(traj[0].positions)

    for i in range(len(traj) - 1):
        t1 = traj[i].time_from_start.sec + traj[i].time_from_start.nanosec / 1e9
        t2 = traj[i + 1].time_from_start.sec + traj[i + 1].time_from_start.nanosec / 1e9
        if t1 <= elapsed_time <= t2:
            progress = (elapsed_time - t1) / (t2 - t1) if (t2 - t1) > 0 else 0.0
            p1 = np.array(traj[i].positions)
            p2 = np.array(traj[i + 1].positions)
            return p1 + progress * (p2 - p1)

    # trajectory finished: hold final pose, free the arm up for a new goal
    tiago_brain.active_trajectory = None
    return np.array(traj[-1].positions)


def main():
    rclpy.init()
    tiago_brain = TiagoBrain()

    # m = mujoco.MjModel.from_xml_path(MODEL_PATH)
    m = mujoco.MjModel.from_xml_string(xml_string)
    d = mujoco.MjData(m)

    renderer = mujoco.Renderer(m, 480, 640)
    camera_id = 0 
    
    ee_id = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "arm_7_link")

    arm_joint_names = ["arm_1_joint", "arm_2_joint", "arm_3_joint", "arm_4_joint", "arm_5_joint", "arm_6_joint", "arm_7_joint"]
    arm_actuator_names = ["arm_1_joint_motor", "arm_2_joint_motor", "arm_3_joint_motor", "arm_4_joint_motor", "arm_5_joint_motor", "arm_6_joint_motor", "arm_7_joint_motor"]

    joint_qpos_indices = [m.jnt_qposadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)] for name in arm_joint_names]
    joint_dof_indices = [m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name)] for name in arm_joint_names]
    actuator_ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, name) for name in arm_actuator_names]

    home_angles = np.array([0.07, -0.98, -0.54, 2.29, 0.0, 0.0, 0.0])

    # Robot's initial pose BEFORE physics starts
    for i, idx in enumerate(joint_qpos_indices):
        d.qpos[idx] = home_angles[i]
    mujoco.mj_forward(m, d)
    for name, aid, dof in zip(arm_joint_names, actuator_ids, joint_dof_indices):
        print(f"{name}: qfrc_bias={d.qfrc_bias[dof]:7.2f}  "
          f"forcerange={m.actuator_forcerange[aid]}  ctrlrange={m.actuator_ctrlrange[aid]}")

    with mujoco.viewer.launch_passive(m, d) as viewer:
        last_render_time = time.time()
        start_real_time = time.time() 
        
        while viewer.is_running() and rclpy.ok():
            elapsed_real_time = time.time() - start_real_time
            while d.time < elapsed_real_time:
                target_positions = get_target_positions(tiago_brain, home_angles)
                apply_joint_pd_control(m, d, target_positions, joint_qpos_indices, joint_dof_indices, actuator_ids)
                mujoco.mj_step(m, d)
            
            viewer.sync()
            rclpy.spin_once(tiago_brain, timeout_sec=0.0)

            # 10-SECOND TIMER 
            if elapsed_real_time >= 10.0 and not tiago_brain.ready_to_move:
                print("\n========================================================")
                print("[SYSTEM] 10 seconds elapsed. Robot stabilized.")
                print("[SYSTEM] Unlocking YOLO. Starting cup stability analysis!")
                print("========================================================\n")
                tiago_brain.ready_to_move = True
            
            current_time = time.time()
            if current_time - last_render_time > 0.1:
                current_ros_time = tiago_brain.get_clock().now().to_msg()

                # RGB
                renderer.disable_depth_rendering()
                renderer.update_scene(d, camera_id)
                pixels = renderer.render()
                pixels_uint8 = (pixels * 255).astype(np.uint8)
                img_msg = tiago_brain.bridge.cv2_to_imgmsg(pixels_uint8, encoding="rgb8")
                img_msg.header.stamp = current_ros_time
                img_msg.header.frame_id = "head_front_camera_link"
                tiago_brain.image_pub.publish(img_msg)

                # CameraInfo
                cam_info = CameraInfo()
                cam_info.header = img_msg.header
                cam_info.height, cam_info.width = 480, 640
                cam_info.distortion_model = 'plumb_bob'
                fovy = m.cam_fovy[camera_id]
                f = 240.0 / np.tan(fovy * np.pi / 360.0)
                cam_info.k = [f, 0.0, 320.0, 0.0, f, 240.0, 0.0, 0.0, 1.0]
                cam_info.p = [f, 0.0, 320.0, 0.0, 0.0, f, 240.0, 0.0, 0.0, 0.0, 1.0, 0.0]
                tiago_brain.camera_info_pub.publish(cam_info)

                # DEPTH
                renderer.enable_depth_rendering()
                renderer.update_scene(d, camera_id)
                raw_depth = renderer.render()
                extent = m.stat.extent
                znear = m.vis.map.znear * extent
                zfar = m.vis.map.zfar * extent
                metric_depth = znear / (1.0 - raw_depth * (1.0 - znear / zfar))
                metric_depth_mm = (np.clip(metric_depth, 0.0, 3.0) * 1000.0).astype(np.uint16)
                
                depth_msg = tiago_brain.bridge.cv2_to_imgmsg(metric_depth_mm, encoding="16UC1")
                depth_msg.header.stamp = current_ros_time
                depth_msg.header.frame_id = "head_front_camera_link"
                tiago_brain.depth_pub.publish(depth_msg)
            
                last_render_time = current_time

            # JOINT STATE PUBLISHER FOR MOVEIT
            js_msg = JointState()
            js_msg.header.stamp = tiago_brain.get_clock().now().to_msg()
            js_msg.name = arm_joint_names
            js_msg.position = [d.qpos[idx] for idx in joint_qpos_indices]
            tiago_brain.joint_state_pub.publish(js_msg)
            
            # --- MOVEMENT ---
            # if tiago_brain.active_trajectory is None:
            #     apply_joint_pd_control(m, d, home_angles, joint_qpos_indices, joint_dof_indices, actuator_ids)
            # else:
            #     elapsed_time = time.time() - tiago_brain.trajectory_start_time
            #     traj = tiago_brain.active_trajectory
            #     target_positions = None
                
            #     if elapsed_time <= 0:
            #         target_positions = np.array(traj[0].positions)
            #     else:
            #         for i in range(len(traj) - 1):
            #             t1 = traj[i].time_from_start.sec + traj[i].time_from_start.nanosec / 1e9
            #             t2 = traj[i+1].time_from_start.sec + traj[i+1].time_from_start.nanosec / 1e9
                        
            #             if t1 <= elapsed_time <= t2:
            #                 progress = (elapsed_time - t1) / (t2 - t1) if (t2 - t1) > 0 else 0
            #                 p1 = np.array(traj[i].positions)
            #                 p2 = np.array(traj[i+1].positions)
            #                 target_positions = p1 + progress * (p2 - p1)
            #                 break
                    
            #         if target_positions is None:
            #             target_positions = np.array(traj[-1].positions)
                        
            #     apply_joint_pd_control(m, d, target_positions, joint_qpos_indices, joint_dof_indices, actuator_ids)

if __name__ == '__main__':
    main()