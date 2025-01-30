import rospy
import PyKDL


import random
import math
import time
import sys
from enum import Enum, auto


from std_msgs.msg import Bool
from gazebo_msgs.srv import SpawnModel, GetLinkState, GetModelState
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped, Twist
from nav_msgs.msg import Odometry

import moveit_commander
import moveit_msgs.msg
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive



FRAME_ID = 'map'
SCENE = moveit_commander.PlanningSceneInterface()

table_size = [PyKDL.Vector(1.0, 0.6, 0.5),
              PyKDL.Vector(1.4, 0.4, 0.8),
              PyKDL.Vector(1.0, 1.0, 0.75)]

POSITION_HISTORY = [None] * 12



"""
                            DODATKOWA FUNKCJONALNOŚĆ
"""


class RobotState:
    def __init__(self) -> None:
        self.is_move_done = False

    def move_done_callback(self, _):
        self.is_move_done = True


def clear_history():
    global POSITION_HISTORY
    POSITION_HISTORY = [None] * 12


def create_quaternion(roll, pitch, yaw):
    rot = PyKDL.Rotation.RPY(roll, pitch, yaw)
    return rot.GetQuaternion()


def get_link_pose(link_full_name):
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        resp = get_link_state(link_full_name, '')
        if resp.success:
            pos = resp.link_state.pose.position
            quat = resp.link_state.pose.orientation

            rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
            pos = PyKDL.Vector(pos.x, pos.y, pos.z)
            return PyKDL.Frame(rot, pos)
        else:
            print(f"Failed to get link state for {link_full_name}: {resp.status_message}")
            return None
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return None


def get_model_pose(model_full_name):
    rospy.wait_for_service('/gazebo/get_model_state')

    try:
        get_model_state = rospy.ServiceProxy('/gazebo/get_model_state', GetModelState)
        resp = get_model_state(model_full_name, '')

        if resp.success:
            pos = resp.pose.position
            quat = resp.pose.orientation

            rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
            pos = PyKDL.Vector(pos.x, pos.y, pos.z)
            return PyKDL.Frame(rot, pos)
        else:
            print(f"Failed to get model state for {model_full_name}: {resp.status_message}")
            return None
    except rospy.ServiceException as e:
        print(f"Service call failed: {e}")
        return None


def get_positions():

    table_pose = []
    table_position = []
    for table_index in range(3):
        cur_pose = get_link_pose(f'table_{table_index}::link')
        table_pose.append(cur_pose)
        table_position.append(cur_pose.p)

    object_pose = []
    object_position = []
    for object_name in ['bottom', 'middle','top']:
        cur_pose = get_link_pose(f'{object_name}::link')
        object_pose.append(cur_pose)
        object_position.append(cur_pose.p)

    poses = [
        table_position, object_position, object_pose
    ]

    return poses


def identify_location(table_position, object_position):

    allign = {}

    for i in range(3):
        
        vect1 = table_position[i] - object_position[0]
        dif1 = vect1.Norm()

        vect2 = table_position[i] - object_position[1]
        dif2 = vect2.Norm()

        vect3 = table_position[i] - object_position[2]
        dif3 = vect3.Norm()


        if dif1 == min(dif1, dif2, dif3):
            allign['bottom'] = f'table_{i}'

        if dif2 == min(dif1, dif2, dif3):
            allign['middle'] = f'table_{i}' 

        if dif3 == min(dif1, dif2, dif3):
            allign['top'] = f'table_{i}'
        
    print(f'\n {allign} \n')
    return allign



"""
                            CZĘŚĆ MOBILNA
"""


def get_destination_xy(obj_name, objects_data):
    
    destination_table = objects_data[obj_name]['table']
    object_pos = objects_data[obj_name]['position']

    if destination_table == 'table_0':
        yaw = math.pi/2
        rot = create_quaternion(0, 0, yaw)
        dest_x = object_pos.x()
        dest_y = object_pos.y() - 0.8

    elif destination_table == 'table_1':
        yaw = -math.pi/2
        rot = create_quaternion(0, 0, yaw)
        dest_x = object_pos.x()
        dest_y = object_pos.y() + 0.8

    elif destination_table == 'table_2':
        
        if object_pos.y() > -1.1:
            yaw = -math.pi/2
            rot = create_quaternion(0, 0, yaw)
            dest_y = object_pos.y() + 0.8
            dest_x = object_pos.x()

        else:
            dest_y = object_pos.y()
            if object_pos.y() < -1.1:
                dest_y = -1.1
            
            if object_pos.x() < -1.0:
                yaw = 0
                rot = create_quaternion(0, 0, yaw)
                dest_x = -1.9
            else:
                yaw = math.pi
                rot = create_quaternion(0, 0, yaw)
                dest_x = -0.1

    print(f'\n ojbect at {destination_table}')
    print(f' object      (x, y): ({object_pos.x()}, {object_pos.y()})')
    print(f' destination (x, y): ({dest_x}, {dest_y})\n')

    return dest_x, dest_y, rot, yaw


def destination_msg_create(x, y, rot):

    msg = PoseStamped()
    msg.pose.orientation.x = rot[0]
    msg.pose.orientation.y = rot[1]
    msg.pose.orientation.z = rot[2]
    msg.pose.orientation.w = rot[3]
    msg.pose.position.x, msg.pose.position.y = x, y

    return msg


def is_at_point(x, y, robot_pose, yaw_obj=None):
    eps_xy = 0.15
    eps_orient = 0.15

    x_dif   = abs(robot_pose.p.x() - x)
    y_dif   = abs(robot_pose.p.y() - y)
    
    if  x_dif < eps_xy and y_dif < eps_xy:
        if yaw_obj is None:
            return True
        else:
            roll, pitch, yaw = robot_pose.M.GetRPY()
            zR_dif  = abs(yaw - yaw_obj)
            if zR_dif < eps_orient:
                return True
    return False


def check_is_stuck():
    delta_x = abs(POSITION_HISTORY[0][0] - POSITION_HISTORY[-1][0])
    delta_y = abs(POSITION_HISTORY[0][1] - POSITION_HISTORY[-1][1])

    eps = 0.005

    if delta_x < eps and delta_y < eps:
        return True
    return False


def route(name, objects_data, pub):
    clear_history()
    is_stuck = False
    destination_accomplished = False

    x, y, rot_quat, obj_yaw = get_destination_xy(name, objects_data)
    msg = destination_msg_create(x, y, rot_quat)
    his_pub = rospy.Publisher("/key_vel", Twist, queue_size=10)

    while not destination_accomplished:
        robot_pose = get_link_pose('tiago::base_footprint')

        POSITION_HISTORY.pop(0)
        POSITION_HISTORY.append([robot_pose.p.x(), robot_pose.p.y()])

        if not is_stuck and POSITION_HISTORY[0] != None:
            is_stuck = check_is_stuck()

        if is_stuck:
            print("\nSTUCK\n")
            if round(POSITION_HISTORY[6][0], 3) != round(POSITION_HISTORY[5][0], 3) and round(POSITION_HISTORY[5][1], 3) != round(POSITION_HISTORY[5][1], 3):
                is_stuck = False
            goal = Twist()
            goal.linear.x = -0.1
            his_pub.publish(goal)
            msg = destination_msg_create(x, y-0.02, rot_quat)

        else:
            pub.publish(msg)

        destination_accomplished = is_at_point(x, y, robot_pose, obj_yaw)

        rospy.sleep(1)
        print(f'\n state: goint to {name}\n')
        print(f' history:\n{POSITION_HISTORY}\n')
        print(f' destination position (x, y): ({x}, {y})')
        print(f' destination rotation (Yaw): {obj_yaw}\n')
        print(f' current robot position (x, y): ({robot_pose.p.x()}, {robot_pose.p.y()})')
        print(f' current robot rotation (Yaw): {robot_pose.M.GetRPY()[2]}\n\n')

    print('\n Goal achived')
    return destination_accomplished



"""
                            CZĘŚĆ MANIPULACYJNA
"""


def grip(object_name, objects_data, move_group):
    print('\n\n podnoszenie start')
    obj_pose_kdl = get_model_pose(object_name)
    obj_pose = Pose()
    arm_above(move_group)
    base_pose = objects_data[object_name]['position']
    print(base_pose)
    obj_pose.position.x = base_pose[0] + 0.2
    obj_pose.position.y = base_pose[1] + 0.2
    obj_pose.position.z = base_pose[2] + 0.2
    move_arm(move_group, obj_pose)
    obj_pose.orientation.x, obj_pose.orientation.y, obj_pose.orientation.z, obj_pose.orientation.w = obj_pose_kdl.M.GetQuaternion()
    obj_pose.position.x, obj_pose.position.y, obj_pose.position.z = obj_pose_kdl.p
    move_arm(move_group, obj_pose)
    arm_homming(move_group)


def move_arm(move_group, pose_goal):
    print('\n moving arm\n')
    move_group.set_pose_target(pose_goal)
    success = move_group.go(wait=True)
    print(success)
    move_group.stop()
    move_group.clear_pose_targets()


def arm_homming(move_group):
    joint_goal = move_group.get_current_joint_values()
    # joint_goal[0] = 0.3
    joint_goal[1] = 0.8 #0.2
    joint_goal[2] = 0.5 #-1.335
    joint_goal[3] = -2.5 #-0.2
    joint_goal[4] = 1.924
    joint_goal[5] = -1.57
    joint_goal[6] = 1.369
    joint_goal[7] = 0.0
    print('\n homing\n')
    move_group.go(joint_goal, wait=True)
    move_group.stop()


def open_gripper(gripper_group):
    joint_goal = gripper_group.get_current_joint_values()
    #[0.001, 0.044] - [min, max]
    joint_goal[0] = 0.044
    joint_goal[1] = 0.044
    gripper_group.go(joint_goal, wait=True)
    gripper_group.stop()


def close_gripper(gripper_group):
    joint_goal = gripper_group.get_current_joint_values()
    #[0.001, 0.044] - [min, max]
    joint_goal[0] = 0.01
    joint_goal[1] = 0.01
    gripper_group.go(joint_goal, wait=False)
    time.sleep(4)
    gripper_group.stop()


def straight_arm(move_group):
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0.3
    joint_goal[1] = 0.07
    joint_goal[2] = 0.0
    joint_goal[3] = 0.0
    joint_goal[4] = 0.0
    joint_goal[5] = 0.0
    joint_goal[6] = 0.0
    joint_goal[7] = 0.0
    move_group.go(joint_goal, wait=True)
    move_group.stop()


def arm_above(move_group):
    print('\n setting arm above \n')
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0.33
    joint_goal[1] = 0.4
    joint_goal[2] = 0.75 #-1.335
    joint_goal[3] = -2 #-0.2
    joint_goal[4] = 1.97
    joint_goal[5] = 2.0
    joint_goal[6] = -0.5
    joint_goal[7] = 1.57
    move_group.go(joint_goal, wait=True)
    move_group.stop()


def pose_to_grip(objects_data, obj_name, above = 0.0):

    gripper_len = 0.17

    pose_kdl = get_link_pose(f'{obj_name}::link')
    pose_kdl_quat = objects_data[obj_name]['orientation']

    print(pose_kdl)

    object_quat = PyKDL.Rotation.Quaternion(pose_kdl_quat[0], pose_kdl_quat[1], pose_kdl_quat[2], pose_kdl_quat[3], )

    robot_pose = get_link_pose('tiago::base_footprint')
    robot_rot = robot_pose.M


    print(type(robot_rot))
    basic_rotation = robot_rot * PyKDL.Rotation.Quaternion(0.0, 0.0, 0.707, 0.707)

    print(type(basic_rotation))
    print(type(robot_rot.GetQuaternion()))

    target_pose = Pose()
    target_pose.orientation.x, target_pose.orientation.y, target_pose.orientation.z, target_pose.orientation.w = basic_rotation.GetQuaternion()

    target_pose.position.x, target_pose.position.y, target_pose.position.z = pose_kdl.p
    target_pose.position.z += above

    # if round(target_pose.orientation.z, 3) == 0.707:
    #     if round(target_pose.orientation.w, 3) == 0.707:
    #         target_pose.position.x += gripper_len
    #     elif round(target_pose.orientation.w, 3) == -0.707:
    #         target_pose.position.x -= gripper_len
    # if round(target_pose.orientation.w, 1) == 1.0:
    #     target_pose.position.y -= gripper_len
    # if round(target_pose.orientation.z, 1) == 1.0:
    #     target_pose.position.y += gripper_len
    # print(target_pose.position)

    return target_pose


"""
                            MAIN
"""


def main():
    for i in range(40):
        rospy.sleep(0.5)
        print('\n 0 0 0')

    rospy.init_node('tower', anonymous=False)
    pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

    robot_state = RobotState()
    rospy.Subscriber("reached_goal", Bool, robot_state.move_done_callback)

    for i in range(20):
        print('\n')


    moveit_commander.roscpp_initialize(sys.argv)
    move_group = moveit_commander.MoveGroupCommander("arm_torso")
    # gripper_group = moveit_commander.MoveGroupCommander("gripper")
    move_group.set_num_planning_attempts(50)
    move_group.set_goal_position_tolerance(0.2)
    move_group.set_goal_orientation_tolerance(100)
    print("\n\n\n\n\nReady to work\n")

    arm_homming(move_group)


    poses = get_positions()
    tab_positions = poses[0]
    obj_positions = poses[1]
    obj_orientations = []
    for i in range(3):
        obj_orientations.append(poses[2][i].M.GetQuaternion())
    
    obj_tab_relation = identify_location(tab_positions, obj_positions)

    objects_data = {
        'bottom':  {
            'id': 0,
            'table': obj_tab_relation['bottom'],
            'position': obj_positions[0],
            'orientation': obj_orientations[0]
        },
        'middle':  {
            'id': 1,
            'table': obj_tab_relation['middle'],
            'position': obj_positions[1],
            'orientation': obj_orientations[1]
        },
        'top':  {
            'id': 2,
            'table': obj_tab_relation['top'],
            'position': obj_positions[2],
            'orientation': obj_orientations[2]
        }
    }

    if objects_data['middle']['table'] != 'table_1':
        first_msg = destination_msg_create(0.3, 0.3, create_quaternion(0, 0, 3/4*math.pi))
        for i in range(10):
            pub.publish(first_msg)
            rospy.sleep(0.5)
    

    def seq(obj_name):
        route(obj_name, objects_data, pub)
        grip(obj_name, objects_data, move_group)
        rospy.sleep(5)

    

    seq('middle')
    seq('bottom')
    seq('top')
    seq('bottom')


    """
    
    route('middle', objects_data, pub)
    # middle_pose_above = pose_to_grip(objects_data, 'middle', 0.3)
    # move_arm(move_group, middle_pose_above)
    # print('\n\n\n\n is above\n\n')
    # middle_pose = pose_to_grip(objects_data, 'middle')
    # move_arm(move_group, middle_pose)

    middle_pose_kdl = get_model_pose('middle')
    middle_pose = Pose()
    arm_above(move_group)
    base_pose = objects_data['middle']['position']
    print(base_pose)
    middle_pose.position.x = base_pose[0] + 0.2
    middle_pose.position.y = base_pose[1] + 0.2
    middle_pose.position.z = base_pose[2] + 0.2
    move_arm(move_group, middle_pose)
    middle_pose.orientation.x, middle_pose.orientation.y, middle_pose.orientation.z, middle_pose.orientation.w = middle_pose_kdl.M.GetQuaternion()
    middle_pose.position.x, middle_pose.position.y, middle_pose.position.z = middle_pose_kdl.p
    move_arm(move_group, middle_pose)

    arm_homming(move_group)

    rospy.sleep(5)


    
    route('bottom', objects_data, pub)
    # bottom_pose_above = pose_to_grip(objects_data, 'bottom', 0.3)
    # move_arm(move_group, bottom_pose_above)
    # print('\n\n\n\n is above\n\n')
    # bottom_pose = pose_to_grip(objects_data, 'bottom')
    # move_arm(move_group, bottom_pose)
    # arm_homming(move_group)
    
    bottom_pose_kdl = get_model_pose('bottom')
    bottom_pose = Pose()
    bottom_pose.orientation.x, bottom_pose.orientation.y, bottom_pose.orientation.z, bottom_pose.orientation.w = bottom_pose_kdl.M.GetQuaternion()
    # bottom_pose.orientation.w = 1;
    bottom_pose.position.x, bottom_pose.position.y, bottom_pose.position.z = bottom_pose_kdl.p
    bottom_pose.position.z += 0.2
    arm_above(move_group)
    move_arm(move_group, bottom_pose)
    bottom_pose.position.z -= 0.2
    move_arm(move_group, bottom_pose)

    arm_homming(move_group)

    rospy.sleep(5)


    route('top', objects_data, pub)
    # top_pose_above = pose_to_grip(objects_data, 'top', 0.3)
    # move_arm(move_group, top_pose_above)
    # print('\n\n\n\n is above\n\n')
    # top_pose = pose_to_grip(objects_data, 'middle')
    # move_arm(move_group, top_pose)
    # arm_homming(move_group)

    top_pose_kdl = get_model_pose('top')
    top_pose = Pose()
    top_pose.orientation.x, top_pose.orientation.y, top_pose.orientation.z, top_pose.orientation.w = top_pose_kdl.M.GetQuaternion()
    # top_pose.orientation.w = 1;
    top_pose.position.x, top_pose.position.y, top_pose.position.z = top_pose_kdl.p
    top_pose.position.z += 0.2
    move_arm(move_group, top_pose)
    top_pose.position.z -= 0.2
    move_arm(move_group, top_pose)

    arm_homming(move_group)

    rospy.sleep(5)


    route('bottom', objects_data, pub)
    # bottom_pose_above = pose_to_grip(objects_data, 'bottom', 0.3)
    # move_arm(move_group, bottom_pose_above)
    # print('\n\n\n\n is above\n\n')
    # bottom_pose = pose_to_grip(objects_data, 'bottom')
    # move_arm(move_group, bottom_pose)
    # arm_homming(move_group)

    bottom_pose_kdl = get_model_pose('bottom')
    bottom_pose = Pose()
    bottom_pose.orientation.x, bottom_pose.orientation.y, bottom_pose.orientation.z, bottom_pose.orientation.w = bottom_pose_kdl.M.GetQuaternion()
    # bottom_pose.orientation.w = 1;
    bottom_pose.position.x, bottom_pose.position.y, bottom_pose.position.z = bottom_pose_kdl.p
    bottom_pose.position.z += 0.2
    move_arm(move_group, bottom_pose)
    bottom_pose.position.z -= 0.2
    move_arm(move_group, bottom_pose)

    arm_homming(move_group)

    rospy.sleep(5)
    """
    
    return 0
    




if __name__ == "__main__":
    exit(main())
