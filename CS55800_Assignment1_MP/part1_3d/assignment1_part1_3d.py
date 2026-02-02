from __future__ import division
import pybullet as p
import pybullet_data
import numpy as np
import time
import argparse
import math
import os
import sys
import random


UR5_JOINT_INDICES = [0, 1, 2]


def set_joint_positions(body, joints, values):
    assert len(joints) == len(values)
    for joint, value in zip(joints, values):
        p.resetJointState(body, joint, value)


def draw_sphere_marker(position, radius, color):
   vs_id = p.createVisualShape(p.GEOM_SPHERE, radius=radius, rgbaColor=color)
   marker_id = p.createMultiBody(basePosition=position, baseCollisionShapeIndex=-1, baseVisualShapeIndex=vs_id)
   return marker_id


def remove_marker(marker_id):
   p.removeBody(marker_id)


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--birrt', action='store_true', default=False)
    parser.add_argument('--smoothing', action='store_true', default=False)
    args = parser.parse_args()
    return args

#your implementation starts here
#refer to the handout about what the these functions do and their return type
###############################################################################
class RRT_Node:
    def __init__(self, conf):
        self.parent = None
        self.children = []
        self.conf = conf

    def set_parent(self, parent):
        self.parent = parent

    def add_child(self, child):
        self.children.append(child)

def sample_conf():
    return (random.uniform(-2 * np.pi, 2 * np.pi), random.uniform(-2 * np.pi, 2 * np.pi), random.uniform(-np.pi, np.pi))
   
def find_nearest(rand_node, node_list):
    closest_dist = np.inf
    closest_idx = None
    for index, node in enumerate(node_list):
        curr_dist = np.linalg.norm(np.array(rand_node) - np.array(node.conf))
        if closest_dist > curr_dist:
            closest_dist = curr_dist
            closest_idx = index
    
    return closest_idx
        
def steer_to(rand_node, nearest_node):
    dist = np.linalg.norm(np.array(rand_node) - np.array(nearest_node.conf))
    incr = (np.array(rand_node) - np.array(nearest_node.conf)) * (0.05 / dist)
    curr_conf = np.array(nearest_node.conf)
    while dist >= 0.05:
        curr_conf += incr
        dist -= 0.05
        if collision_fn(tuple(curr_conf)):
            return True
    
    return collision_fn(rand_node)

def steer_to_until(rand_node, nearest_node):
    step = 0.05
    dist = np.linalg.norm(np.array(rand_node) - np.array(nearest_node.conf))
    incr = (np.array(rand_node) - np.array(nearest_node.conf)) * (step / dist)
    curr_conf = np.array(nearest_node.conf)
    while dist >= step:
        curr_conf += incr
        dist -= step
        if collision_fn(tuple(curr_conf)):
            if ((curr_conf - incr) is np.array(nearest_node.conf)):
                return None
            else:
                return curr_conf - incr
    
    if collision_fn(rand_node):
        return curr_conf
    else:
        return rand_node

def RRT():

    node_list = [RRT_Node(start_conf)]
    while True:
        rand_conf = sample_conf()
        nearest_idx = find_nearest(rand_conf, node_list)
        if not steer_to(rand_conf, node_list[nearest_idx]):
            node_list.append(RRT_Node(rand_conf))
            node_list[nearest_idx].add_child(node_list[-1])
            node_list[-1].set_parent(node_list[nearest_idx])
            if np.linalg.norm(np.array(rand_conf) - np.array(goal_conf)) < 0.5:
                break
    
    path_conf = [goal_conf]
    curr_node = node_list[-1]
    while curr_node != None:
        path_conf.append(curr_node.conf)
        curr_node = curr_node.parent
    
    return path_conf[::-1]

def connect(rand_node, node_list):
    nearest_idx = find_nearest(rand_node, node_list)
    collision = steer_to(rand_node, node_list[nearest_idx])

def BiRRT():

    start_node_list = [RRT_Node(start_conf)]
    end_node_list = [RRT_Node(goal_conf)]
    flag = False

    while True:
        flag = not flag
        if flag:
            rand_conf = sample_conf()
            nearest_idx = find_nearest(rand_conf, start_node_list)
            no_col_conf = steer_to_until(rand_conf, start_node_list[nearest_idx])
            if no_col_conf is not None:
                start_node_list.append(RRT_Node(no_col_conf))
                start_node_list[nearest_idx].add_child(start_node_list[-1])
                start_node_list[-1].set_parent(start_node_list[nearest_idx])
                nearest_other_idx = find_nearest(no_col_conf, end_node_list)
                if steer_to(no_col_conf, end_node_list[nearest_other_idx]):
                    break
        else:
            rand_conf = sample_conf()
            nearest_idx = find_nearest(rand_conf, end_node_list)
            no_col_conf = steer_to_until(rand_conf, end_node_list[nearest_idx])
            if no_col_conf is not None:
                end_node_list.append(RRT_Node(no_col_conf))
                end_node_list[nearest_idx].add_child(end_node_list[-1])
                end_node_list[-1].set_parent(end_node_list[nearest_idx])
                nearest_other_idx = find_nearest(no_col_conf, start_node_list)
                if steer_to(no_col_conf, start_node_list[nearest_other_idx]):
                    break
        
    
    end_path = []
    start_path = []

    end_node = end_node_list[nearest_other_idx] if flag else end_node_list[-1]
    start_node = start_node_list[-1] if flag else start_node_list[nearest_other_idx]
    
    while end_node != None:
        end_path.append(end_node.conf)
        end_node = end_node.parent
    
    while start_node != None:
        start_path.append(start_node.conf)
        start_node = start_node.parent
    
    return start_path[::-1] + end_path

def BiRRT_smoothing():
    start_node_list = [RRT_Node(start_conf)]
    end_node_list = [RRT_Node(goal_conf)]
    flag = False

    while True:
        flag = not flag
        if flag:
            rand_conf = sample_conf()
            nearest_idx = find_nearest(rand_conf, start_node_list)
            no_col_conf = steer_to_until(rand_conf, start_node_list[nearest_idx])
            if no_col_conf is not None:
                start_node_list.append(RRT_Node(no_col_conf))
                start_node_list[nearest_idx].add_child(start_node_list[-1])
                start_node_list[-1].set_parent(start_node_list[nearest_idx])
                nearest_other_idx = find_nearest(no_col_conf, end_node_list)
                if steer_to(no_col_conf, end_node_list[nearest_other_idx]):
                    break
        else:
            rand_conf = sample_conf()
            nearest_idx = find_nearest(rand_conf, end_node_list)
            no_col_conf = steer_to_until(rand_conf, end_node_list[nearest_idx])
            if no_col_conf is not None:
                end_node_list.append(RRT_Node(no_col_conf))
                end_node_list[nearest_idx].add_child(end_node_list[-1])
                end_node_list[-1].set_parent(end_node_list[nearest_idx])
                nearest_other_idx = find_nearest(no_col_conf, start_node_list)
                if steer_to(no_col_conf, start_node_list[nearest_other_idx]):
                    break
        
    
    end_path = []
    start_path = []

    end_node = end_node_list[nearest_other_idx] if flag else end_node_list[-1]
    start_node = start_node_list[-1] if flag else start_node_list[nearest_other_idx]
    
    while end_node != None:
        end_path.append(end_node.conf)
        end_node = end_node.parent
    
    while start_node != None:
        start_path.append(start_node.conf)
        start_node = start_node.parent
    
    path_conf = start_path[::-1] + end_path

    while len(path_conf) > 3:
        path_len = len(path_conf)
        rand_idx_1 = random.randint(0, path_len - 1)
        rand_idx_2 = random.randint(0, path_len - 1)

        if not steer_to(path_conf[rand_idx_1], RRT(path_conf[rand_idx_2])):
            if rand_idx_2 > rand_idx_1:
                path_conf = path_conf[:rand_idx_1 + 1] + path_conf[rand_idx_2:]
            else:
                path_conf = path_conf[:rand_idx_2 + 1] + path_conf[rand_idx_1:]
    
    return path_conf

###############################################################################
#your implementation ends here

if __name__ == "__main__":
    args = get_args()

    # set up simulator
    physicsClient = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setPhysicsEngineParameter(enableFileCaching=0)
    p.setGravity(0, 0, -9.8)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI, False)
    p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, True)
    p.resetDebugVisualizerCamera(cameraDistance=1.400, cameraYaw=58.000, cameraPitch=-42.200, cameraTargetPosition=(0.0, 0.0, 0.0))

    # load objects
    plane = p.loadURDF("plane.urdf")
    ur5 = p.loadURDF('assets/ur5/ur5.urdf', basePosition=[0, 0, 0.02], useFixedBase=True)
    obstacle1 = p.loadURDF('assets/block.urdf',
                           basePosition=[1/4, 0, 1/2],
                           useFixedBase=True)
    obstacle2 = p.loadURDF('assets/block.urdf',
                           basePosition=[2/4, 0, 2/3],
                           useFixedBase=True)
    obstacles = [plane, obstacle1, obstacle2]

    # start and goal
    start_conf = (-0.813358794499552, -0.37120422397572495, -0.754454729356351)
    start_position = (0.3998897969722748, -0.3993956744670868, 0.6173484325408936)
    goal_conf = (0.7527214782907734, -0.6521867735052328, -0.4949270744967443)
    goal_position = (0.35317009687423706, 0.35294029116630554, 0.7246701717376709)
    goal_marker = draw_sphere_marker(position=goal_position, radius=0.02, color=[1, 0, 0, 1])
    set_joint_positions(ur5, UR5_JOINT_INDICES, start_conf)

    
		# place holder to save the solution path
    path_conf = None

    # get the collision checking function
    from collision_utils import get_collision_fn
    collision_fn = get_collision_fn(ur5, UR5_JOINT_INDICES, obstacles=obstacles,
                                       attachments=[], self_collisions=True,
                                       disabled_collisions=set())

    if args.birrt:
        if args.smoothing:
            # using birrt with smoothing
            path_conf = BiRRT_smoothing()
        else:
            # using birrt without smoothing
            path_conf = BiRRT()
    else:
        # using rrt
        path_conf = RRT()

    if path_conf is None:
        # pause here
        input("no collision-free path is found within the time budget, finish?")
    else:
        # execute the path
        while True:
            for q in path_conf:
                set_joint_positions(ur5, UR5_JOINT_INDICES, q)
                time.sleep(0.5)
            input("Keep Going?")
