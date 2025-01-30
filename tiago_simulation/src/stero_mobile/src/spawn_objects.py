#!/usr/bin/env python3

# Copyright (c) 2023, Robot Control and Pattern Recognition Group,
# Institute of Control and Computation Engineering
# Warsaw University of Technology
#
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the Warsaw University of Technology nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
# Author: Dawid Seredynski
#

import rospy
import PyKDL
import random
import math

from gazebo_msgs.srv import SpawnModel, GetLinkState
from geometry_msgs.msg import Pose, Point, Quaternion


def getCuboidInertiaParameters(mass, sx, sy, sz):
    ixx = 1.0/12.0 * mass * (sy*sy + sz*sz)
    iyy = 1.0/12.0 * mass * (sx*sx + sz*sz)
    izz = 1.0/12.0 * mass * (sx*sx + sy*sy)
    return ixx, 0.0, 0.0, iyy, 0.0, izz, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0


def getInertialSdf(mass, ixx, ixy, ixz, iyy, iyz, izz, x, y, z, rr, rp, ry):
    sdf = '<inertial><mass>{}</mass>'.format(mass)
    sdf += '<inertia><ixx>{}</ixx><ixy>{}</ixy><ixz>{}</ixz>'.format(ixx, ixy, ixz)
    sdf += '<iyy>{}</iyy><iyz>{}</iyz><izz>{}</izz></inertia>'.format(iyy, iyz, izz)
    sdf += '<pose>{} {} {} {} {} {}</pose></inertial>'.format(x, y, z, rr, rp, ry)
    return sdf


def getBoxGeomSdf(sx, sy, sz):
    return '<geometry><box><size>{} {} {}</size></box></geometry>'.format(sx, sy, sz)


def getCylinderGeomSdf(radius, length):
    return '<geometry><cylinder><radius>{}</radius><length>{}</length></cylinder></geometry>'.format(
                                                                                    radius, length)


def getModelSdf(model_type, name, mass, sx, sy, sz, x, y, z, rr, rp, ry, color):
    sdf = '<sdf version=\'1.7\'><model name=\'{}\'>'.format(name)
    sdf += '<pose>{} {} {} {} {} {}</pose>'.format(x, y, z, rr, rp, ry)
    sdf += '<link name=\'link\'>'
    if model_type == 'box':
        ixx, ixy, ixz, iyy, iyz, izz, x, y, z, rr, rp, ry = getCuboidInertiaParameters(mass, sx, sy, sz)
    elif model_type == 'cylinder':
        # The parameters are almost the same as for cuboid
        ixx, ixy, ixz, iyy, iyz, izz, x, y, z, rr, rp, ry = getCuboidInertiaParameters(mass, sx, sy, sz)
    else:
        raise Exception('Wrong type of model')

    sdf += getInertialSdf(mass, ixx, ixy, ixz, iyy, iyz, izz, x, y, z, rr, rp, ry)
    sdf += '<collision name=\'collision\'>'
    if model_type == 'box':
        sdf += getBoxGeomSdf(sx, sy, sz)
    elif model_type == 'cylinder':
        radius = sx
        length = sz
        sdf += getCylinderGeomSdf(radius, length)
    else:
        raise Exception('Wrong type of model')
    sdf += '<max_contacts>10</max_contacts><surface><contact><ode/></contact>'
    sdf += '<bounce/><friction><torsional><ode/></torsional><ode/></friction></surface></collision>'
    sdf += '<visual name=\'visual\'>'
    if model_type == 'box':
        sdf += getBoxGeomSdf(sx, sy, sz)
    elif model_type == 'cylinder':
        radius = sx
        length = sz
        sdf += getCylinderGeomSdf(radius, length)
    else:
        raise Exception('Wrong type of model')
    sdf += '<material><script><name>Gazebo/{}</name>'.format(color)
    sdf += '<uri>file://media/materials/scripts/gazebo.material</uri>'
    sdf += '</script></material></visual><self_collide>0</self_collide>'
    sdf += '<enable_wind>0</enable_wind><kinematic>0</kinematic></link></model></sdf>'
    return sdf


def spawnModel(model_type, model_name, mass, size, pose, color):
    model_xml = getModelSdf(model_type, model_name, mass, size.x(), size.y(), size.z(), 0, 0, 0, 0, 0, 0, color)

    x = pose.p.x()
    y = pose.p.y()
    z = pose.p.z()
    q = pose.M.GetQuaternion()

    initial_pose = Pose(Point(x, y, z), Quaternion(q[0], q[1], q[2], q[3]))
    robot_namespace = '/'
    reference_frame = 'world'

    rospy.wait_for_service('/gazebo/spawn_sdf_model')
    try:
        spawn_sdf_model = rospy.ServiceProxy('/gazebo/spawn_sdf_model', SpawnModel)
        resp = spawn_sdf_model(model_name, model_xml, robot_namespace, initial_pose, reference_frame)
        print(resp.status_message)
        return resp.success
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return False


def getLinkPose(link_full_name):
    rospy.wait_for_service('/gazebo/get_link_state')
    try:
        get_link_state = rospy.ServiceProxy('/gazebo/get_link_state', GetLinkState)
        resp = get_link_state(link_full_name, '')
        print(resp.status_message)
        if resp.success:
            pos = resp.link_state.pose.position
            quat = resp.link_state.pose.orientation
            return PyKDL.Frame(PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w),
                                PyKDL.Vector(pos.x, pos.y, pos.z))
        # else:
        return None
    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        return None


def main():
    rospy.sleep(2)
    rospy.init_node('stero_spawn_objects', anonymous=False)
    rospy.sleep(0.5)
    
    table_size = {  'table_0': PyKDL.Vector(1.0, 0.6, 0.5),
                    'table_1': PyKDL.Vector(1.4, 0.4, 0.8),
                    'table_2': PyKDL.Vector(1.0, 1.0, 0.75)}

    object_size = { 'bottom': PyKDL.Vector(0.1, 0.02, 0.1),
                    'middle': PyKDL.Vector(0.06, 0.06, 0.06),
                    'top'   : PyKDL.Vector(0.03, 0.03, 0.2)}

    object_type = { 'bottom':'box',
                    'middle':'box',
                    'top'   :'cylinder'}

    object_color = {'bottom': 'Red',
                    'middle': 'Green',
                    'top'   : 'Blue'}

    table_pose = {}
    for table_name in table_size:
        p = getLinkPose('{}::link'.format(table_name))
        table_pose[table_name] = p

    up_spawn_offset = 0.01
    margin = 0.05

    table_names = list(table_size.keys())
    for object_name in object_size:
        table_name = random.choice(table_names)
        table_names.remove(table_name)

        T_W_O = table_pose[table_name]
        x = random.uniform( -table_size[table_name].x()/2+margin,
                            table_size[table_name].x()/2-margin)
        y = random.uniform( -table_size[table_name].y()/2+margin,
                            table_size[table_name].y()/2-margin)
        z = table_size[table_name].z()/2 + object_size[object_name].z()/2 + up_spawn_offset
        spawn_point_O = PyKDL.Vector(x, y, z)

        
        spawn_point_W = T_W_O * spawn_point_O
        spawn_pose_W = PyKDL.Frame(PyKDL.Rotation.RotZ(random.uniform(-math.pi, math.pi)),
                                                                            spawn_point_W)
        rospy.loginfo('Spawning object "{}" on "{}" at position {}'.format(object_name, table_name,
                                                                                spawn_point_W))
        if not spawnModel(object_type[object_name], object_name, 0.2, object_size[object_name],
                                                        spawn_pose_W, object_color[object_name]):
            raise Exception('could not spawn object')

    return 0


if __name__ == "__main__":
    exit(main())
