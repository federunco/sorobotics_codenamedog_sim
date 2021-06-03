# Copyright 2021 I.S. "A. Sobrero" - SoRobotics Team. All rights reserved.
# Use of this source code is governed by the GPL 3.0 license that can be
# found in the LICENSE file.

import pybullet as p
import numpy as np
import pybullet_data
import time
import pathlib
import math
from ik.ik import IKSolver
from ik.leg import Leg
from gaitplanner.gait_planner import GaitPlanner
import numpy as np

GRAVITY = -9.81 # non facciamo gli ingegneri va
dt = 1e-3
iters = 2000
exc_path = pathlib.Path(__file__).parent.absolute()
physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, GRAVITY)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0.25]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
p.setAdditionalSearchPath(str(exc_path))

plane_startpos = [0,0,0]
plane_startori = p.getQuaternionFromEuler([0,-60,0])

test_visual = p.createVisualShape(p.GEOM_BOX, halfExtents=[0.5,0.5,0.05])
test_collision = p.createCollisionShape(p.GEOM_BOX, halfExtents=[0.5,0.5,0.05])
'''test_body = p.createMultiBody(baseMass=0, baseCollisionShapeIndex=test_collision, \
baseVisualShapeIndex=test_visual, basePosition = [0, 0, 0], baseOrientation=plane_startori)
'''
botID = p.loadURDF("./mech/sorobotics_dog.urdf",cubeStartPos, cubeStartOrientation)
p.stepSimulation()
cubePos, cubeOrn = p.getBasePositionAndOrientation(botID)

joints_index = {}
fixed_index = {}    

for joint in range(p.getNumJoints(botID)):
    joint_info = p.getJointInfo(botID, joint)
    if joint_info[2] is p.JOINT_FIXED:
        fixed_index[joint_info[1].decode("utf-8")] = joint
    if joint_info[2] is p.JOINT_REVOLUTE:
        joints_index[joint_info[1].decode("utf-8")] = joint

# add friction to robot feet
p.changeDynamics(botID, fixed_index['rear_left_toe'], lateralFriction=1)
p.changeDynamics(botID, fixed_index['rear_right_toe'], lateralFriction=1)
p.changeDynamics(botID, fixed_index['front_left_toe'], lateralFriction=1)
p.changeDynamics(botID, fixed_index['front_right_toe'], lateralFriction=1)

def move_leg(leg_name, alpha, beta, gamma, spd):
    p.setJointMotorControl2(botID, joints_index[leg_name + '_shoulder'], p.POSITION_CONTROL, targetPosition=gamma-math.pi/2, force=1000, maxVelocity=spd)
    p.setJointMotorControl2(botID, joints_index[leg_name + '_leg'], p.POSITION_CONTROL, targetPosition=beta-math.pi/2, force=1000, maxVelocity=spd)
    p.setJointMotorControl2(botID, joints_index[leg_name + '_foot'], p.POSITION_CONTROL, targetPosition=alpha-math.pi, force=1000, maxVelocity=spd)

ik = IKSolver(simulated_environment=True)
leg_l = Leg()
leg_r = Leg(right_axis=True)

# set home position
try:
    alpha_l, beta_l, gamma_l = ik.solve(leg_l, 0, 5.5, 15)
    alpha_r, beta_r, gamma_r = ik.solve(leg_l, 0, 5.5, 15)

    spd=5

    move_leg('front_left', alpha_l, beta_l, gamma_l, spd)
    move_leg('rear_left', alpha_l, beta_l, gamma_l, spd)
    move_leg('front_right', alpha_r, beta_r, gamma_r, spd)
    move_leg('rear_right', alpha_r, beta_r, gamma_r, spd)
except:
    pass

p.setRealTimeSimulation(1)
i=0

#chor = Choreograph()

trot = GaitPlanner()

height = 15
#body frame to foot frame vector (0.08/-0.11 , -0.07 , -height)
bodytoFeet0 = np.matrix([[ 3 , -5.5 , -height],
                         [ 3 ,  5.5 , -height],
                         [-3, -5.5 , -height],
                         [-3 ,  5.5, -height]])

interval = 0.03  
start_time = time.time()
last_time = start_time
t = []
prev_err = 0
sum_err = 0

orn = np.array([0. , 0. , 0.])
pos = np.array([0. , 0. , 0.])
Upid_yorn = [0.]
Upid_y = [0.]
Upid_xorn = [0.]
Upid_x = [0.]
startTime = time.time()
lastTime = startTime
t = []
       
        
T = 0.5 #period of time (in seconds) of every step
offset = np.array([0. , 0.5 , 0.5 , 0.]) #defines the offset between each foot step in this order (FR,FL,BR,BL)

while(1):
    if (time.time() - last_time >= interval):
        last_time = time.time()
        t = time.time() - start_time
    i = i+1
    robot_pos, robot_orientation_quat = p.getBasePositionAndOrientation(botID)
    robot_euler = p.getEulerFromQuaternion(robot_orientation_quat)

    V = 0.4
    angle = 0
    forceAngle = 0
    Wrot = 0
    Vcompliant = 0

    bodytoFeet  = trot.loop(V + Vcompliant , angle + forceAngle , Wrot , T , offset , bodytoFeet0)

    bodyToFR = np.asarray([bodytoFeet[0,0],bodytoFeet[0,1],bodytoFeet[0,2]])
    bodyToFL = np.asarray([bodytoFeet[1,0],bodytoFeet[1,1],bodytoFeet[1,2]])
    bodyToBR = np.asarray([bodytoFeet[2,0],bodytoFeet[2,1],bodytoFeet[2,2]])
    bodyToBL = np.asarray([bodytoFeet[3,0],bodytoFeet[3,1],bodytoFeet[3,2]])

    print("L: {} R: {}".format(bodyToFL[1], bodyToFR[1]))

    try:
        alpha_fr, beta_fr, gamma_fr = ik.solve(leg_r, -bodyToFR[0], -bodyToFR[1] + 0, bodyToFR[2])
        alpha_br, beta_br, gamma_br = ik.solve(leg_r, -bodyToBR[0], -bodyToBR[1] + 0, bodyToBR[2])
        alpha_fl, beta_fl, gamma_fl = ik.solve(leg_l, -bodyToFL[0], -bodyToFL[1] + 0, bodyToFL[2])
        alpha_bl, beta_bl, gamma_bl = ik.solve(leg_l, -bodyToBL[0], -bodyToBL[1] + 0, bodyToBL[2])

        move_leg('front_left', alpha_fl, beta_fl, gamma_fl, spd)
        move_leg('rear_left', alpha_bl, beta_bl, gamma_bl, spd)
        move_leg('front_right', alpha_fr, beta_fr, gamma_fr, spd)
        move_leg('rear_right', alpha_br, beta_br, gamma_br, spd)
    except:
        pass

    p.stepSimulation()
    time.sleep(1/240.)

    #robot_pose = chor.loop(1, 0, 0, 0.4, offset, )
    #p.setGravity(0,0,GRAVITY)
    #time.sleep(1/240.)

p.disconnect()