import pybullet as p
import pybullet_data
import time
import pathlib
import math
from ik.ik import IKSolver
from ik.leg import Leg

GRAVITY = -9.81 # non facciamo gli ingegneri va
dt = 1e-3
iters = 2000
exc_path = pathlib.Path(__file__).parent.absolute()
physicsClient = p.connect(p.GUI)

p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, GRAVITY)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,1]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
p.setAdditionalSearchPath(str(exc_path))

botID = p.loadURDF("./mech/sorobotics_dog.urdf",cubeStartPos, cubeStartOrientation)
p.stepSimulation()
cubePos, cubeOrn = p.getBasePositionAndOrientation(botID)

joints_index = {}
for joint in range(p.getNumJoints(botID)):
    joint_info = p.getJointInfo(botID, joint)
    # we only want moving joints indexes, not the fixed ones
    if joint_info[2] == p.JOINT_REVOLUTE:
        joints_index[joint_info[1].decode("utf-8")] = joint

print(joints_index)

def move_leg(leg_name, alpha, beta, gamma, spd):
    p.setJointMotorControl2(botID, joints_index[leg_name + '_shoulder'], p.POSITION_CONTROL, targetPosition=gamma-math.pi/2, force=1000, maxVelocity=spd)
    p.setJointMotorControl2(botID, joints_index[leg_name + '_leg'], p.POSITION_CONTROL, targetPosition=beta-math.pi/2, force=1000, maxVelocity=spd)
    p.setJointMotorControl2(botID, joints_index[leg_name + '_foot'], p.POSITION_CONTROL, targetPosition=alpha-math.pi, force=1000, maxVelocity=spd)



"""
jointFrictionForce = 1

for joint in range(p.getNumJoints(botID)):
    p.setJointMotorControl2(botID, joint, p.POSITION_CONTROL, force=jointFrictionForce)
"""

# add friction to robot feet
'''
p.changeDynamics(botID, joints_index['front_left_foot'], lateralFriction=2)
p.changeDynamics(botID, joints_index['front_right_foot'], lateralFriction=2)
p.changeDynamics(botID, joints_index['rear_left_foot'], lateralFriction=2)
p.changeDynamics(botID, joints_index['rear_right_foot'], lateralFriction=2)
'''

# set base position

ik = IKSolver(simulated_environment=True)
leg_l = Leg()
leg_r = Leg(right_axis=True)
alpha_l, beta_l, gamma_l = ik.solve(leg_l, 0, 5.5, 15)
alpha_r, beta_r, gamma_r = ik.solve(leg_l, 0, 5.5, 15)

spd=1

move_leg('front_left', alpha_l, beta_l, gamma_l, spd)
move_leg('rear_left', alpha_l, beta_l, gamma_l, spd)
move_leg('front_right', alpha_r, beta_r, gamma_r, spd)
move_leg('rear_right', alpha_r, beta_r, gamma_r, spd)

p.setRealTimeSimulation(1)
while(1):
    #p.setGravity(0,0,GRAVITY)
    time.sleep(1/240.)

p.disconnect()