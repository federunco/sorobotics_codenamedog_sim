import pybullet as p
import pybullet_data
import time
import pathlib

GRAVITY = -9.8 # non facciamo gli ingegneri va
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

jointFrictionForce = 1
for joint in range(p.getNumJoints(botID)):
    p.setJointMotorControl2(botID, joint, p.POSITION_CONTROL, force=jointFrictionForce)

p.setRealTimeSimulation(1)
while(1):
    p.setGravity(0,0,GRAVITY)
    time.sleep(1/240.)

p.disconnect()