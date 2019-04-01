import gym.spaces
import numpy as np
from libs.yarp_icub import yarp
from libs.yarp_icub import icub
import time
import os
import sys
import subprocess
import math
#from env.Gazebo.CExtension.yarpMath.cYarpMath import dcm2axis


class iCub(object):
    def __init__(self, motor, img, obs_high_dim, obs_low_dim, action_dim, robot_name, **kwargs):

        # gym related settings: high and low observation space
        self.motor = motor
        self.img = img

        self.obs_high_dim = obs_high_dim
        self.obs_low_dim = obs_low_dim

        # setup action space
        # in RL setting the actions are usually normalized to be in the region of -1 and +1

        high = np.ones([action_dim])
        self.action_space = gym.spaces.Box(-high, high)

        # setup observation space
        high = np.inf * np.ones([obs_low_dim])
        if self.img and self.motor:
            self.observation_space = gym.spaces.Dict(
                {"img": gym.spaces.Box(low=0, high=255, shape=obs_high_dim, dtype=np.uint8),
                 "motor": gym.spaces.Box(-high, high)})

        elif self.img and not self.motor:
            self.observation_space = gym.spaces.Box(low=0, high=255, shape=obs_high_dim, dtype=np.uint8)

        elif not self.img and self.motor:
            self.observation_space = gym.spaces.Box(-high, high)

        else:
            raise ValueError("You need to output observations")

        # yarp and icub settings: activation part

        self.left_arm_driver = None
        self.left_arm_IPositionControl = None
        self.left_arm_IEncoders = None
        self.LencsP = None
        self.LencsV = None
        self.LencsA = None
        self.left_limb = None
        self.left_chain = None

        self.right_arm_clientCartCtrl = None
        self.right_arm_CartesianControl = None  # it is the pointer you should use.
        self.right_arm_IPositionControl = None
        self.right_arm_IEncoders = None
        self.finger = None          #added finger

        self.head_driver = None

        self.torso_driver = None

        self.LCameraPort = None
        self.RCameraPort = None

        self.G = None  # GazeboClient
        self.target_location = None  # the location of the target, np.array (3,)

        self.robot_name = robot_name
        self.firstInitialization = False  # flag to indicate if we have robot connected

        self.desiredEndEffectorPosition = None  # the RL planner's decision will change this vector
        self.desiredEndEffectorAngle = None

        self.currentEEPose = None

        # all paramters for the iCub
        self.activationPart = kwargs['activationPart']

        self.desiredEEPos = []  # for plotting
        self.actualEEPos = []  # for plotting

    def reset(self):
        if not self.firstInitialization:  # we only do this once
            self.firstInitialization = True
            self.openPart()  # open iCub parts, necessary software, open port, and build yarp connection
            self.G.simulationControl("p", 1)
            self.G.environment_creator()

            '''
			Information about iCub joint
			'''

        self.robot_specific_reset()
        state = self.calc_state()  # get low dimensional space

        return state

    def invKin(self, orientation, position):
        pos = yarp.Vector(3)
        orn = yarp.Vector(4)

        pos.set(0,position[0])
        pos.set(1,position[1])
        pos.set(2,position[2])
        orn.set(0, orientation[0])
        orn.set(1, orientation[1])
        orn.set(2, orientation[2])
        orn.set(3, orientation[3])
        if not self.right_arm_CartesianControl.goToPosition(pos):
            print("Cannot satisfy the position and orientaion requirements")

        return

    def robot_specific_reset(self):
        raise NotImplementedError()

    def calc_state(self):
        raise NotImplementedError()

    def applyAction(self, eeCommands):
        raise NotImplementedError()

    def openPart(self):
        raise NotImplementedError()


class iCubG(iCub):
    """
	This is a derived class for iCub grasping task
	"""

    def __init__(self, robot_name='body0', action_dim=3, obs_high_dim=(160, 240, 1), obs_low_dim=9,
                 motor=True, img=True, **kwargs):

        iCub.__init__(self, robot_name=robot_name, action_dim=action_dim,
                      obs_high_dim=obs_high_dim, obs_low_dim=obs_low_dim, motor=motor, img=img, **kwargs)

    def robot_specific_reset(self):

        isDone = False
        x0 = yarp.Vector(3)  # yarp position Vector
        o0 = yarp.Vector(4)  # yarp orientation Vector

        while not isDone:
            self.right_arm_CartesianControl.setInTargetTol(4)
            time.sleep(0.1)
            isDone = self.right_arm_CartesianControl.checkMotionDone()  # solver will reset itself

        # After the cartesian solver resets, we send command to the Gazebo for world reset
        self.G.simulationControl("r")
        # change the tolerance back
        self.right_arm_CartesianControl.setInTargetTol(0.001)

        objectx, objecty, objectz = self.G.blockRandomizer()
        self.G.modelTask(options="m", modelName="cube", xyzrpy=tuple([objectx, objecty, objectz, 0.0, 0.0, 0.0]))
        time.sleep(0.5)



        if not self.right_arm_CartesianControl.getPose(x0, o0):
            print("Cannot get Pose from the iCub")


        self.desiredEndEffectorPosition = [x0.get(0), x0.get(1), x0.get(2)]


        self.desiredEEPos = []  # for plotting
        self.actualEEPos = []  # for plotting

        self.target_location = [objectx, objecty, objectz]
        self.to_target_vec = [math.inf,math.inf,math.inf]

    def calc_state(self):


        state = []
        pos = yarp.Vector(3)
        orn = yarp.Vector(4)

        # TODO: design the state for the problem

        if not self.right_arm_CartesianControl.getPose(pos,orn):
            print("Cannot get Pose from the iCub")
        currentEndEffectorPosition = [pos.get(0), pos.get(1), pos.get(2)]
        for i in range(3):
            self.to_target_vec[i] = currentEndEffectorPosition[i] - self.target_location[i]
        state.append(self.to_target_vec)
        #print('this is state', state)
        # state.extend(list(gripperPos))

        return np.array(state)

    def applyAction(self, position):

        self.invKin([0, 1, 0, 3.14], position)  # apply actions using iCub cartesian control

        return

    def openPart(self):
        # launch cartesian solver
        subprocess.Popen("yarprobotinterface", cwd="env/Gazebo/Config")

        while not yarp.Network.exists("/simCartesianControl/right_arm/rpc:o"):
            time.sleep(0.5)
        print("yarp robot interface launched")
        subprocess.Popen(["iKinCartesianSolver", "--part", "right_arm"], cwd="env/Gazebo/Config")
        while not yarp.Network.exists("/cartesianSolver/right_arm/in"):
            time.sleep(0.5)
        print("cartesian solver launched")
        time.sleep(1)  # wait for these two programs

        # use activation part to open drivers
        # activationPart: [left_arm, right_arm, head, torso, left_camera, right_camera]
        if self.activationPart[0]:
            left_arm_options = yarp.Property()
            self.left_arm_driver = yarp.PolyDriver()
            # set the poly driver options
            left_arm_options.put("robot", "icub")
            left_arm_options.put("device", "remote_controlboard")
            left_arm_options.put("local", "/left_arm/client")
            left_arm_options.put("remote", "/icub/left_arm")

            # wait for port being ready
            while not yarp.Network.exists("/icub/left_arm/command:i"):
                time.sleep(0.5)
            print("The simulator left-arm ports are ready for connection")

            # opening the drivers
            print('Opening the motor driver for left arm')
            self.left_arm_driver.open(left_arm_options)
            if not self.left_arm_driver.isValid():
                print('Cannot open the left arm driver!')

                sys.exit()

            # Create position control object
            self.left_arm_IPositionControl = self.left_arm_driver.viewIPositionControl()
            # Create encoder object
            self.left_arm_IEncoders = self.left_arm_driver.viewIEncoders()

            if self.left_arm_IPositionControl is None or self.left_arm_IEncoders is None:
                print('Cannot view motor positions/encoders for left arm')

                sys.exit()

            # there are 16 DOF (including fingers) on one icub arm. So, the next 3 vectors have dimension (16,)
            # the vector that store the encoder position value for all controllable axes for left arm
            self.LencsP = yarp.Vector(self.left_arm_IEncoders.getAxes())
            # velocity
            self.LencsV = yarp.Vector(self.left_arm_IEncoders.getAxes())
            # acceleration
            self.LencsA = yarp.Vector(self.left_arm_IEncoders.getAxes())


            # Create kinematics object
            self.left_limb = icub.iCubArm("left")
            self.left_chain = self.left_limb.asChain()

        if self.activationPart[1]:
            # Right arm cartesian control
            # set the poly driver options
            right_car_arm_options = yarp.Property()
            right_car_arm_options.put("robot", "icub")
            right_car_arm_options.put("device", "cartesiancontrollerclient")
            right_car_arm_options.put("remote", "/icub/cartesianController/right_arm")
            right_car_arm_options.put("local", "/client/right_arm")
            self.right_arm_clientCartCtrl = yarp.PolyDriver()

            # wait for port being ready
            while not yarp.Network.exists("/icub/right_arm/command:i") \
                    or not yarp.Network.exists("/cartesianSolver/right_arm/right_arm/command:o") \
                    or not yarp.Network.exists("/simCartesianControl/right_arm/rpc:o"):
                time.sleep(0.5)
            print("The simulator right-arm cart ports are ready for connection")

            # opening the drivers
            print('Opening the motor driver for right arm cart')
            self.right_arm_clientCartCtrl.open(right_car_arm_options)
            if not self.right_arm_clientCartCtrl.isValid():
                print('Cannot open the right cart arm driver!')
                sys.exit()

            # Create cartesian position control object
            self.right_arm_CartesianControl = self.right_arm_clientCartCtrl.viewICartesianControl()
            self.right_arm_CartesianControl.setInTargetTol(0.001)

            if self.right_arm_CartesianControl is None:
                print('Cannot view cartesian control for right arm')
                sys.exit()
            time.sleep(3)

            ##defining a different end effector
            # set the poly driver options
            right_arm_options = yarp.Property()
            right_arm_options.put("robot", "icub")
            right_arm_options.put("device", "remote_controlboard")
            right_arm_options.put("remote", "/icub/right_arm")
            right_arm_options.put("local", "/right_arm/client")
            self.right_arm_driver = yarp.PolyDriver()

            # wait for port being ready
            while not yarp.Network.exists("/icub/right_arm/command:i"):
                time.sleep(0.5)
            print("The simulator right-arm ports are ready for connection")

            # opening the drivers
            print('Opening the motor driver for right arm')
            self.right_arm_driver.open(right_arm_options)
            if not self.right_arm_driver.isValid():
                print('Cannot open the right arm driver!')
                sys.exit()
            self.right_arm_IEncoders = self.right_arm_driver.viewIEncoders()
            if self.right_arm_IEncoders is None:
                print('Cannot view IEncoders for right arm')
                sys.exit()
            time.sleep(3)

            jnts = self.right_arm_IEncoders.getAxes()
            encs = yarp.Vector(jnts)
            self.right_arm_IEncoders.getEncoders(encs.data())
            joints = yarp.Vector(jnts)
            self.finger = icub.iCubFinger("right_middle")
            self.finger.getChainJoints(encs, joints)
            for i in range(jnts):
                joints.set(i,math.pi / 180 * joints.get(i))
            tipFrame = self.finger.getH(joints)
            tip_x = getCol(tipFrame,3)
            tip_o = dcm2axis(tipFrame)
            self.right_arm_CartesianControl.attachTipFrame(tip_x, tip_o)
            print("Finished defining a different effector")

        if self.activationPart[2]:
            head_options = yarp.Property()
            self.head_driver = yarp.PolyDriver()
            #set the poly driver options
            head_options.put("robot", "icub")
            head_options.put("device", "remote_controlboard")
            head_options.put("local", "/head/client")
            head_options.put("remote", "/icub/head")

            # wait for port being ready
            while not yarp.Network.exists("/icub/head/command:i"):
                time.sleep(0.5)
            print("The simulator head ports are ready for connection")

            # opening the drivers
            print('Opening the motor driver for head')
            self.head_driver.open(head_options)

            if not self.head_driver.isValid():
                print('Cannot open the head driver!')
                sys.exit()

        if self.activationPart[3]:
            torso_options = yarp.Property()
            self.torso_driver = yarp.PolyDriver()
            # set the poly driver options
            torso_options.put("robot", "icub")
            torso_options.put("device", "remote_controlboard")
            torso_options.put("local", "/torso/client")
            torso_options.put("remote", "/icub/torso")

            # wait for port being ready
            while not yarp.Network.exists("/icub/torso/command:i"):
                time.sleep(0.5)
            print("The simulator torso ports are ready for connection")

            # opening the drivers
            print('Opening the motor driver for torso')
            self.torso_driver.open(torso_options)
            if not self.torso_driver.isValid():
                print('Cannot open the torso driver!')

                sys.exit()

        if self.activationPart[4]:
            # Create a port and connect it to the iCub simulator left camera
            self.LCameraPort = yarp.BufferedPortImageRgb()
            self.LCameraPort.open("/LImagePort")
            yarp.Network.connect("/icub/cam/left", "/LImagePort")

        if self.activationPart[5]:
            # Create a port and connect it to the iCub simulator right camera
            self.RCameraPort = yarp.BufferedPortImageRgb()
            self.RCameraPort.open("/RImagePort")
            yarp.Network.connect("/icub/cam/right", "/RImagePort")


        self.G.modelTaskClient.open("/GazeboClient/modelTaskClient")
        if not self.G.modelTaskClient.addOutput("/gazeboInterface/modelTaskServer"):
            print("Failed to connect to the gazeboInterface")

            exit(0)
        self.G.simulationControlClient.open("/GazeboClient/simulationControlClient")
        if not self.G.simulationControlClient.addOutput("/gazeboInterface/simulationControlServer"):
            print("Failed to connect to the gazeboInterface")
            exit(0)

        print("Connection to the gazeboInterface has been established")
        # wait a bit for the interface
        yarp.Time_delay(1.0)

def dcm2axis(R):
    v = np.zeros(4)
    v[0] = R[2, 1] - R[1, 2]
    v[1] = R[0, 2] - R[2, 0]
    v[2] = R[1, 0] - R[0, 1]
    v[3] = 0.0
    r = np.linalg.norm(v)
    theta = math.atan2(0.5 * r, 0.5 * (R[0, 0] + R[1, 1] + R[2, 2] - 1))
    if (r < 1e-9):
        A = R[:3, :3]
        U, S, V = np.linalg.svd(A - np.eye(3, 3))
        v[0] = V[0, 2]
        v[1] = V[1, 2]
        v[2] = V[2, 2]
        r = np.linalg.norm(v)
    v = (1.0 / r) * v
    v[3] = theta
    V = yarp.Vector(4)
    V.set(0, v[0])
    V.set(1, v[1])
    V.set(2, v[2])
    V.set(3, v[3])
    return V

def getCol(matrix,col_num):
    V = yarp.Vector(3)
    for i in range(3):
        V.set(i, matrix.get(i,col_num))
    return V



def printYarpVector(v):
    len = v.length()
    list = []
    for i in range(len):
        list.append(v.get(i))
    print('printYarpVector:',list)
    return