# This file is created by Peixin Chang on 6/25/2018
"""
RL algorithm (Python)<--->iCub environment(Python)<--->GazeboClient(Python)<--->GazeboInterface (C++)<--->Gazebo robot
                                        YARP                                         YARP, Gazebo          Gazebo, YARP
GazeboClient is a client for sending request/query to GazeboInterface.
e.g. What is the position of an object? Or, insert an object at a certain x,y,z
It contains a lot of YARP client RPC ports
GazeboInterface receives the request/query from the GazeboClient, uses ROS wrapper to communicate with Gazebo simulator
and sends back the information. It contains a lot of YARP server RPC ports
"""


from libs.yarp_icub import yarp
import os
import subprocess
import xml.etree.ElementTree as ET
import random
import time

class GazeboClient(object):
    def __init__(self,**kwargs):
        self.path2tables=kwargs['path2tables']
        self.path2items=kwargs['path2items']
        self.path2world=kwargs['path2world']
        self.path2image = kwargs['path2image']
        self.tablex= kwargs['tablex']
        self.tabley = kwargs['tabley']
        self.tablez = kwargs['tablez']
        self.diff_height=kwargs['diff_height']
        self.height_range=kwargs['height_range']
        self.render=kwargs['render']
        self.activationPart=kwargs['activationPart']
        self.robot=kwargs['robot']

        # yarp ports
        self.modelTaskClient = yarp.RpcClient()
        self.simulationControlClient = yarp.RpcClient()

        self.table_length = None
        self.table_width = None  ##or vice versa???

    def simulator_on(self):
        # initialize YARP network
        yarp.Network.init()
        while not yarp.Network.checkNetwork():
            time.sleep(0.5)
            print("You should launch YARP server!")

        # Launch the simulation with the given launchfile name
        if not os.path.exists(self.path2world):
            raise IOError("File " + self.path2world + " does not exist")

        # write to environment for model loading
        if "GAZEBO_MODEL_PATH" not in os.environ:
            os.environ['GAZEBO_MODEL_PATH'] = "env/Gazebo/GazeboModel/iCub"

        if self.render:
            subprocess.Popen(["gazebo", self.path2world])
            print("Starting Gazebo")
        else:
            subprocess.Popen(["gzserver", self.path2world])
            print("Starting Only gzserver")

        time.sleep(5) #wait for the simulator

        #launch the gazeboInterface
        subprocess.Popen("./gazeboInterface", cwd="env/Gazebo/gazeboInterface")

        while not yarp.Network.exists("/gazeboInterface/modelTaskServer") or \
                not yarp.Network.exists("/gazeboInterface/simulationControlServer"):
            time.sleep(0.5)
        print("gazeboInterface is launched")





    @staticmethod
    def simulator_off():
        # Kill gzclient, gzserver but keep YARP network
        print("Turning off the simulator~")
        processes = os.popen("ps -Af").read()
        gzclient_count = processes.count('gzclient')
        gzserver_count = processes.count('gzserver')
        gazeboInterface_count=processes.count('gazeboInterface')
        yarprobotinterface_count=processes.count('yarprobotinterface')
        iKinCartesianSolver_count = processes.count('iKinCartesianSolver')

        if gzclient_count > 0:
            os.system("killall -9 gzclient")
        if gzserver_count > 0:
            os.system("killall -9 gzserver")

        if gazeboInterface_count > 0:
            os.system("killall -9 gazeboInterface")

        if yarprobotinterface_count > 0:
            os.system("killall -9 yarprobotinterface")

        if iKinCartesianSolver_count > 0:
            os.system("killall -9 iKinCartesianSolver")



        if gzclient_count >0 or gzserver_count > 0 or gazeboInterface_count > 0 or yarprobotinterface_count > 0\
                or iKinCartesianSolver_count>0:
            os.wait()

    @staticmethod
    def environment_setting():
        pass
        #os.system("gz physics -u 2000")

    def environment_creator(self):

        num_table=len([f for f in os.listdir(self.path2tables)])

        # randomly decide which table should be inserted
        index=random.randint(1,num_table)
        # create a string that lead to the model folder
        tableString=self.path2tables+"Plane"+str(index)+"/model.sdf"
        # read table information

        tree=ET.parse(tableString)
        root=tree.getroot()

        size=root[0][1][2][0][0][0].text
        size=[float(s) for s in size.split()]
        self.table_length=size[0]
        self.table_width=size[1] ##or vice versa???

        materials=root[0][1][2][1][0][1].text
        print("The location of the table is:",self.tablex, self.tabley, self.tablez)

        if self.diff_height: # we randomly vary the height of the table based on tablez
            self.tablez=self.tablez+random.uniform(-self.height_range,self.height_range)

        # insert the table into the world. x,y,z is the geometric center of the table
        name_of_model=root[0].attrib['name']
        self.modelTask("f", name_of_model, tuple([self.tablex, self.tabley, self.tablez, 0.0,0.0,0.0]), tableString)


        #insert the first objects the object should be on the table
        objectx, objecty, objectz=self.blockRandomizer()

        name_of_model="cube"
        path2model=self.path2items+"cube.sdf"

        self.modelTask("f", name_of_model, tuple([objectx, objecty, objectz, 0.0, 0.0, 0.0]), path2model)

        #if block disappears, wait
        time.sleep(1)

        # # insert the second objects
        name_of_model = "image0"
        path2image = self.path2image+"model.sdf"

        objectx1, objecty1, objectz1 = [3, 0, 1.5]
        self.modelTask("f", name_of_model, tuple([objectx1, objecty1, objectz1, 0.0, 0.0, 0.0]), path2image)

        time.sleep(3)

        return tuple([objectx,objecty,objectz])

    def blockRandomizer(self):

        objectx = self.tablex #+ random.uniform(-self.table_length * 0.5 + 0.03, 0)  #self.table_length * 0.5 - 0.1)
        objecty = self.tabley/2-0.1 #+ random.uniform(-self.table_width * 0.5 + 0.01, self.table_width * 0.5 -0.02)   #self.table_width * 0.5 - 0.1)

        #change to 0
        objectz = self.tablez + 0.06        #so that it will fall vertically to the table

        return objectx, objecty, objectz


    def modelTask(self, options, modelName="icub", xyzrpy=tuple([0.5, 0.5, 0.5, 0.0, 0.0, 0.0]), fileName="/"):

        if options=="f" and fileName=="/": print("Use default fileName")
        if options == "d" or options=="p" and modelName=="icub": print("Use default modelName")


        # Prepare command
        Out = yarp.Bottle()
        In = yarp.Bottle()
        Out.clear()
        In.clear()
        Out.addString(options)
        Out.addString(modelName)
        Out.addDouble(xyzrpy[0])
        Out.addDouble(xyzrpy[1])
        Out.addDouble(xyzrpy[2])
        Out.addDouble(xyzrpy[3])
        Out.addDouble(xyzrpy[4])
        Out.addDouble(xyzrpy[5])
        Out.addString(fileName)

        # Send command using Out and receive respond using In
        self.modelTaskClient.write(Out,In)
        if options=="p":
            return tuple([(In.get(0)).asDouble(), (In.get(1)).asDouble(), (In.get(2)).asDouble()])
        else:
            if (In.get(0)).asInt()==0:
                print("Request is not fulfilled for", options)


    def simulationControl(self, options, args=-1):
        '''
        This function will call gazeboInterface for controlling simulation process
        Usage:

    	"out" is a bottle indicating the request
    	"in" is a bool indicating if the request is fullfilled
    	-p, --pause=arg
              Pause/unpause simulation. 0=unpause, 1=pause.
        		e.g. the bottle[0]="p", bottle[1]=1 will pause the simulation

       -m, --multi-step=arg
              Step simulation mulitple iteration. arg=the number of steps you want an int

       -r, --reset-all
              Reset time and icub poses. All other things will not be affected



        :return:
        '''
        Out = yarp.Bottle()
        In = yarp.Bottle()
        Out.clear()
        In.clear()
        Out.addString(options)
        Out.addInt(args)
        self.simulationControlClient.write(Out,In)
        if (In.get(0)).asInt()==0:
            print("Request is not fulfilled for", options)

    def clean_up(self):


        # closing the drivers and ports
        if self.robot.activationPart[0] and self.robot.left_arm_driver is not None:
            self.robot.left_arm_driver.close()
        if self.robot.activationPart[1] and self.robot.right_arm_clientCartCtrl is not None:
            self.robot.right_arm_clientCartCtrl.close()
        if self.robot.activationPart[2] and self.robot.torso_driver is not None:
            self.robot.torso_driver.close()
        if self.robot.activationPart[3] and self.robot.head_driver is not None:
            self.robot.head_driver.close()
        if self.robot.activationPart[4] and self.robot.LCameraPort is not None:
            self.robot.LCameraPort.close()
        if self.robot.activationPart[5] and self.robot.RCameraPort is not None:
            self.robot.RCameraPort.close()

        time.sleep(0.5)
        print("YARP drivers and ports closed")

        self.simulator_off()





