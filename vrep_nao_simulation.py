import vrep
import math
import numpy as np

def to_rad(deg):
    return 2*math.pi*deg/360

def to_deg(rad):
    return rad*360/(2*math.pi)

class VrepPioneerSimulation:
    def __init__(self):

        self.ip = '127.0.0.1'
        self.port = 19997
        self.scene = './Nao.ttt'
        self.gain = 2
        self.initial_position = [3,3,to_rad(45)]

        self.r = 0.096 # wheel radius
        self.R = 0.267 # demi-distance entre les r

        print('New pioneer simulation started')
        vrep.simxFinish(-1)
        self.client_id = vrep.simxStart(self.ip, self.port, True, True, 5000, 5)

        if self.client_id!=-1:
            print ('Connected to remote API server on %s:%s' % (self.ip, self.port))
            res = vrep.simxLoadScene(self.client_id, self.scene, 1, vrep.simx_opmode_oneshot_wait)
            res, self.pioneer = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx', vrep.simx_opmode_oneshot_wait)
            res, self.left_motor = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_leftMotor', vrep.simx_opmode_oneshot_wait)
            res, self.right_motor = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_rightMotor', vrep.simx_opmode_oneshot_wait)
            self.sensors = []
            for i in range(16):
                res, tmp = vrep.simxGetObjectHandle(self.client_id, 'Pioneer_p3dx_ultrasonicSensor'+str(i+1), vrep.simx_opmode_oneshot_wait)
                self.sensors.append(tmp)
            self.set_position(self.initial_position)
            vrep.simxStartSimulation(self.client_id, vrep.simx_opmode_oneshot_wait)

        else:
            print('Unable to connect to %s:%s' % (self.ip, self.port))

    def set_position(self, position):
        """Set the position (x,y,theta) of the robot

        Args:
            position (list): the position [x,y,theta]
        """

        vrep.simxSetObjectPosition(self.client_id, self.pioneer, -1, [position[0], position[1], 0.13879], vrep.simx_opmode_oneshot_wait)
        vrep.simxSetObjectOrientation(self.client_id, self.pioneer, -1, [0, 0, to_deg(position[2])], vrep.simx_opmode_oneshot_wait)

    def get_position(self):
        """Get the position (x,y,theta) of the robot

        Return:
            position (list): the position [x,y,theta]
        """
        position = []
        res, tmp = vrep.simxGetObjectPosition(self.client_id, self.pioneer, -1, vrep.simx_opmode_oneshot_wait)
        position.append(tmp[0])
        position.append(tmp[1])

        res, tmp = vrep.simxGetObjectOrientation(self.client_id, self.pioneer, -1, vrep.simx_opmode_oneshot_wait)
        position.append(tmp[2]) # en radian

        return position
    
    def get_distances(self):
        distances = []
        for i in range(16):
            errorCode, detectionState, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector = vrep.simxReadProximitySensor(self.client_id, self.sensors[i], vrep.simx_opmode_oneshot_wait)
#            print(detectedPoint)
            distances.append(np.linalg.norm(detectionState))
        return distances

    def set_motor_velocity(self, control):
        """Set a target velocity on the pioneer motors, multiplied by the gain
        defined in self.gain

        Args:
            control(list): the control [left_motor, right_motor]
        """
        vrep.simxSetJointTargetVelocity(self.client_id, self.left_motor, self.gain*control[0], vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetVelocity(self.client_id, self.right_motor, self.gain*control[1], vrep.simx_opmode_oneshot_wait)

    def get_motor_velocity(self):
        """Get the velocities (q1, q2) of the left and right wheels of the robot
        
        Return:
            command (list): [q1, q2]
        """
        command=[]
        
        res, linear_vel, angular_vel = vrep.simxGetObjectVelocity(self.client_id, self.left_motor, vrep.simx_opmode_oneshot_wait)
        command.append(angular_vel[1]) #We append gamma that is the angular velocity of the wheel
        
        res, linear_vel, angular_vel = vrep.simxGetObjectVelocity(self.client_id, self.right_motor, vrep.simx_opmode_oneshot_wait)
        command.append(angular_vel[1]) #We append gamma that is the angular velocity of the wheel
        
        return command
    
    def get_linear_velocity(self):
        res, linear_vel, angular_vel = vrep.simxGetObjectVelocity(self.client_id, self.pioneer, vrep.simx_opmode_oneshot_wait)
        return linear_vel