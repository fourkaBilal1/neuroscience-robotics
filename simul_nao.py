import vrep
import math
import time
from brian2 import *
import matplotlib.pyplot as plt


def to_rad(deg):
    return 2*math.pi*deg/360

def to_deg(rad):
    return rad*360/(2*math.pi)


# simulation config
ip = '127.0.0.1'
port = 19997
scene = './Nao.ttt'
position_init = [0,0,to_rad(0)]

joints_h={-1,-1,-1,-1}
joints_v={-1,-1,-1,-1}


# Simulation 
K=1
Af= 0.5
sigma_f = 10
sigma_s = 150
tau_m = 10*ms
tau_s = K*tau_m
i_inj = 1


stimilus = TimedArray([0,0,40],dt = 10*ms)

injected=[]
for i in range(0,100):
    injected.append(sin(10*Hz*stimilus(i*ms)*i*ms+3000))

plt.plot(injected)
plt.ylabel('i_inj')
plt.savefig('i_inj.png',bbox_inches="tight",dpi=300)
plt.show()

eqs = Equations('''
dv/dt = (-v + Af*tanh(sigma_f/Af*v)-q + i_inj) /tau_m : 1
dq/dt = (-q + sigma_s * v)/tau_s : 1
i_inj = 0.8*sin(30*Hz*stimilus(t)*t+3000) : 1
''')

G = NeuronGroup(10, eqs,method="rk4")
trace = StateMonitor(G, ('v','q','i_inj'), record=True)

G.v = 0.5*rand()
G.q = 0.5*rand()

run(200*ms, report='text')

plot(trace.t/ms, trace.q[0], label='v')
plot(trace.t/ms, trace.i_inj[0], label='i_inj')
title("sigma_s: %.2f / sigma_f : %.2f \n tau_m = %.1f ms / tau_s = %.1f ms" % (sigma_s,sigma_f,tau_m*1000,tau_s*1000))
xlabel('t (ms)')
ylabel('q')
savefig('q.png',bbox_inches="tight",dpi=300)
show()

plot(trace.t/ms, trace.v[0], label='v')
plot(trace.t/ms, trace.i_inj[0], label='i_inj')
title("sigma_s: %.2f / sigma_f : %.2f \n tau_m = %.1f ms / tau_s = %.1f ms" % (sigma_s,sigma_f,tau_m*1000,tau_s*1000))
xlabel('t (ms)')
ylabel('v')
savefig('v.png',bbox_inches="tight",dpi=300)
show()


print ('Program started')
vrep.simxFinish(-1) # just in case, close all opened connections
client_id=vrep.simxStart(ip,port,True,True,5000,5) # Connect to V-REP

if client_id!=-1:
    print ('Connected to remote API server on %s:%s' % (ip, port))
    print('Client Id : %d' %(client_id))
    res = vrep.simxLoadScene(client_id, scene, 1, vrep.simx_opmode_oneshot_wait)
    res, NAO = vrep.simxGetObjectHandle(client_id, 'NAO', vrep.simx_opmode_oneshot_wait)
    res, NAO2 = vrep.simxGetObjectHandle(client_id, 'NAO#0', vrep.simx_opmode_oneshot_wait)

    res, h1_motor = vrep.simxGetObjectHandle(client_id, 'LShoulderPitch3', vrep.simx_opmode_oneshot_wait)
    res, h2_motor = vrep.simxGetObjectHandle(client_id, 'LShoulderRoll3', vrep.simx_opmode_oneshot_wait)
    res, h3_motor = vrep.simxGetObjectHandle(client_id, 'LElbowYaw3', vrep.simx_opmode_oneshot_wait)
    res, h4_motor = vrep.simxGetObjectHandle(client_id, 'LElbowRoll3', vrep.simx_opmode_oneshot_wait)

    res, v1_motor = vrep.simxGetObjectHandle(client_id, 'RShoulderPitch3', vrep.simx_opmode_oneshot_wait)
    res, v2_motor = vrep.simxGetObjectHandle(client_id, 'RShoulderRoll3', vrep.simx_opmode_oneshot_wait)
    res, v3_motor = vrep.simxGetObjectHandle(client_id, 'RElbowYaw3', vrep.simx_opmode_oneshot_wait)
    res, v4_motor = vrep.simxGetObjectHandle(client_id, 'RElbowRoll3', vrep.simx_opmode_oneshot_wait)
    

    res, v1_motor2 = vrep.simxGetObjectHandle(client_id, 'RShoulderPitch3#0', vrep.simx_opmode_oneshot_wait)
    res, v2_motor2 = vrep.simxGetObjectHandle(client_id, 'RShoulderRoll3#0', vrep.simx_opmode_oneshot_wait)
    res, v3_motor2 = vrep.simxGetObjectHandle(client_id, 'RElbowYaw3#0', vrep.simx_opmode_oneshot_wait)
    res, v4_motor2 = vrep.simxGetObjectHandle(client_id, 'RElbowRoll3#0', vrep.simx_opmode_oneshot_wait)
    
    
    vrep.simxStartSimulation(client_id, vrep.simx_opmode_oneshot_wait)

    position = position_init 

    prev_error = 100000
    i=1.5446
    j=0
    a = (trace.i_inj[0]).tolist()
    b = (trace.v[0]).tolist()
    continue_running = True
    while(continue_running):
    #Ask for stop running
      #  input("Press enter  to stop the simulation")
        
        current_time = vrep.simxGetLastCmdTime(client_id)
        command_h = [1.5, 0, 0, 0]
        command_v = [-1.5,-1.5,0,0] #-2, 1.5, 3]
        vrep.simxSetJointTargetPosition(client_id, h1_motor, command_h[0], vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(client_id, h2_motor, command_h[1], vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(client_id, h3_motor, command_h[2], vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(client_id, h4_motor, command_h[3], vrep.simx_opmode_oneshot_wait)

        vrep.simxSetJointTargetPosition(client_id, v1_motor, command_v[0], vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(client_id, v2_motor, command_v[1], vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(client_id, v3_motor, command_v[2], vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(client_id, v4_motor, a[j]+0.8 , vrep.simx_opmode_oneshot_wait) 
        
        vrep.simxSetJointTargetPosition(client_id, v1_motor2, command_v[0], vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(client_id, v2_motor2, command_v[1], vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(client_id, v3_motor2, command_v[2], vrep.simx_opmode_oneshot_wait)
        vrep.simxSetJointTargetPosition(client_id, v4_motor2, b[j]+0.8 , vrep.simx_opmode_oneshot_wait) 
        #0.0349 to 1.5446
        print("j = %d \t\t %f \t\t %f "%(j,a[j]+0.8,b[j]+0.8))
        if i== 1.5446:
            i=0.0349
        elif i==0.0349:
            i=1.5446
        # wait for velocity stabilization
        time.sleep(0.100)
        j = j+4
        # calcul prochain input
        #res, tmp = vrep.simxGetObjectPosition(client_id, snake, -1, vrep.simx_opmode_oneshot_wait)
        #position[0] = tmp[0]
        #position[1] = tmp[1]

        #res, tmp = vrep.simxGetObjectOrientation(client_id, snake, -1, vrep.simx_opmode_oneshot_wait)
        #position[2] = tmp[2] # en radian


        delta_t = (vrep.simxGetLastCmdTime(client_id)-current_time)/1000
           
    
    #continue_running = False    
    # terminate
    vrep.simxStopSimulation(client_id, vrep.simx_opmode_oneshot_wait)
    vrep.simxFinish(client_id)

else:
    print('Unable to connect to %s:%s' % (ip, port))
