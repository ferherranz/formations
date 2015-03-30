#!/usr/bin/env python

import rospy
from matplotlib import pyplot as plt
from matplotlib import animation as animation
import numpy as np
import math
from std_msgs.msg import String
from geometry_msgs.msg import *
import time
import tf
import angles
SCALE = 100

def cart2polar(x, y):
  r = np.sqrt(x**2 + y**2)
  theta = np.arctan2(y, x)
  return r, theta

def getRPY(orientation):        
        (r, p, y) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
        return r, p, y
#Flock
class Flock(object):
    def __init__(self):
        self.node = []

        #self.node.append({'id':0, 'agent':Agent('robot_0'), 'p':[300,300], 'xi':[300,300], 'neighbor':[1,3]})
        #self.node.append({'id':1, 'agent':Agent('robot_1'), 'p':[300,100], 'xi':[300,100], 'neighbor':[0,2]})
        #self.node.append({'id':2, 'agent':Agent('robot_2'), 'p':[100,300], 'xi':[100,300], 'neighbor':[1,3]})
        #self.node.append({'id':3, 'agent':Agent('robot_3'), 'p':[100,100], 'xi':[100,100], 'neighbor':[0,2]})

	self.node.append({'id':0, 'agent':Agent('robot_0'), 'p':[300,300], 'xi':[300,300], 'neighbor':[1,2,3]})
        self.node.append({'id':1, 'agent':Agent('robot_1'), 'p':[300,100], 'xi':[300,100], 'neighbor':[0,2,3]})
        self.node.append({'id':2, 'agent':Agent('robot_2'), 'p':[100,100], 'xi':[100,100], 'neighbor':[0,1,3]})
        self.node.append({'id':3, 'agent':Agent('robot_3'), 'p':[100,300], 'xi':[100,300], 'neighbor':[0,1,2]})
        
 
	

                
        
class Agent(object):
    def __init__(self,name):
        
        self.name = name
        self.pose= Pose()
        
        rospy.Subscriber("/" + self.name + "/robot_pose", Pose, self.PoseCallback)        
        
    def PoseCallback(self, pose):      
      self.pose = pose
      #print self.name + ": " + str(self.pose.position.x) + " " + str(self.pose.position.y)
     
      
    def SendCmd(self,v_x,v_theta):
	pub = rospy.Publisher("/" + self.name + "/cmd_vel", Twist)
	# publish the defined linear and angular velocity
	pub.publish(Twist(Vector3(v_x, 0, 0), Vector3(0, 0, v_theta)))
	
        
        
#Goal
class Goal(object):
    def __init__(self):
        self.node = [600,600]


class Obstacle(object):
    def __init__(self):
        self.node = []

#        self.node.append([600,600])

##        self.node.append([-50,100])
##        self.node.append([10,10])
##        self.node.append([100,10])
##        self.node.append([50,50])
##        self.node.append([-10,10])
##        self.node.append([-30,100])


#Flock
class World(object):
    def __init__(self, flock, goal, obstacle):
        self.flocknode = flock.node
        #self.my_agents = agent.agents
        self.goalnode = goal.node
        self.obstaclenode = obstacle.node
        self.update()

    def formation(self):
        for x_i in self.flocknode:
            dp_x = 0
            dp_y = 0
            for j in x_i['neighbor']:
                x_j = self.flocknode[j]
                #dp_x += (x_i['p'][0] - x_j['p'][0]) - (x_i['xi'][0] - x_j['xi'][0])
                #dp_y += (x_i['p'][1] - x_j['p'][1]) - (x_i['xi'][1] - x_j['xi'][1])
                
                
                xv_i = [x_i['agent'].pose.position.x*SCALE, x_i['agent'].pose.position.y*SCALE]
                xv_j = [x_j['agent'].pose.position.x*SCALE, x_j['agent'].pose.position.y*SCALE]
                
                dist = np.sqrt(pow(xv_i[0] - xv_j[0],2) + pow(xv_i[1] - xv_j[1],2))
                
                #dp_x += ((x_i['p'][0] - x_j['p'][0]) - np.sqrt(pow(xv_i[0] - xv_j[0],2)))
                #dp_y += ((x_i['p'][1] - x_j['p'][1]) - np.sqrt(pow(xv_i[1] - xv_j[1],2)))
                
                dp_x += ((x_i['p'][0] - x_j['p'][0]) - (xv_i[0] - xv_j[0]))
                dp_y += ((x_i['p'][1] - x_j['p'][1]) - (xv_i[1] - xv_j[1]))
                
            #print [x_i['id'], xv_i, x_i['p'], x_j['id'], xv_j, x_j['p'],dp_x, dp_y]
                
            x_i['dp_f'] = [dp_x, dp_y]
	    
    def destination(self):
        for x_i in self.flocknode:
            gl = self.goalnode
            xv = [x_i['agent'].pose.position.x*SCALE, x_i['agent'].pose.position.y*SCALE]
            
            dist = np.sqrt(pow(gl[0] - xv[0],2) + pow(gl[1] - xv[1],2))
            
            dp_x = (gl[0] - xv[0])
            dp_y = (gl[1] - xv[1])
            x_i['dp_g'] = [dp_x, dp_y]
            #print x_i['dp_g']
           
           
##    def avoidance(self)
##        dlt = 10
##        amp = 1
##        for x_i in self.flocknode:
##            dp_x = 0
##            dp_y = 0
##            for ob in self.obstaclenode:
##                l_x = x_i['p'][0] - ob[0]
##                l_y = x_i['p'][1] - ob[1]
##                gauss = amp * math.exp(- (l_x**2/(2 * dlt**2) + l_y**2/(2 * dlt**2)))
##                dp_x += l_x * gauss
##                dp_y += l_y * gauss
##            x_i['dp_a'] = [dp_x, dp_y]

    def avoidance(self):
        for x_i in self.flocknode:
            dp_x = 0
            dp_y = 0
            for ob in self.obstaclenode:
                dis_x = x_i['p'][0] - ob[0]
                dis_y = x_i['p'][1] - ob[1]
                dis = math.sqrt(dis_x**2 + dis_y**2)
                threshold = 30
                if dis < threshold:
                    dp_x += (x_i['p'][0] - ob[0]) - (threshold/dis)*dis_x
                    dp_y += (x_i['p'][1] - ob[1]) - (threshold/dis)*dis_y
            x_i['dp_a'] = [-dp_x, -dp_y]

    def update(self):
        self.formation()
        self.destination()
        self.avoidance()

#World
class Controlor(object):
    def __init__(self):
	self.start_time = rospy.Time.now() 
        self.time = self.start_time
        self.timestep = 0.01
        self.world = World(Flock(), Goal(), Obstacle())


    def update(self, oldflocknode, newflocknode):
        now = rospy.Time.now() 
        dt = float((now - self.time).to_sec())
        self.time = now
        length =  len(oldflocknode)
        for i in range(length):
            on = oldflocknode[i]
            #np_x = on['p'][0] + dt * (0.5*on['dp_f'][0] + 1.1*on['dp_g'][0] + 100*on['dp_a'][0])
            #np_y = on['p'][1] + dt * (0.5*on['dp_f'][1] + 1.1*on['dp_g'][1] + 100*on['dp_a'][1])
            np_x = (0.1*on['dp_g'][0] / SCALE) + 0.05*on['dp_f'][0]/ SCALE
            np_y = (0.1*on['dp_g'][1] / SCALE) + 0.05*on['dp_f'][1]/ SCALE
            
            #np_x = 0.5*on['dp_f'][0] / SCALE
            #np_y = 0.5*on['dp_f'][1] / SCALE
            
            r,p,yaw = getRPY(on['agent'].pose.orientation)
            
            mod = np.sqrt(pow(np_x, 2)+(pow(np_y, 2)))
	    ang = np.arctan2(np_y, np_x) - angles.normalize(yaw,0,2*np.pi)
            ang = angles.normalize(ang,-np.pi,np.pi)
            
            vel = np.sqrt(np_x*np_x + np_y*np_y)
            
            if vel > 0.2:
	      vel = 0.2
	      
            #if i == 0:
	    #  print on['dp_g'], on['dp_f'], vel, np.degrees(yaw)
            
            on['agent'].SendCmd(vel,ang/np.pi)
            self.world.update()


    def step(self):
        self.update(self.world.flocknode, self.world.flocknode)

    #Flock
    def flock_pos(self):
        pos_x = []
        pos_y = []

        for i in self.world.flocknode:            
            pos_x.append(i['agent'].pose.position.x*SCALE)
            pos_y.append(i['agent'].pose.position.y*SCALE)

            
	pos_x.append(self.world.flocknode[0]['agent'].pose.position.x*SCALE) 
	pos_y.append(self.world.flocknode[0]['agent'].pose.position.y*SCALE)


        return (pos_x, pos_y)

    
    def obstacle_pos(self):
        pos_x = []
        pos_y = []
        for i in self.world.obstaclenode:
            pos_x.append(i[0])
            pos_y.append(i[1])

        return (pos_x, pos_y)



#------------------------------------------------------------
rospy.init_node('formation', anonymous=True)

#wait for the node to be initialized
time.sleep (1)
cont = Controlor()


# set up figure and animation
fig = plt.figure()
ax = fig.add_subplot(111, aspect='equal', autoscale_on=False,
                     xlim=(-1, 1000), ylim=(-1, 1000))
ax.grid()

points, = ax.plot([], [], 'o-', lw=2)
obs, = ax.plot([], [], 'o', lw=2)
time_text = ax.text(0.02, 0.95, '', transform=ax.transAxes)

def init():
    """initialize animation"""
    points.set_data([], [])
    obs.set_data(*cont.obstacle_pos())
    time_text.set_text('')
    return points, obs, time_text

def animate(i):
    """perform animation step"""
    cont.step()
    points.set_data(*cont.flock_pos())
    time_text.set_text('time = %.1f' % float((cont.time-cont.start_time).to_sec()))
    #plt.savefig('fig/plot'+str(cont.time)+'.eps', format = 'eps')
    return points, time_text

# choose the interval based on dt and the time to animate one step
from time import time
t0 = time()
animate(0)
t1 = time()
interval = 5000 * 0.01 - (t1 - t0)

ani = animation.FuncAnimation(fig, animate, frames=24,
                              interval=interval, blit=True, init_func=init)



plt.show()

rospy.spin()