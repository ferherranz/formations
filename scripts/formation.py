#!/usr/bin/env python

import rospy
from matplotlib import pyplot as plt
from matplotlib import animation as animation
import numpy as np
import math
from std_msgs.msg import String
from geometry_msgs.msg import *
from geometry_msgs.msg import *
from visualization_msgs.msg import *
from std_msgs.msg import ColorRGBA
from nav_msgs.msg import *
import time
import tf
import angles


SCALE = 100

def cart2polar(x, y):
  r = np.sqrt(x**2 + y**2)
  theta = np.arctan2(y, x)
  return r, theta

def polar2cart(r, theta):
    x = r * np.cos(theta)
    y = r * np.sin(theta)
    return x, y

def getRPY(orientation):        
    (r, p, y) = tf.transformations.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    return r, p, y
  
def rotMatrix2D(yaw):
    yaw = angles.normalize(yaw,0,2*np.pi)
    R = np.array([[np.cos(yaw), -np.sin(yaw)], 
		 [np.sin(yaw),  np.cos(yaw)]])
    return R
	
	
#Flock
class Flock(object):
    def __init__(self):
        self.node = []
	self.global_costmap = OccupancyGrid()
	rospy.Subscriber("/map", OccupancyGrid, self.GlobalCostMapCallback)   
       
	#self.node.append({'id':0, 'agent':Agent('robot_0'), 'role': 'leader', 'p':[0,0], 'xi':[0,0], 'neighbor':[1,2,3]})
        #self.node.append({'id':1, 'agent':Agent('robot_1'), 'role': 'follow', 'p':[0,-200], 'xi':[0,0], 'neighbor':[0,2,3]})
        #self.node.append({'id':2, 'agent':Agent('robot_2'), 'role': 'follow', 'p':[-200,-200], 'xi':[0,0], 'neighbor':[0,1,3]})
        #self.node.append({'id':3, 'agent':Agent('robot_3'), 'role': 'follow', 'p':[-200,0], 'xi':[0,0], 'neighbor':[0,1,2]})
        
	self.node.append({'id':0, 'agent':Agent('robot_0'), 'role': 'leader', 'p':[0,0], 'xi':[0,0], 'neighbor':[1,2,3]})
        self.node.append({'id':1, 'agent':Agent('robot_1'), 'role': 'follow', 'p':[-200,100], 'xi':[0,0], 'neighbor':[0,2,3]})
        self.node.append({'id':2, 'agent':Agent('robot_2'), 'role': 'follow', 'p':[-200,-100], 'xi':[0,0], 'neighbor':[0,1,3]})
        self.node.append({'id':3, 'agent':Agent('robot_3'), 'role': 'follow', 'p':[-400,0], 'xi':[0,0], 'neighbor':[0,1,2]})
        
	
        
    def getLeader(self):
	for x_i in self.node:
            if x_i['role'] == 'leader':
            	return x_i['agent']
            
    
    def GlobalCostMapCallback(self, global_costmap):      
	self.global_costmap = global_costmap     
	
    def publishConnections(self):
	pub = rospy.Publisher("formation/connections", MarkerArray, queue_size=10)
	markerArray = MarkerArray()
	line_color = ColorRGBA(0,0,1,1)
	line_width = 0.01
	for agent in self.node:
	    for j in agent['neighbor']:
		neighbor = self.node[j]
		a_pos = agent['agent'].pose.position
		n_pos = neighbor['agent'].pose.position
		
		p1 = Point(a_pos.x, a_pos.y, a_pos.z)
		p2 = Point(n_pos.x, n_pos.y, n_pos.z)
		
		marker = Marker()
		marker.header.frame_id = '/map'
		marker.type = marker.LINE_LIST
		marker.action = marker.ADD
		marker.color = line_color
		marker.points = [p1, p2]
		marker.scale.x = line_width
		markerArray.markers.append(marker)
	    
	    marker = Marker()
	    marker.header.frame_id = "/map"
	    marker.type = marker.SPHERE
	    marker.action = marker.ADD
	    marker.scale.x = 0.2
	    marker.scale.y = 0.2
	    marker.scale.z = 0.2
	    marker.color.a = 1.0
	    marker.color.r = 0.0
	    marker.color.g = 1.0
	    marker.color.b = 0.0
	    marker.pose.orientation.w = 1.0
	    
	    marker.pose.position.x = (agent['xi'][0]/SCALE)
	    marker.pose.position.y = (agent['xi'][1]/SCALE)
	    marker.pose.position.z = 0
	    markerArray.markers.append(marker)
        id = 0
	for m in markerArray.markers:
	    m.lifetime = rospy.Duration(1.0)
	    m.id = id
	    id += 1     
	pub.publish(markerArray)      
        
	

class Agent(object):
    def __init__(self,name):        
        self.name = name
        self.pose = Pose()
        
        rospy.Subscriber("/" + self.name + "/robot_pose", Pose, self.PoseCallback)      
     
    def PoseCallback(self, pose):      
      self.pose = pose      
      
    def SendCmd(self,v_x,v_theta):
	pub = rospy.Publisher("/" + self.name + "/cmd_vel", Twist, queue_size=10)
	# publish the defined linear and angular velocity
	pub.publish(Twist(Vector3(v_x, 0, 0), Vector3(0, 0, v_theta)))
    
    def SendCmdTwist(self,twist_msg):
	pub = rospy.Publisher("/" + self.name + "/cmd_vel", Twist, queue_size=10)
	# publish the defined linear and angular velocity
	pub.publish(twist_msg)
        
        
#Goal
class Goal(object):
    def __init__(self):	
        self.pose = []        
        rospy.Subscriber("formation/goal", PoseStamped, self.GoalCallback)    
        
    def GoalCallback(self, goal):      
	self.pose = [goal.pose.position.x*SCALE, goal.pose.position.y*SCALE]

	
class Obstacle(object):
    def __init__(self):
        self.node = []

#Flock
class World(object):
    def __init__(self, flock, goal, obstacle):
	self.flock = flock
        self.flocknode = flock.node 
        self.goal = goal
        self.obstaclenode = obstacle.node
        self.update()

    def formation(self):
	
	leader = self.flock.getLeader()
	r, p, yaw = getRPY(leader.pose.orientation) 
	
	rotMatrix = rotMatrix2D(yaw)
	
	leader_pos = leader.pose.position 
        for x_i in self.flocknode:
            dp_x = 0
            dp_y = 0
            
            xv_i = [x_i['agent'].pose.position.x*SCALE, x_i['agent'].pose.position.y*SCALE]
            
	    #ref_x_i rotated and translated to leader coordinates frame
	    ref_x_i = rotMatrix.dot(np.array(x_i['p']))
	    ref_x_i = [ref_x_i[0] + leader_pos.x*SCALE, ref_x_i[1] + leader_pos.y*SCALE]
	    
	    x_i['xi'] = ref_x_i          
      
            for j in x_i['neighbor']:
                x_j = self.flocknode[j]                
                
                xv_j = [x_j['agent'].pose.position.x*SCALE, x_j['agent'].pose.position.y*SCALE]
                
                #ref_x_j rotated and transalted to leader coordinates frame
                ref_x_j = rotMatrix.dot(np.array(x_j['p']))
                ref_x_j = [ref_x_j[0] + leader_pos.x*SCALE, ref_x_j[1] + leader_pos.y*SCALE]
                
                dp_x += ((ref_x_i[0] - ref_x_j[0]) - (xv_i[0] - xv_j[0]))
                dp_y += ((ref_x_i[1] - ref_x_j[1]) - (xv_i[1] - xv_j[1]))
                
                
                
            x_i['dp_f'] = [dp_x, dp_y]


    def destination(self):

        gl = self.goal.pose

	for x_i in self.flocknode:
	    dp_x = 0
	    dp_y = 0

	    if gl: # there is goal	
	      xv = [x_i['agent'].pose.position.x*SCALE, x_i['agent'].pose.position.y*SCALE]
	      dist = np.sqrt(pow(gl[0] - xv[0],2) + pow(gl[1] - xv[1],2))
            
	      dp_x = (gl[0] - xv[0])
	      dp_y = (gl[1] - xv[1])
            
	    x_i['dp_g'] = [dp_x, dp_y]
	  #print x_i['dp_g']
           
    #def robotAvoidance(self):
	#list_avoidance = [];
	#safe_dist = 0.5 * SCALE
        #for x_i in self.flocknode:
            #dp_x = 0
            #dp_y = 0
            #for x_j in self.flocknode:
                #if x_i['id'] != x_j['id']:		    
		    #xv_i = [x_i['agent'].pose.position.x*SCALE, x_i['agent'].pose.position.y*SCALE]
		    #xv_j = [x_j['agent'].pose.position.x*SCALE, x_j['agent'].pose.position.y*SCALE]
		    
		    
		    #diff_x = xv_i[0] - xv_j[0] 
		    #diff_y = xv_i[1] - xv_j[1]
		    
		    #dist = np.sqrt(pow(diff_x,2) + pow(diff_y,2))
		    
		    #id_av1 = str(x_i['id']) + str(x_j['id'])
		    #id_av2 = str(x_j['id']) + str(x_i['id'])
		    #if not id_av1 in list_avoidance and not id_av2 in list_avoidance:
			#if dist < safe_dist:
			  #dp_x += safe_dist - diff_x 
			  #dp_y += safe_dist - diff_y
			  #list_avoidance.append(id_av1)
			  #print "WARNNING IF"
		    #else:
			#if dist < safe_dist:
			  #dp_x += diff_x - safe_dist 
			  #dp_y += diff_y - safe_dist			  
			  #print "WARNNING ELSE"
		      
		      ##print [x_i['id'],x_j['id'],  xv_i[0],  xv_i[1], xv_j[0],  xv_j[1], diff_x, diff_y, safe_dist]
            #x_i['dp_r'] = [-dp_x, -dp_y]
            
    def robotAvoidance(self):
	safe_dist = 0.75 * SCALE
        for x_i in self.flocknode:
            dp_x = 0
            dp_y = 0
            for x_j in self.flocknode:
                if x_i['id'] != x_j['id']:		    
		    xv_i = [x_i['agent'].pose.position.x*SCALE, x_i['agent'].pose.position.y*SCALE]
		    xv_j = [x_j['agent'].pose.position.x*SCALE, x_j['agent'].pose.position.y*SCALE]
		    
		    diff_x = xv_i[0] - xv_j[0] 
		    diff_y = xv_i[1] - xv_j[1]
		    
		    dist = np.sqrt(pow(diff_x,2) + pow(diff_y,2))

		    if dist > 0 and  dist < safe_dist:
		      dp_x += (xv_i[0] - xv_j[0]) #- (safe_dist/dist)*dist
		      dp_y += (xv_i[1] - xv_j[1]) #- (safe_dist/dist)*dist
		      
		      print "WARNNING: Robot Avoidance Safe distance"
		    
            x_i['dp_r'] = [dp_x, dp_y]
            
    def avoidance(self):
	dlt = 10
        amp = 1
        for x_i in self.flocknode:
            dp_x = 0
            dp_y = 0
            i = 0
            for iy in range(self.flock.global_costmap.info.height):
	      for ix in range(self.flock.global_costmap.info.width):		
		if self.flock.global_costmap.data[i] > 1:
		    ob = []
		    # generate an obstacle for any point in the occupancyGrid that has nonzero probability.
		    ob.append((ix*self.flock.global_costmap.info.resolution + self.flock.global_costmap.info.origin.position.x)*SCALE)
		    ob.append((iy*self.flock.global_costmap.info.resolution + self.flock.global_costmap.info.origin.position.y)*SCALE)
		    
		    xv_i = [x_i['agent'].pose.position.x*SCALE, x_i['agent'].pose.position.y*SCALE]
		    
		    dis_x = xv_i[0] - ob[0]
		    dis_y = xv_i[1] - ob[1]
		    dis = math.sqrt(dis_x**2 + dis_y**2)
		    threshold = 2*SCALE
		    
		    if dis > 0 and dis < threshold:		
			print "WARNING"
			dp_x += (xv_i[0] - ob[0]) - (threshold/dis)*dis_x
			dp_y += (xv_i[1] - ob[1]) - (threshold/dis)*dis_y
			#l_x = xv_i[0] - ob[0]
			#l_y = xv_i[1] - ob[1]
			#gauss = amp * math.exp(- (l_x**2/(2 * dlt**2) + l_y**2/(2 * dlt**2)))
			#dp_x += l_x * gauss
			#dp_y += l_y * gauss
		i += 1
	    x_i['dp_a'] = [-dp_x, -dp_y]
	    

    def update(self):
        self.formation()
        self.destination()
        self.avoidance()
        self.robotAvoidance()
        self.flock.publishConnections()
        
    

#World
class Controlor(object):
    def __init__(self):
        self.world = World(Flock(), Goal(), Obstacle())


    def update(self, flocknode):
        
        length =  len(flocknode)
        for i in range(length):
            fn = flocknode[i]
            if fn['role'] != 'leader':
	      
	      
	      if fn['dp_r'][0] == 0 and fn['dp_r'][1] == 0:
		np_x = (0.1*fn['dp_g'][0] / SCALE) + (0.1*fn['dp_f'][0]/ SCALE) + (1*fn['dp_a'][0] / SCALE)
		np_y = (0.1*fn['dp_g'][1] / SCALE) + (0.1*fn['dp_f'][1]/ SCALE) + (1*fn['dp_a'][1] / SCALE)
	      else:
		print "Avoiding"
		np_x = (1*fn['dp_a'][0] / SCALE) + (0.25*fn['dp_r'][0]/ SCALE)
		np_y = (1*fn['dp_a'][1] / SCALE) + (0.25*fn['dp_r'][1]/ SCALE)
	      r,p,yaw = getRPY(fn['agent'].pose.orientation)  
	      
	      mod = np.sqrt(pow(np_x, 2)+(pow(np_y, 2)))
	      ang = np.arctan2(np_y, np_x) - angles.normalize(yaw,0,2*np.pi)
	      ang = angles.normalize(ang,-np.pi,np.pi)
	      
	      vel = np.sqrt(np_x*np_x + np_y*np_y)
	      #print vel
	      if vel > 0.6:
		vel = 0.6
	     
	      fn['agent'].SendCmd(vel,ang/np.pi)
            
            self.world.update()


    def step(self):
        self.update(self.world.flocknode)

  
#------------------------------------------------------------

if __name__ == '__main__':
    
    try:
	rospy.init_node('formation', anonymous=True)

	#wait for the node to be initialized
	time.sleep (1)
	cont = Controlor()
    	rate = rospy.Rate(50) # 50hz
	while not rospy.is_shutdown():
	    cont.step()
	    rate.sleep()

    except rospy.ROSInterruptException: pass

