#!/usr/bin/env python

import rospy
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

from flock import *

SCALE = 100

class Controler(object):
    def __init__(self):        
        self.flock = Flock()

    def update(self):  
	flocknode = self.flock.node
	
	leader = self.flock.getLeader()
        length =  len(flocknode)
        for i in range(length):
            fn = flocknode[i]
            if fn['role'] != 'leader':
	      xv = [fn['agent'].pose.position.x*SCALE, fn['agent'].pose.position.y*SCALE]
	      xf = fn['xf']
	      dis = np.sqrt( (xv[0] - xf[0])**2 + (xv[1] - xf[1])**2) / SCALE
	      
	      if dis < 0.3: # check if the robot is in the desired formation pose
		print "in formation pose"
		
		r, p, leader_yaw = getRPY(leader.pose.orientation)
		r, p, yaw = getRPY(fn['agent'].pose.orientation)  
		
		ang = leader_yaw - yaw	
		ang = angles.normalize(ang,-np.pi,np.pi)
		fn['agent'].SendCmd(0,ang/np.pi)
		
	      else:
		if fn['dp_r'][0] == 0 and fn['dp_r'][1] == 0:
		  #np_x = (0.1*fn['dp_g'][0] / SCALE) + (0.1*fn['dp_f'][0]/ SCALE) + (1*fn['dp_a'][0] / SCALE)
		  #np_y = (0.1*fn['dp_g'][1] / SCALE) + (0.1*fn['dp_f'][1]/ SCALE) + (1*fn['dp_a'][1] / SCALE)
		  np_x = (0.1*fn['dp_f'][0]/ SCALE) + (1*fn['dp_a'][0] / SCALE)
		  np_y = (0.1*fn['dp_f'][1]/ SCALE) + (1*fn['dp_a'][1] / SCALE)
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
		if vel > 0.7:
		  
		  vel = 0.7
	      
		fn['agent'].SendCmd(vel,ang/np.pi)
	      #end if
	      
            self.flock.update()


    def step(self):
        self.update()

  
#------------------------------------------------------------

if __name__ == '__main__':
    
    try:
	rospy.init_node('formation', anonymous=True)

	#wait for the node to be initialized
	time.sleep (1)
	cont = Controler()
    	rate = rospy.Rate(50) # 50hz
	while not rospy.is_shutdown():
	    cont.step()
	    rate.sleep()

    except rospy.ROSInterruptException: pass

