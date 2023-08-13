#!/usr/bin/env python3

import rospy 
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
#from kobuki_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import matplotlib.pyplot as plt
import math
import numpy as np
import message_filters
start=np.array([[1.28856943],[0.74908222]])
goal=np.array([[1.5],[-4.5]])
laser_band=2

odom_data=[]
odom_data.append(start)
circum=0
ob_dist_error=5e-2
yaw_error=5*math.pi/180
ob_dist=0.25
arg=[]
flag_laser=0
goal_dir_found_flag=0
hit_vec=0
prev_error1=0
cum_error1=0
prev_heading=0
corner=0
theta1=0
hit_point=0
circum_list=[]
step_size=0.1
leave_point=0
waypoints=[start]
prev_circum=0
def algo(ranges,loc,yaw):
    
    global laser_band
    global ob_dist_error
    global ob_dist
    global circum
    global goal
    global  yaw_error
    global arg
    global prev_error1
    m=len(ranges)
    global corner,hit_point,circum_list,step_size,leave_point,waypoints,prev_circum,odom_data
    laser_readings={ "front":min(min(ranges[0:int(laser_band+3)+1]),min(ranges[m-int(laser_band+3):m])),
                     "left" :min(ranges[int(m/4)-int(laser_band/2):int(m/4)+int(laser_band/2)+1]),
                     "right":min(ranges[3*int(m/4)-int(laser_band/2):3*int(m/4)+int(laser_band/2)+1]),
                     "front_right" :min(ranges[int(m/(4*3))+3*int(m/4)-int(laser_band/2):int(m/(4*3))+3*int(m/4)+int(laser_band/2)+1])
                      }
    
    laser_readings_req=np.array(ranges[3*int(m/4):m]+ranges[0:int(m/4)]+ranges[int(m/4)+1:2*int(m/4)]+ranges[2*int(m/4)+1:3*int(m/4)])
    laser_readings_req[laser_readings_req==0]=math.inf
    
    #print(ranges[90], "    circum=",circum)
  
    if(np.linalg.norm(loc-odom_data[-1])>=step_size):
             odom_data.append(loc)   
    if(circum==0):
            if(np.linalg.norm(loc-waypoints[-1])>=step_size):
                         waypoints.append(loc)
            
            v=-loc+goal
            target=math.atan2(v[1,0],v[0,0])
            move_the_bot.angular.z=0.5*abs(-target+yaw)
            move_the_bot.linear.x=0.0
            
            if(abs(target-yaw)<=yaw_error):
                
                move_the_bot.angular.z=0.0
                move_the_bot.linear.x = 0.3
                if((np.min(laser_readings_req[int(m/4)-3:int(m/4)+3])<ob_dist)):
                        move_the_bot.linear.x = 0.0
                        move_the_bot.angular.z=0.0
                        
                        circum="yes"
                        hit_point=loc
                        circum_list.append(loc)
                        print("*************************************************")
                        print(hit_point)
            prev_circum=circum
                
                       
     
            
                                  
                
                
                
                
                
                                      
                      
                      
                                       
   
    
    elif (circum=="yes"):
         
         if(np.linalg.norm(circum_list[-1]-loc)>=step_size):
                         circum_list.append(loc)
         if(np.linalg.norm(circum_list[0]-loc)<(3*step_size) and len(circum_list)>20):
                   circum_list_np=np.array(circum_list)
                   temp=circum_list-goal
                   tt=(np.reshape(((temp)),(temp.shape[0],temp.shape[1])))
                   ttt=(np.linalg.norm(tt,axis=1))
                   ind=np.argmin(ttt)
                   leave_point=circum_list[ind]
                   circum="leave point"
                   print(odom_data)
                   print(leave_point)
                   print("***********************************************************")
                   corner=0
         if(corner==0):
            if(abs(np.min(np.concatenate((laser_readings_req[m-int(m/8):m],laser_readings_req[0:int(m/8)])))-ob_dist)>=ob_dist_error):
                           error1=-(np.min(np.concatenate((laser_readings_req[m-int(m/8):m],laser_readings_req[0:int(m/8)])))-ob_dist)
                           move_the_bot.linear.x=0.1
                           move_the_bot.angular.z=4*error1+(2*(error1-prev_error1))
                          
                           prev_error1=error1
            else:
                 move_the_bot.linear.x=0.1
                 move_the_bot.angular.z=0
                 
            if(np.max(laser_readings_req[int(m/4)-3:int(m/4)+3])<0.3 ):
                         corner="I"
            if(np.min(laser_readings_req[int(50/(360/m)):int(50/(360/m))+10])>1    and ( True or (np.max(laser_readings_req[int(m/4)-3:int(m/4)+3])>0.5))):
                    corner="II"
                             
         
         if(corner=="I"):
               
                if(np.max(laser_readings_req[int(m/4)-3:int(m/4)+3])<1):
                             move_the_bot.linear.x=0.0
                             move_the_bot.angular.z=0.7
                else:
                    corner=0
         if(corner=="II"):
                 
                 if(np.min(laser_readings_req[int(60/(360/m)):int(60/(360/m))+6])>0.7):
                             move_the_bot.linear.x=0.0
                             move_the_bot.angular.z=-0.7
                 else:
                     corner=0
         prev_circum=circum                                             
                        
    elif(circum=="leave point"):
         if(np.linalg.norm(loc-waypoints[-1])>=step_size):
                         waypoints.append(loc)
            
         if(np.linalg.norm(loc-leave_point)<step_size):
                    circum=0
                    circum_list=[]
    
         if(corner==0):
            if(abs(np.min(np.concatenate((laser_readings_req[m-int(m/8):m],laser_readings_req[0:int(m/8)])))-ob_dist)>=ob_dist_error):
                           error1=-(np.min(np.concatenate((laser_readings_req[m-int(m/8):m],laser_readings_req[0:int(m/8)])))-ob_dist)
                           move_the_bot.linear.x=0.1
                           move_the_bot.angular.z=4*error1+(2*(error1-prev_error1))
                          
                           prev_error1=error1
            else:
                 move_the_bot.linear.x=0.1
                 move_the_bot.angular.z=0
                 
            if(np.max(laser_readings_req[int(m/4)-3:int(m/4)+3])<0.3 ):
                         corner="I"
            if(np.min(laser_readings_req[int(50/(360/m)):int(50/(360/m))+10])>1    and (True or (np.max(laser_readings_req[85:95])>0.5))):
                    corner="II"
                             
         
         if(corner=="I"):
                
                if(np.max(laser_readings_req[int(m/4)-3:int(m/4)+3])<1):
                             move_the_bot.linear.x=0.0
                             move_the_bot.angular.z=0.7
                else:
                    corner=0
         if(corner=="II"):
                 
                 if(np.min(laser_readings_req[int(60/(360/m)):int(60/(360/m))+6])>0.7):
                             move_the_bot.linear.x=0.0
                             move_the_bot.angular.z=-0.7
                 else:
                      corner=0
         prev_circum=circum  
    elif(circum=="stuck"):
                 if(min(laser_readings_req[int(m/4)-3:int(m/4)+3])<0.5):
                         move_the_bot.linear.x=-0.1
                         move_the_bot.angular.z=-0.5
                 else: 
                     circum=prev_circum
                     
                                         
                                      
    if(np.linalg.norm(loc-goal)<step_size):
                  move_the_bot.linear.x=0
                  move_the_bot.angular.z=0     
                  print(odom_data)   
                  odom_data=np.array(odom_data)
                  odom_data=np.reshape(odom_data,(odom_data.shape[0],odom_data.shape[1]))
                  x_odom_data=odom_data[:,0]
                  y_odom_data=odom_data[:,1]
                  plt.plot(x_odom_data,y_odom_data)
                  plt.show()              
             
    if(min(laser_readings_req[int(m/4)-3:int(m/4)+3])<0.17):
            
            circum="stuck"
             
    print("circum=",circum)
    print("front=",laser_readings_req[int(m/4)-3:int(m/4)+3])
    #print("lp",leave_point)
    #print(laser_readings) 
    #print(loc)   
    publish_to_cmd_vel.publish(move_the_bot) 
    arg=[]
    
    

      

    
def laserdata_callback(msg):
      global flag_laser
      #print((msg.ranges[122]))
      #print(len(msg.ranges))
      if(len(msg.ranges)>=240):
         
        arg.append(msg.ranges)
       
        #print((msg.ranges[0]))
        flag_laser=1
       

    
def odom_callback(msg):
    
     global flag_laser
     global arg
     if(flag_laser==1):
        ori=msg.pose.pose.orientation
        pos=msg.pose.pose.position
        oril=[ori.x,ori.y,ori.z,ori.w]
        yaw=euler_from_quaternion(oril)[2]
        loc=np.array([[pos.x],[pos.y]])
        
        
        arg.append(loc)
        arg.append(yaw)
       
        flag_laser=0
        #print(arg[0])
       
        
        algo(arg[0],arg[1],arg[2])
        #move_the_bot.linear.x=0.05
        #publish_to_cmd_vel.publish(move_the_bot) 
        #print("x")
       
        



if __name__ == "__main__":
    
    rospy.init_node('turtlebot_controller_node')

    r=rospy.Rate(10)
    subscribe_to_laser = rospy.Subscriber('/scan', LaserScan, callback = laserdata_callback)
    subscribe_to_odometry=rospy.Subscriber('/odom', Odometry, callback = odom_callback)
    
    rospy.loginfo('My node has been started')
    publish_to_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size = 10)
    
  
    #create an object of Twist data

    move_the_bot = Twist()
    r.sleep()
    rospy.spin()


