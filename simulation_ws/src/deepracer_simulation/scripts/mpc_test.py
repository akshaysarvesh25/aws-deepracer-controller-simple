#!/usr/bin/env python
import rospy
import time
import math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from std_msgs.msg import Float64
from ackermann_msgs.msg import AckermannDriveStamped
from gazebo_msgs.msg import ModelStates
from deepracer_msgs.msg import Progress
import PID_control
import tf
import pandas as pd 

flag_move = 0

#x_des = 1 
#y_des = 0 
x_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/output',AckermannDriveStamped,queue_size=1)
throttle = 0.0
heading = 0.0
pos=[0,0]
yaw=0.0
global count
count = 0

     
def set_position(data):
    #global x_des
    #global y_des
            
    racecar_pose = data.pose[1]
    pos[0] = racecar_pose.position.x
    pos[1] = racecar_pose.position.y
    quaternion = (
            data.pose[1].orientation.x,
            data.pose[1].orientation.y,
            data.pose[1].orientation.z,
            data.pose[1].orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    yaw = euler[2]
   
            
    err = math.sqrt((x_des-pos[0])**2+(y_des-pos[1])**2)
    #heading = math.atan((y_des-pos[1])/(x_des-pos[0]+0.00001)
    if not (err<0.5 or ((x_des-pos[0])<0.3) or ((y_des-pos[1])<0.3)):
        control_car(pos,yaw)
    else:
        print("Stopping car...")
        stop_car()
        sub.unregister()        
        servo_commands()
        

def control_car(pos,yaw):
    #print("Navigating to",x_des, y_des)
    
    msg = AckermannDriveStamped()
    #print("====position=====",pos[0],pos[1])
    speed_control = PID_control.PID(1e-6,0,1e-6)
    err = math.sqrt((x_des-pos[0])**2+(y_des-pos[1])**2)
    throttle = speed_control.Update(err)
    #print("distance:", err)
    #print("throttle:",throttle)
    
    
    steer_control = PID_control.PID(1e-8,0,1e-7)
    heading = math.atan((y_des-pos[1])/(x_des-pos[0]+0.01))
    steer = steer_control.Update(heading-yaw)
    #print("yaw:",yaw)
    #print("steer_angle:",heading-yaw)
    #print("steer:",steer)
    
   
    #print("========throttle signal=======",throttle)
    msg.drive.speed = throttle 
    x_pub.publish(msg)
    #time.sleep(1)
           
    msg.drive.steering_angle = steer
    x_pub.publish(msg)
    #time.sleep(1)
    #print("==============")

def stop_car():
    msg = AckermannDriveStamped()
    msg.drive.speed = 0
    msg.drive.steering_angle = 0
    msg.drive.steering_angle_velocity = 0
    x_pub.publish(msg)
    print("Goal Reached!") 

def servo_commands():
    print("Car is at :",pos[0],pos[1])
         
    msg = AckermannDriveStamped()

    df = pd.read_csv('mpc_control.txt', sep = ' ')

    steer = df.d
    throttle = df.v

    for i in range(df.shape[0]):
        print("throttle:",throttle[i], "steer:", steer[i])

        msg.drive.speed = throttle[i]
        x_pub.publish(msg)
        time.sleep(1)

        msg.drive.steering_angle = steer[i]
        x_pub.publish(msg)
        time.sleep(1)

    #rospy.Subscriber("/progress", Progress, set_throttle_steer)
    #sub = rospy.Subscriber("/gazebo/model_states", ModelStates, set_position)   

    while not (rospy.is_shutdown()):
        """
        print("====position=====",pos[0],pos[1])
        speed_control = PID_control.PID(0.000001,0,0.000001)
        err = math.sqrt((x_des-pos[0])**2+(y_des-pos[1])**2)
        throttle = speed_control.Update(err)
        print("distance:", err)
        print("throttle:",throttle)
        """       
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        rospy.init_node('servo_commands', anonymous=True)
               
        servo_commands()
    except rospy.ROSInterruptException:
        pass