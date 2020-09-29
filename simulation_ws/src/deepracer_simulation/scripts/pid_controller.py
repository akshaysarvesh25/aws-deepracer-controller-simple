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

flag_move = 0
x_des = 10
y_des = 10
x_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/output',AckermannDriveStamped,queue_size=1)
throttle = 0.0
heading = 0.0
pos=[0,0]
yaw=0.0
q=[0,0,0,0]
def set_position(data):
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

    print(yaw)
    """
    err = math.sqrt((x_des-pos[0])**2+(y_des-pos[1])**2)
    #heading = math.atan((y_des-pos[1])/(x_des-pos[0]+0.00001))
    if not (err<0.5):
        control_car(pos,yaw)
    """


def set_throttle_steer(data):
    #print(data.pose[1].orientation.y)
    #racecar_pose = data.pose[1]
    #print("callback", data.yaw)
    #pos[0] = data.x
    #pos[1] = data.y
    #print(data)
    #yaw = data.yaw
    #print(yaw)
    """

    err = math.sqrt((x_des-pos[0])**2+(y_des-pos[1])**2)
    #heading = math.atan((y_des-pos[1])/(x_des-pos[0]+0.00001))
    if not (err<0.5):
        control_car(pos,yaw)
    """


def control_car(pos,yaw):
    
    msg = AckermannDriveStamped()
    print("====position=====",pos[0],pos[1])
    speed_control = PID_control.PID(0.000001,0,0.000001)
    err = math.sqrt((x_des-pos[0])**2+(y_des-pos[1])**2)
    throttle = speed_control.Update(err)
    #print("distance:", err)
    #print("throttle:",throttle)
    print("yaw:",yaw)
    
    steer_control = PID_control.PID(0.000001,0,0.0000001)
    heading = math.atan((y_des-pos[1])/(x_des-pos[0]+0.01))
    steer = steer_control.Update(heading-yaw)
    #print("yaw:",yaw)
    #print("heading:",heading)
    #print("steer_angle:",heading-yaw)
    #print("steer:",steer)
    
   
    #print("========throttle signal=======",throttle)
    msg.drive.speed = throttle 
    x_pub.publish(msg)
    #time.sleep(1)
           
    msg.drive.steering_angle = steer
    x_pub.publish(msg)
    #time.sleep(1)
    print("==============")


def servo_commands():

    #time.sleep(2)
    rospy.init_node('servo_commands', anonymous=True)
    
    msg = AckermannDriveStamped()
    #rospy.Subscriber("/progress", Progress, set_throttle_steer)
    rospy.Subscriber("/gazebo/model_states", ModelStates, set_position)
    rospy.Subscriber("/progress", Progress, set_throttle_steer)

     
    #err = math.sqrt((x_des-pos[0])**2+(y_des-pos[1])**2)
    #heading = math.atan((y_des-pos[1])/(x_des-pos[0]+0.00001))
    
    #print("read yaw2",yaw)
    while not (rospy.is_shutdown()):
        """    
        print("====position=====",pos[0],pos[1])
        speed_control = PID_control.PID(0.000001,0,0.000001)
        err = math.sqrt((x_des-pos[0])**2+(y_des-pos[1])**2)
        throttle = speed_control.Update(err)
        print("distance:", err)
        print("throttle:",throttle)
        
        steer_control = PID_control.PID(0.0001,0,0.000001)
        heading = math.atan((y_des-pos[1])/(x_des-pos[0]+0.01))
        steer = steer_control.Update(heading-yaw)
        print("yaw:",yaw)
        print("heading:",heading)
        print("steer_angle:",heading-yaw)
        print("steer:",steer)
        
       
        #print("========throttle signal=======",throttle)
        msg.drive.speed = throttle 
        x_pub.publish(msg)
        #time.sleep(1)
               
        msg.drive.steering_angle = steer
        x_pub.publish(msg)
        #time.sleep(1)
        print("==============")
        """

        
    msg.drive.speed = 0.0
    x_pub.publish(msg)
    time.sleep(1)

    
    """

    while not rospy.is_shutdown:
        msg.drive.speed = 1.0
    	x_pub.publish(msg)
	time.sleep(1)
    """

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    try:
        servo_commands()
    except rospy.ROSInterruptException:
        pass
