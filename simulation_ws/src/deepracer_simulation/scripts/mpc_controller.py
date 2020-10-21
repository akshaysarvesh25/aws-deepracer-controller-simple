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

import matplotlib.pyplot as plt
import cvxpy
import math
import numpy as np
import sys
import heapq
from matplotlib.pyplot import figure
from scipy.signal import savgol_filter
import pandas as pd


#sys.path.append("../../PathPlanning/CubicSpline/")

try:
    import cubic_spline_planner
except:
    raise

#############A-star#############################
def heuristic(a, b):    
    return np.sqrt((b[0] - a[0]) ** 2 + (b[1] - a[1]) ** 2)
 
def n_score(grid,a):
    score = 0    
    for i in range(-6,6):
        for j in range(-6,6):
            if i!=0 and j!=0:
                if a[0]+i>=0 and a[0]+i<grid.shape[0] and a[1]+j>=0 and a[1]+j<grid.shape[1]:
                    score = score+(10/abs(i)+10/abs(j))*grid[a[0]+i,a[1]+j]
    return score

def astar(array, start, goal):

    #neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
    #neighbors = [(0,1),(0,-1),(1,0),(-1,0)]
    close_set = set()
    came_from = {}
    gscore = {start:n_score(array,start)}
    #gscore_1 = {start:0}
    fscore = {start:heuristic(start, goal)}
    oheap = []
    heapq.heappush(oheap, (fscore[start], start))
    while oheap:
        current = heapq.heappop(oheap)[1]
        if current == goal:
            data = []
            while current in came_from:
                data.append(current)
                current = came_from[current]
            return data
        close_set.add(current)
        for i, j in neighbors:
            neighbor = current[0] + i, current[1] + j
            #tentative_g_score = gscore[current] + heuristic(current, neighbor)
            tentative_g_score_1 = gscore[current] + heuristic(current, neighbor) 
            tentative_g_score =   tentative_g_score_1 + n_score(array, neighbor)
            #tentative_g_score = gscore[current] + heuristic(current, neighbor)
            #print(neighbor, gscore_1[current],gscore[current],n_score(grid, neighbor))
            
            if 0 <= neighbor[0] < array.shape[0]:
                if 0 <= neighbor[1] < array.shape[1]:
                    if array[neighbor[0]][neighbor[1]] == 1:
                        continue
                else:
                    continue
            else:
                continue 

            if neighbor in close_set and tentative_g_score >= gscore.get(neighbor, 0):
                continue
 

            if  tentative_g_score < gscore.get(neighbor, 0) or neighbor not in [i[1]for i in oheap]:
                came_from[neighbor] = current
                #gscore_1[neighbor] = tentative_g_score_1
                gscore[neighbor] = tentative_g_score_1
                fscore[neighbor] = tentative_g_score + heuristic(neighbor, goal)
                heapq.heappush(oheap, (fscore[neighbor], neighbor))
    return False

#########################A-star##############################################

def get_straight_course2(dl,y_coords,x_coords):
    ax = y_coords
    ay = x_coords
    cx, cy, cyaw, ck, s = cubic_spline_planner.calc_spline_course(
        ax, ay, ds=dl)

    return cx, cy, cyaw, ck

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.predelta = None

def pi_2_pi(angle):
    while(angle > math.pi):
        angle = angle - 2.0 * math.pi

    while(angle < -math.pi):
        angle = angle + 2.0 * math.pi

    return angle 

def calc_nearest_index(state, cx, cy, cyaw, pind):
    N_IND_SEARCH = 10

    dx = [state.x - icx for icx in cx[pind:(pind + N_IND_SEARCH)]]
    dy = [state.y - icy for icy in cy[pind:(pind + N_IND_SEARCH)]]

    d = [idx ** 2 + idy ** 2 for (idx, idy) in zip(dx, dy)]

    mind = min(d)

    ind = d.index(mind) + pind

    mind = math.sqrt(mind)

    dxl = cx[ind] - state.x
    dyl = cy[ind] - state.y

    angle = pi_2_pi(cyaw[ind] - math.atan2(dyl, dxl))
    if angle < 0:
        mind *= -1

    return ind, mind

def smooth_yaw(yaw):

    for i in range(len(yaw) - 1):
        dyaw = yaw[i + 1] - yaw[i]

        while dyaw >= math.pi / 2.0:
            yaw[i + 1] -= math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

        while dyaw <= -math.pi / 2.0:
            yaw[i + 1] += math.pi * 2.0
            dyaw = yaw[i + 1] - yaw[i]

    return yaw

############################Parameters###########################################

flag_move = 0
 
x_pub = rospy.Publisher('/vesc/low_level/ackermann_cmd_mux/output',AckermannDriveStamped,queue_size=1)
throttle = 0.0

dl = 1.0  # course tick
grid = np.zeros((80,80))
grid = np.pad(grid,(1,1),'constant',constant_values=1)
grid[1:25,41]=1
grid[37:69,41]=1
grid[40,1:17]=1
grid[40,29:41]=1
grid[40,53:81]=1
grid[17:25,17:25]=1
grid[9:17,53:69]=1
grid[57:65,21:33]=1
grid[53:61,49:57]=1
start = (5,5)
goal = (75,65)
route = astar(grid, start, goal)
route = route + [start]
route = route[::-1]
x_coords = []
y_coords = []
for i in (range(0,len(route))):
    x = route[i][0]
    y = route[i][1]
    x_coords.append(x)
    y_coords.append(y)

x_f = savgol_filter(x_coords, 31, 3)
y_f = savgol_filter(y_coords, 31, 3)

cx, cy, cyaw, ck = get_straight_course2(dl,y_f,x_f)
state = State(x=cx[0], y=cy[0], yaw=1.57)
goal = [cx[-1], cy[-1]]
if state.yaw - cyaw[0] >= math.pi:
    state.yaw -= math.pi * 2.0
elif state.yaw - cyaw[0] <= -math.pi:
    state.yaw += math.pi * 2.0


x = [state.x]
y = [state.y]
yaw = [state.yaw]
pos = [0,0]
t = [0.0]
d = [0.0]
v = [0.0]
target_ind, _ = calc_nearest_index(state, cx, cy, cyaw, 0)

odelta, ov = [0,0,0,0,0], None

cyaw = smooth_yaw(cyaw)

######MPC Parameters#########

NX = 3  # x = x, y, yaw
NU = 2  # a = [accel, steer]
T = 5  # horizon length

# mpc parameters
R = np.diag([0.01, 0.01])  # input cost matrix
Rd = np.diag([0.01, 1.0])  # input difference cost matrix
Q = np.diag([1.0, 1.0, 0.5])  # state cost matrix
Qf = Q  # state final matrix
GOAL_DIS = 1.5  # goal distance
STOP_SPEED = 0.5 / 3.6  # stop speed
MAX_TIME = 500.0  # max simulation time

# iterative paramter
MAX_ITER = 3  # Max iteration
DU_TH = 0.5  # iteration finish param

TARGET_SPEED = 7.2 / 3.6  # [m/s] target speed
N_IND_SEARCH = 10  # Search index number

DT = 0.05  # [s] time tick

# Vehicle parameters
LENGTH = 0.45  # [m]
WIDTH = 0.2  # [m]
BACKTOWHEEL = 0.1  # [m]
WHEEL_LEN = 0.03  # [m]
WHEEL_WIDTH = 0.02  # [m]
TREAD = 0.07  # [m]
WB = 0.15  # [m]

MAX_STEER = np.deg2rad(30.0)  # maximum steering angle [rad]
MAX_DSTEER = np.deg2rad(30.0)  # maximum steering speed [rad/s]
MAX_SPEED = 7.2 / 3.6  # maximum speed [m/s]
MIN_SPEED = -20.0 / 3.6  # minimum speed [m/s]
MAX_ACCEL = 1.0  # maximum accel [m/ss]
     
def set_position(data):
    global ov, target_ind, odelta        
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

    state = State(x=pos[0], y=pos[1], yaw=yaw)

    print("Car is at :",state.x, state.y, state.yaw)

    if check_goal(state, goal):
        print("Goal")
        print("Stopping car...")
        stop_car()
        sub.unregister()
    else:
        xref, target_ind, dref = calc_ref_trajectory(
            state, cx, cy, cyaw, ov, ck, dl, target_ind)
        x0 = [state.x, state.y, state.yaw]
        ov, odelta, ox, oy, oyaw = iterative_linear_mpc_control(
            xref, x0, dref, ov, odelta)
        if odelta is not None:
            di, vi = odelta[0], ov[0]
        control_car(vi, di)

    """               
    if not (err<0.5 or ((x_des-pos[0])<0.3) or ((y_des-pos[1])<0.3)):
        control_car(pos,yaw)
    else:
        print("Stopping car...")
        stop_car()
        sub.unregister()        
        servo_commands()
    """

       

def control_car(throttle,steer):
        
    msg = AckermannDriveStamped()

    print("throttle : ",throttle,"steer : ",steer)
  
    msg.drive.speed = throttle 
    x_pub.publish(msg)
           
    msg.drive.steering_angle = steer
    x_pub.publish(msg)

def get_linear_model_matrix(v, phi, delta):

    A = np.zeros((NX, NX))
    A[0, 0] = 1.0
    A[1, 1] = 1.0
    A[2, 2] = 1.0
    A[0, 2] = - DT * v * math.sin(phi)
    A[1, 2] = DT * v * math.cos(phi)

    B = np.zeros((NX, NU))
    B[0, 0] = DT * math.cos(phi)
    B[1, 0] = DT * math.sin(phi)
    B[2,0] = DT * math.tan(delta)/WB
    B[2, 1] = DT * v / (WB * math.cos(delta) ** 2)

    C = np.zeros(NX)
    C[0] = DT * v * math.sin(phi) * phi
    C[1] = - DT * v * math.cos(phi) * phi
    C[2] = - DT * v * delta / (WB * math.cos(delta) ** 2)

    return A, B, C

def update_state(state, v, delta):

    # input check
    if delta >= MAX_STEER:
        delta = MAX_STEER
    elif delta <= -MAX_STEER:
        delta = -MAX_STEER

    state.x = state.x + v * math.cos(state.yaw) * DT
    state.y = state.y + v * math.sin(state.yaw) * DT
    state.yaw = state.yaw + v / WB * math.tan(delta) * DT
    
    return state

def get_nparray_from_matrix(x):
    return np.array(x).flatten()

def predict_motion(x0, ov, od, xref):
   
    xbar = xref * 0.0
    for i, _ in enumerate(x0):
        xbar[i, 0] = x0[i]

    state = State(x=x0[0], y=x0[1], yaw=x0[2])
    for (vi, di, i) in zip(ov, od, range(1, T + 1)):
        state = update_state(state, vi, di)
        xbar[0, i] = state.x
        xbar[1, i] = state.y
        xbar[2, i] = state.yaw

    return xbar

def iterative_linear_mpc_control(xref, x0, dref, ov, od):
    """
    MPC contorl with updating operational point iteraitvely
    """

    if ov is None or od is None:
        ov = [0.0] * T
        od = [0.0] * T

    for i in range(MAX_ITER):
        xbar = predict_motion(x0, ov, od, xref)
        pov, pod = ov[:], od[:]
        ov, od, ox, oy, oyaw = linear_mpc_control(xref, xbar, x0, dref, ov)
        print("throttle : ",ov[0],"steer : ",od[0])
        du = sum(abs(ov - pov)) + sum(abs(od - pod))  # calc u change value
        if du <= DU_TH:
            break
    else:
        print("Iterative is max iter")

    return ov, od, ox, oy, oyaw

def linear_mpc_control(xref, xbar, x0, dref, ov):
    """
    linear mpc control

    xref: reference point
    xbar: operational point
    x0: initial state
    dref: reference steer angle
    """

    x = cvxpy.Variable((NX, T + 1))
    u = cvxpy.Variable((NU, T))

    cost = 0.0
    constraints = []

    for t in range(T):
        cost += cvxpy.quad_form(u[:, t], R)

        if t != 0:
            cost += cvxpy.quad_form(xref[:, t] - x[:, t], Q)

        A, B, C = get_linear_model_matrix(
            ov[0], xbar[2, t], dref[0, t])
        constraints += [x[:, t + 1] == A * x[:, t] + B * u[:, t] + C]

        if t < (T - 1):
            cost += cvxpy.quad_form(u[:, t + 1] - u[:, t], Rd)
            constraints += [cvxpy.abs(u[1, t + 1] - u[1, t]) <=
                            MAX_DSTEER * DT]

    cost += cvxpy.quad_form(xref[:, T] - x[:, T], Qf)

    constraints += [x[:, 0] == x0]
    constraints += [u[0, :] <= MAX_SPEED]
    constraints += [u[0, :] >= MIN_SPEED]
    

    prob = cvxpy.Problem(cvxpy.Minimize(cost), constraints)
    prob.solve(solver=cvxpy.ECOS, verbose=False)

    if prob.status == cvxpy.OPTIMAL or prob.status == cvxpy.OPTIMAL_INACCURATE:
        ox = get_nparray_from_matrix(x.value[0, :])
        oy = get_nparray_from_matrix(x.value[1, :])
        oyaw = get_nparray_from_matrix(x.value[2, :])
        ov = get_nparray_from_matrix(u.value[0, :])
        odelta = get_nparray_from_matrix(u.value[1, :])

    else:
        print("Error: Cannot solve mpc..")
        ov, odelta, ox, oy, oyaw = None, None, None, None, None, None

    return ov, odelta, ox, oy, oyaw

def calc_ref_trajectory(state, cx, cy, cyaw, ov, ck, dl, pind):
    xref = np.zeros((NX, T + 1))
    dref = np.zeros((1, T + 1))
    ncourse = len(cx)

    ind, _ = calc_nearest_index(state, cx, cy, cyaw, pind)

    if pind >= ind:
        ind = pind

    xref[0, 0] = cx[ind]
    xref[1, 0] = cy[ind]
    xref[2, 0] = cyaw[ind]
    dref[0, 0] = 0.0  # steer operational point should be 0

    travel = 0.0

    for i in range(T + 1):
        
        if ov is not None:  

            travel += abs(ov[0]) * DT
            dind = int(round(travel / dl))
            if (ind + i) < ncourse:
                xref[0, i] = cx[ind + i]
                xref[1, i] = cy[ind + i]
                xref[2, i] = cyaw[ind + i]
                dref[0, i] = 0.0
            else:
                xref[0, i] = cx[ncourse - 1]
                xref[1, i] = cy[ncourse - 1]
                xref[2, i] = cyaw[ncourse - 1]
                dref[0, i] = 0.0
        else:
            xref[0, i] = cx[i]
            xref[1, i] = cy[i]
            xref[2, i] = cyaw[i]
            dref[0, i] = 0.0

        """

        if (ind + dind) < ncourse:
            xref[0, i] = cx[ind + dind]
            xref[1, i] = cy[ind + dind]
            xref[2, i] = cyaw[ind + dind]
            dref[0, i] = 0.0
        else:
            xref[0, i] = cx[ncourse - 1]
            xref[1, i] = cy[ncourse - 1]
            xref[2, i] = cyaw[ncourse - 1]
            dref[0, i] = 0.0
        """

    return xref, ind, dref

def check_goal(state, goal):

    # check goal
    dx = state.x - goal[0]
    dy = state.y - goal[1]
    d = math.hypot(dx, dy)

    isgoal = (d <= GOAL_DIS)

    if isgoal:
        return True

    return False

def stop_car():
    msg = AckermannDriveStamped()
    msg.drive.speed = 0
    msg.drive.steering_angle = 0
    msg.drive.steering_angle_velocity = 0
    x_pub.publish(msg)
    print("Goal Reached!") 

def servo_commands():
    print(__file__ + " start!!")
 
    global sub
    
    msg = AckermannDriveStamped()

    sub = rospy.Subscriber("/gazebo/model_states", ModelStates, set_position)   

    while not (rospy.is_shutdown()):
        time.sleep(1)
     
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()




if __name__ == '__main__':
    try:
        rospy.init_node('servo_commands', anonymous=True)
               
        servo_commands()
    except rospy.ROSInterruptException:
        pass
