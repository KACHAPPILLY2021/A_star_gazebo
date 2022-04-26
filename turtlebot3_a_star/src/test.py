#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
# import queue
import time
import matplotlib.pyplot as plt
import numpy as np
import math as mt
from collections import deque
import copy
try:
   import queue
except ImportError:
   import Queue as queue

start_node = [10.0, 10.0, 00.0]
goal_node = [90.0, 90.0] 
cost_to_come = 0
visited_node = {}
Queue = queue.PriorityQueue()
factor = 1
visited_region = np.zeros((int(factor*200), int(factor*200)))  # matrix to store the visited region. It is 3 dimensional matrix
UL = 50
UR = 48
clearance = 2.0
path_not_found = False

##############################################################
#### Define obstacle zone considering clearance and radius ###
##############################################################

# Function to check whether any part of the robot is in the obstacle region or not
def isObstacle(x_c, y_c , clearance) : 
    radius = 1
    # queue to store points along the circumference of given robot position
    cir=deque()
    # Dividing the circle into 36 equal points
    for i in range(36):
        t = (2*mt.pi)*(i) /36
        # Radius of circle = radius of robot + clearance 
        y = y_c + (radius+clearance)*mt.sin((t))
        x = x_c+ (radius+clearance)*mt.cos((t))
        x=round(x)
        y=round(y)
        cir.append((x,y))
    # Passing the coordinates of robot circle through each shape    
    bool_1 = circle1(cir) 
    bool_2 = circle2(cir)  
    bool_3 = square(cir) 
    bool_4 = rec_1(cir)
    bool_5 = rec_2(cir)  
    bool_6 = boundary(cir) 
    if bool_1 or bool_2 or bool_3 or bool_4 or bool_5 or bool_6:
        return True
    else :
        return False

def circle1(arr):
    # Passing each value of queue through circle equation
    for i in range(36) :
        b = (arr[i][0] - (factor*20))**2 + (arr[i][1] - (factor*80))**2 < (factor*10)**2
        if b == True:
            return b
    return False

def circle2(arr):
    # Passing each value of queue through circle equation
    for i in range(36) :
        b = (arr[i][0] - (factor*20))**2 + (arr[i][1] - (factor*20))**2 < (factor*10)**2
        if b == True:
            return b
    return False

def square(arr):
    for i in range(36):
        if arr[i][0]-(factor*2.5) > 0:
            if arr[i][1]-(factor*42.5) > 0:
                if arr[i][0]-(factor*17.5)<0:
                    if arr[i][1]-(factor*57.5)<0:
                        return True                    

def rec_1(arr):
    for i in range(36):
        if arr[i][0]-(factor*37.5) > 0:
            if arr[i][1]-(factor*42.5) > 0:
                if arr[i][0]-(factor*62.5)<0:
                    if arr[i][1]-(factor*57.5)<0:
                        return True

def rec_2(arr):
    for i in range(36):
        if arr[i][0]-(factor*72.5) > 0:
            if arr[i][1]-(factor*20) > 0:
                if arr[i][0]-(factor*87.5)<0:
                    if arr[i][1]-(factor*40)<0:
                        return True  

def boundary(arr):    
    for i in range(36) :
        # These equations signify the horizonal and vertical boundaries of the plane
        if arr[i][0]<(0) or arr[i][0]>(factor*100) or arr[i][1]<(0) or arr[i][1]>(factor*100) :
            return True


##################################
#### plot define obstacle zone ###
##################################

fig, ax = plt.subplots()
x_c =[]
y_c =[]
# Defined functions for the obstacle regions using half-plane method
def plot_circle1(x,y):
    return ((x - (factor*20))**2 + (y - (factor*80))**2) < (factor*10)**2

def plot_circle2(x,y):
    return ((x - (factor*20))**2 + (y - (factor*20))**2) < (factor*10)**2

def plot_square(x,y):
    if x-(factor*2.5) > 0:
        if y-(factor*42.5) > 0:
            if x-(factor*17.5)<0:
                if y-(factor*57.5)<0:
                    return True

def plot_rec_1(x,y):
    if x-(factor*37.5) > 0:
        if y-(factor*42.5) > 0:
            if x-(factor*62.5)<0:
                if y-(factor*57.5)<0:
                    return True

def plot_rec_2(x,y):
    if x-(factor*72.5) > 0:
        if y-(factor*20) > 0:
            if x-(factor*87.5)<0:
                if y-(factor*40)<0:
                    return True                    
# Function to check whether a given point is in obstacle region or not         
def actual_obstacle(X,Y) : 
    bool_1 = plot_circle1(X,Y) 
    bool_2 = plot_circle2(X,Y)  
    bool_3 = plot_square(X,Y) 
    bool_4 = plot_rec_1(X,Y)
    bool_5 = plot_rec_2(X,Y)   
    if bool_1 or bool_2 or bool_3 or bool_4 or bool_5 :
        return True
    else :
        return False

# Passing all coordinates to find obstacle coordinates
for i in range(factor*101) :
    for j in range(factor*101) :
        if actual_obstacle(i,j) :
            x_c.append(i)
            y_c.append(j) 
# Plotting obstacles

plt.grid()

ax.set_aspect('equal')
plt.xlim(0,factor*100)
plt.ylim(0,factor*100)
plt.scatter(x_c , y_c , c='red' , s=5)

#######################################
#### Function for taking user input ###
#######################################


# Takes 'x coordinate, y coordinate, orientation' of start position 
def takeStartInput():
    print("Enter x and y co-ordinate of start node(Values ranging inbetween 0 and 10)."+"\n"+ "Please press 'Enter' key after adding each element: ")
    for i in range(3):
         # Inputing coordinates
        if i<2:
            start_node[i] = int(input())*10
        # Inputing orientation
        if i==2:
            # print("please enter k (0 <= k <= 11) for orientation of start node of the robot. Orientation will be equal to 30*k: ")
            print("please enter robot orientation at start position: ")
            k = int(input())
            if k >= 360:
                while k < 360:
                    k = k - 360
            if k < 0:
                while k >= 0:
                    k = k + 360
            start_node[i] = k
            # if k < 0 or k > 11:
            #     print("k should be 0 <= k <= 11.")
            #     takeStartInput()

            
    # Checking for obstacle
    isTrue = isObstacle(start_node[0], start_node[1],clearance)
    if isTrue:
        print("Start node is either in obstacle area or in clearnace area."+"\n"+ " RUN THE PROGRAM AGAIN ")
        takeStartInput()

# Takes 'x coordinate, y coordinate, orientation' of goal position    
def takeGoalInput():
    print("Enter x and y co-ordinate of GOAL node(Values ranging inbetween 0 and 10)."+"\n"+ "Please press 'Enter' key after adding each element: ")
    for i in range(2):
      goal_node[i] = int(input())*10
        # if i==2:
        #     if i==2:
        #         print("please enter k (0 <= k <= 11) for orientation of goal node of the robot. Orientation will be equal to 30*k: ")
        #     k = int(input())
        #     goal_node[i] = 30*k
        #     if k < 0 or k > 11:
        #         print("k should be 0 <= k <= 11")
        #         takeGoalInput()
            
    # Checking for obstacle
    isTrue = isObstacle(goal_node[0], goal_node[1],clearance)
    if isTrue:
        print("Goal node is either in obstacle area or in clearnace area."+"\n"+ " RUN THE PROGRAM AGAIN ")
        takeGoalInput()

############################
#### Function for action ###
############################

def generateNode(Xi,Yi,Thetai,UL,UR):
    t = 0                           # variable to keep track of time for each iteration
    r = 0.033                       # radius of robot
    L = 0.16                        # span of the wheel
    dt = 0.1                        # time interval at which x and y should be computed 
    Xn=Xi                           # end position of x coordinate (intial x + change in the x)
    Yn=Yi                           # end position of y coordinate (intial y + change in the y)
    Thetan = 3.14 * Thetai / 180    # end orientation  (intial theta + change in the theta)
    D=0
    # UL = UL*3.14/60
    # UR = UR*3.14/60
    while t<1:
        t = t + dt
        Delta_Xn = 0.5*r * (UL + UR) * mt.cos(Thetan) * dt
        Xn = Xn + Delta_Xn
        # print(Delta_Xn)
        Delta_Yn = 0.5*r * (UL + UR) * mt.sin(Thetan) * dt
        Yn = Yn + Delta_Yn
        Thetan = Thetan +  (r / L) * (UR - UL) * dt
        D = D +  mt.sqrt(mt.pow((Delta_Xn),2)+mt.pow((Delta_Yn),2))
        
    Thetan = 180 * (Thetan) / 3.14
    # Xn = (round(Xn*2))/2
    # Yn = (round(Yn*2))/2
    if Thetan >= 360:
        n = round(Thetan/360)
        Thetan = Thetan - n*360
    if Thetan < 0:
        n = round(np.abs(Thetan/360))
        if n == 0:
            n = 1
        Thetan = Thetan + n*360
    return Xn, Yn, Thetan, D

# Creation of new nodes
def exploreNode(current_node,current_cost, UL, UR, clearance):
    # generate new node for and action (0, UL)
    next_x, next_y, next_theta, next_cost = generateNode(current_node[0], current_node[1], current_node[2], 0, UL)
    # print(next_theta)
    # check if the new node is falling in obstacle
    isTrue = isObstacle(next_x, next_y, clearance)
    if not isTrue:
        # check if the new node is already visited
        node1 = (next_x, next_y)
        # print(node1)
        if not isVisited(node1):
            cost_to_come = current_cost + next_cost
            cost_to_go = costToGo(node1)
            # calculate new cost for the new node
            new_cost = cost_to_come + cost_to_go
            # append new node into priority queue
            Queue.put((new_cost,(node1[0], node1[1],next_theta,cost_to_come,current_node[0], current_node[1],current_node[2], 0, UL)))
            # plt.arrow(current_node[0], current_node[1],node1[0]-current_node[0],node1[1]-current_node[1],fc="k", head_width=0.5, head_length=0.1)
            # plt.pause(0.005)
            
    # generate new node for and action (UL, 0)
    next_x, next_y, next_theta, next_cost = generateNode(current_node[0], current_node[1], current_node[2], UL, 0)
    # print(next_theta)
    # check if the new node is falling in obstacle
    isTrue = isObstacle(next_x, next_y, clearance)
    if not isTrue:
        # check if the new node is already visited
        node2 = (next_x, next_y)
        if not isVisited(node2):
            cost_to_come = current_cost + next_cost
            cost_to_go = costToGo(node2)
            # calculate new cost for the new node
            new_cost = cost_to_come + cost_to_go
            # append new node into priority queue
            Queue.put((new_cost,(node2[0], node2[1],next_theta,cost_to_come,current_node[0], current_node[1],current_node[2], UL, 0)))
            # plt.arrow(current_node[0], current_node[1],node2[0]-current_node[0],node2[1]-current_node[1],fc="k", head_width=0.5, head_length=0.1)
            # plt.pause(0.005)

    # generate new node for and action (UL, UL)
    next_x, next_y, next_theta, next_cost = generateNode(current_node[0], current_node[1], current_node[2], UL, UL)
    # check if the new node is falling in obstacle
    isTrue = isObstacle(next_x, next_y, clearance)
    if not isTrue:
        # check if the new node is already visited
        node3 = (next_x, next_y)
        if not isVisited(node3):
            cost_to_come = current_cost + next_cost
            cost_to_go = costToGo(node3)
            # calculate new cost for the new node
            new_cost = cost_to_come + cost_to_go
            # append new node into priority queue
            Queue.put((new_cost,(node3[0], node3[1],next_theta,cost_to_come,current_node[0], current_node[1],current_node[2], UL, UL)))
            # plt.arrow(current_node[0], current_node[1],node3[0]-current_node[0],node3[1]-current_node[1],fc="k", head_width=0.5, head_length=0.1)
            # plt.pause(0.005)
    
    # generate new node for and action (UR, 0)
    next_x, next_y, next_theta, next_cost = generateNode(current_node[0], current_node[1], current_node[2], UR, 0)
    # print(next_theta)
    # check if the new node is falling in obstacle
    isTrue = isObstacle(next_x, next_y, clearance)
    if not isTrue:
        # check if the new node is already visited
        node4 = (next_x, next_y)
        if not isVisited(node4):
            cost_to_come = current_cost + next_cost
            cost_to_go = costToGo(node4)
            # calculate new cost for the new node
            new_cost = cost_to_come + cost_to_go
            # append new node into priority queue
            Queue.put((new_cost,(node4[0], node4[1],next_theta,cost_to_come,current_node[0], current_node[1],current_node[2], UR, 0)))
            # plt.arrow(current_node[0], current_node[1],node4[0]-current_node[0],node4[1]-current_node[1],fc="k", head_width=0.5, head_length=0.1)
            # plt.pause(0.005)
    
    # generate new node for and action (0, UR)
    next_x, next_y, next_theta, next_cost = generateNode(current_node[0], current_node[1], current_node[2], 0, UR)
    # print(next_theta)
    # check if the new node is falling in obstacle
    isTrue = isObstacle(next_x, next_y, clearance)
    if not isTrue:
        # check if the new node is already visited
        node5 = (next_x, next_y)
        if not isVisited(node5):
            cost_to_come = current_cost + next_cost
            cost_to_go = costToGo(node5)
            # calculate new cost for the new node
            new_cost = cost_to_come + cost_to_go
            # append new node into priority queue
            Queue.put((new_cost,(node5[0], node5[1],next_theta,cost_to_come,current_node[0], current_node[1],current_node[2], 0, UR)))
            # plt.arrow(current_node[0], current_node[1],node5[0]-current_node[0],node5[1]-current_node[1],fc="k", head_width=0.5, head_length=0.1)
            # plt.pause(0.005)

    # generate new node for and action (UR, UR)
    next_x, next_y, next_theta, next_cost = generateNode(current_node[0], current_node[1], current_node[2], UR, UR)
    # print(next_theta)
    # check if the new node is falling in obstacle
    isTrue = isObstacle(next_x, next_y, clearance)
    if not isTrue:
        # check if the new node is already visited
        node6 = (next_x, next_y)
        if not isVisited(node6):
            cost_to_come = current_cost + next_cost
            cost_to_go = costToGo(node6)
            # calculate new cost for the new node
            new_cost = cost_to_come + cost_to_go
            # append new node into priority queue
            Queue.put((new_cost,(node6[0], node6[1],next_theta,cost_to_come,current_node[0], current_node[1],current_node[2], UR, UR)))
            # plt.arrow(current_node[0], current_node[1],node6[0]-current_node[0],node6[1]-current_node[1],fc="k", head_width=0.5, head_length=0.1)
            # plt.pause(0.005)

    # generate new node for and action (UL, UR)
    next_x, next_y, next_theta, next_cost = generateNode(current_node[0], current_node[1], current_node[2], UL, UR)
    # print(next_theta)
    # check if the new node is falling in obstacle
    isTrue = isObstacle(next_x, next_y, clearance)
    if not isTrue:
        # check if the new node is already visited
        node7 = (next_x, next_y)
        if not isVisited(node7):
            cost_to_come = current_cost + next_cost
            cost_to_go = costToGo(node7)
            # calculate new cost for the new node
            new_cost = cost_to_come + cost_to_go
            # append new node into priority queue
            Queue.put((new_cost,(node7[0], node7[1],next_theta,cost_to_come,current_node[0], current_node[1],current_node[2], UL, UR)))
            # plt.arrow(current_node[0], current_node[1],node7[0]-current_node[0],node7[1]-current_node[1],fc="k", head_width=0.5, head_length=0.1)
            # plt.pause(0.005)

    # generate new node for and action (UR, UL)
    next_x, next_y, next_theta, next_cost = generateNode(current_node[0], current_node[1], current_node[2], UR, UL)
    # print(next_theta)
    # check if the new node is falling in obstacle
    isTrue = isObstacle(next_x, next_y, clearance)
    if not isTrue:
        # check if the new node is already visited
        node8 = (next_x, next_y)
        if not isVisited(node8):
            cost_to_come = current_cost + next_cost
            cost_to_go = costToGo(node8)
            # calculate new cost for the new node
            new_cost = cost_to_come + cost_to_go
            # append new node into priority queue
            Queue.put((new_cost,(node8[0], node8[1],next_theta,cost_to_come,current_node[0], current_node[1],current_node[2], UR, UL)))
            # plt.arrow(current_node[0], current_node[1],node8[0]-current_node[0],node8[1]-current_node[1],fc="k", head_width=0.5, head_length=0.1)
            # plt.pause(0.005)

    # plt.pause(10e-20)
    
# function to compute cost to reach goal node
# cost to reach goal node is euclidean distance between current node and goal node
def costToGo(X):
    X = np.array([X[0],X[1]])
    Y = np.array([goal_node[0],goal_node[1]])
    return 1*np.sqrt(np.sum((X-Y)**2))

# function to check whether current node is goal node or not
# if the current node is within 1.5 unit euclidean distance of the goal node, it will be considered as goal node
def isGoalNode(X):
    rotation = X[2]
    X = np.array([X[0],X[1]])
    Y = np.array([goal_node[0],goal_node[1]])
    if np.sqrt(np.sum((X-Y)**2)) <= 2.5:
        return True
    else:
        return False

# update visited region
def updateVisitedRegion(X):
    # i = int(X[0]*8)
    # j = int(X[1]*8)
    i = int(round(X[0]*2))
    j = int(round(X[1]*2))

    
    visited_region[i][j] = 1
    
    return None

# check whether the current node is visited or not
def isVisited(X):
    # i = int(X[0]*8)
    # j = int(X[1]*8)
    i = int(round(X[0]*2))
    j = int(round(X[1]*2))

    
    if visited_region[i][j]:
        return True
    else:
        return False

##################
#### Main loop ###
##################
# def main():

# clearance = takeClearance()         # take clearance input from user
takeStartInput()                    # take start position and orientation input from user
takeGoalInput()                     # take goal position and orientation input from user
start_node = tuple(start_node)      
goal_node = tuple(goal_node)
total_cost = cost_to_come + costToGo(start_node)

# Inserting first element into the queue
# The first value of each element is 'total cost'
# The second value has the following structure:
# (x position current, y position current , orientation current , cost to come, x parent , y parent , orientation parent)  

Queue.put((total_cost,(start_node[0],start_node[1],start_node[2],cost_to_come,start_node[0],start_node[1],start_node[2],0,0)))

start = time.time()
# plt.plot()
# plt.axis([0,factor*100,0,factor*100])
# plt.title("exploring map to find goal node")

print("finding optimal path to reach goal....")
while True:
    # pop node from priority queue
    # print(Queue)
    
    if Queue.empty():
        print("Path not found")
        path_not_found = True
        break
    
    pop_node = Queue.get()
    current_node = (pop_node[1][0], pop_node[1][1], pop_node[1][2])
    cost_to_come = pop_node[1][3]
    parent_node = (pop_node[1][4], pop_node[1][5], pop_node[1][6])
    RPM_parents = (pop_node[1][7], pop_node[1][8])
    cost = pop_node[0]
    
    # check if the current node is goal node
    if isGoalNode(current_node):
        visited_node[current_node] = (parent_node, RPM_parents)
        # redefine the goal to the nearest reachanble position to the actual goal position
        goal_node = (current_node)
        print('reached at goal node')
        print(goal_node)
        print("HOORAY")
        break
    
    # if the current node is not goal node, explore the next visiting node from current node
    # print(current_node)
    if  isVisited(current_node)== False:
        visited_node[current_node] = (parent_node, RPM_parents)
        # update the visited note in visited region
        # print(current_node)
        updateVisitedRegion(current_node)
        # explore the new node from current node
        exploreNode(current_node,cost_to_come, UL, UR, clearance)
        
######################
#### Back-tracking ###
######################

# Seperating key and values from the visited_node dictionary
# key is the node and value is its parents node
# key list contain all the node and value list contain all the its parents 

key_list = list(visited_node.keys())
value_list = list(visited_node.values())

# Storing nodes of optimal path
x_path = []
y_path = []
RPM_list = [(0,0)]
while True:
    if path_not_found:
        break

    # find the index in the key list corrosponding node
    # find the value in the value list corrospoding to the above index, it will be its parents node 
    position = key_list.index((goal_node))
    value = value_list[position]
    # check if the index is start node during back tracking
    if key_list[position]==(start_node):
        x_path.append(key_list[position][0])
        y_path.append(key_list[position][1])
        break
    else:
        # append the first elements of node to x_path and y element of node to y_path
        x_path.append(key_list[position][0])
        y_path.append(key_list[position][1])
        RPM_list.append(value[1])
        # parent become next node and while loop will continue to run to find its parents 
        goal_node = value[0]

##############################
#### Plotting Optimal path ###
##############################

wheel_rotation_list = RPM_list

# x = []
# y = []

# if x_path:
#     Xi = x_path.pop()
#     Yi = y_path.pop()

#     Xn = Xi
#     Yn = Yi
    
#     x.append(Xn)
#     y.append(Yn)
    
# # Thetan = start_node[2]
# Thetan = 3.14 * start_node[2] / 180
# r = 0.033                       # radius of robot
# L = 0.16                        # span of the wheel
# dt = 0.1                        # time interval at which x and y should be computed 


# while True:
#     if not x_path:
#         break
#     if not RPM_list:
#         break
#     plt.title("Plotting optimal path to travel from start node to goal node using A* algorithm")
#     # x.append(x_path.pop())
#     # y.append(y_path.pop())
#     RPM = RPM_list.pop()
#     UL = RPM[0]
#     UR = RPM[1]
#     t = 0                           # variable to keep track of time for each iteration
#     while t<1:
#         t = t + dt
#         Delta_Xn = 0.5*r * (UL + UR) * mt.cos(Thetan) * dt
#         Xn = Xn + Delta_Xn
#         Delta_Yn = 0.5*r * (UL + UR) * mt.sin(Thetan) * dt
#         Yn = Yn + Delta_Yn
#         Thetan = Thetan +  (r / L) * (UR - UL) * dt
#         x.append(Xn)
#         y.append(Yn)

#     plt.plot(x, y, c = 'red', linewidth=2)

#     plt.pause(0.00000000000000005)

# plt.title("Optimal path to travel from start node to goal node using A* algorithm")
end = time.time()
print("time taken to run the code"+ " : " + str(end-start)+ " seconds ")       
# plt.show()

# def test():
rospy.loginfo("STARTING PUBLISHER")
msg=Twist()
pub=rospy.Publisher('/cmd_vel',Twist,queue_size=1)
rospy.init_node('robot_talker',anonymous=True)
r = 0.033
L = 0.16

while not rospy.is_shutdown():
    while True:
        if not wheel_rotation_list:
            msg.angular.z = 0
            msg.linear.x = 0
            rospy.loginfo('Reached at Goal Position')
            pub.publish(msg)
            break
        W = wheel_rotation_list.pop()
        v_l = (r*2*np.pi*W[0])/60
        v_r = (r*2*np.pi*W[1])/60
      
        Pho = 17*((v_r - v_l)/L)

        v_straight = (v_l + v_r)/2
        rospy.loginfo(Pho)
        if v_l == v_r:
            rospy.loginfo("Going straight")
            msg.linear.x = v_straight
            msg.angular.z = 0
        if Pho<0 :
            rospy.loginfo("Turning right")
            msg.linear.x = v_straight
            msg.angular.z=Pho

            rospy.sleep(0.165)

        if Pho>0 :
            rospy.loginfo("Turning left")
            msg.linear.x = v_straight
            msg.angular.z=Pho

            rospy.sleep(0.165)

        rospy.loginfo(msg)
        pub.publish(msg)

        rospy.sleep(0.9777)

    break


	
