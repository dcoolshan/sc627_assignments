#!/usr/bin/env python3

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from helper import *
import matplotlib.pyplot as plt
#import other helper files if any


rospy.init_node('test', anonymous= True)

#Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

#read input file
input_file=open(r"/home/shantanu/catkin_ws/src/sc627_assignments/assignment_1/input.txt","r")
input_text=input_file.readlines()
# print(input_text)

start_x,start_y=float(input_text[0][0]),float(input_text[0][2])
start=(start_x,start_y)
goal_x,goal_y=float(input_text[1][0]),float(input_text[1][2])
goal=(goal_x,goal_y)
step_size=float(input_text[2].split("\n")[0])


obstacles=[]
poly=[]


for i in range(4,len(input_text)):
    if input_text[i]=="\n":
        
        obstacles.append(poly)
        poly=[]
    else:
        pt=input_text[i].split("\n")[0]
        temp=pt.split(",")
        temp=list(map(float,temp))

        poly.append(temp)


obstacles.append(poly)

current_pos_x=start_x
current_pos_y=start_y
current_pos=[current_pos_x,current_pos_y]
path=[[start_x,start_y]]

vec_sg_x=goal_x-start_x
vec_sg_y=goal_y-start_y
vec_magnitude=sqrt(vec_sg_x**2+vec_sg_y**2)
vec_sg_x=step_size*vec_sg_x/vec_magnitude
vec_sg_y=step_size*vec_sg_y/vec_magnitude
vec_sg=(vec_sg_x,vec_sg_y)


#setting result as initial location
result = MoveXYResult()
result.pose_final.x = start_x
result.pose_final.y = start_y

result.pose_final.theta = calculate_angle(vec_sg_x,vec_sg_y) #in radians (0 to 2pi)


while computeDistancePointToPoint(current_pos_x,current_pos_y,*goal)>step_size:
# for j in range(50):
    dist_lis=[]
    w_lis=[]
    ind_lis=[]
    for i in range(len(obstacles)):
        dist,w,ind=computeDistancePointToPolygon(obstacles[i],current_pos_x,current_pos_y)
        dist_lis.append(dist)
        w_lis.append(w)
        ind_lis.append(ind)

    min_ind=np.argmin(dist_lis)
    
    # print(dist_lis)

    if dist_lis[min_ind]<step_size:
        # print("loop entered")
        print("Failure: There is an obstacle lying between the start and goal")
        break

    else:
        # print("entered")
        current_pos_x=current_pos_x+vec_sg_x
        current_pos_y=current_pos_y+vec_sg_y    
        # print("current_pos updated")


        wp = MoveXYGoal()
        wp.pose_dest.x = current_pos_x
        wp.pose_dest.y = current_pos_y

        wp.pose_dest.theta = calculate_angle(vec_sg_x,vec_sg_y) #theta is the orientation of robot in radians (0 to 2pi)

        #send waypoint to turtlebot3 via move_xy server
        client.send_goal(wp)

        client.wait_for_result()

        #getting updated robot location
        result = client.get_result()

        current_pos_x=result.pose_final.x
        current_pos_y=result.pose_final.y

        
        path.append([current_pos_x,current_pos_y])

        if computeDistancePointToPoint(current_pos_x,current_pos_y,goal_x,goal_y) < step_size:
            print("Success")
            break


# print(path)
file_output=open(r"/home/shantanu/catkin_ws/src/sc627_assignments/assignment_1/output_base.txt","w")
for i in range(len(path)):
    data_pt=str(path[i][0])+","+str(path[i][1])
    file_output.write(data_pt)
    file_output.write("\n")
lis_x=[]
lis_y=[]
for i in range(len(path)):
    lis_x.append(path[i][0])
    lis_y.append(path[i][1])

plt.plot(lis_x,lis_y)
plt.show()



