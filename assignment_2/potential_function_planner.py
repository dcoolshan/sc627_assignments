#!/usr/bin/env python3

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
import matplotlib.pyplot as plt
from helper import *
from math import *
#import other helper files if any



def compute_gradient_att(current_pos,goal):

    d_star_goal=2

    if computeDistancePointToPoint(*current_pos,*goal) <= d_star_goal:
        vec_x=0.8*(current_pos[0]-goal[0])
        vec_y=0.8*(current_pos[1]-goal[1])
        

    else:

        vec_x=(2*0.8/computeDistancePointToPoint(*current_pos,*goal))*(current_pos[0]-goal[0])
        vec_y=(2*0.8/computeDistancePointToPoint(*current_pos,*goal))*(current_pos[1]-goal[1])
        

    
    vec=[vec_x,vec_y]
    return vec


def compute_gradient_rep(current_pos,goal,obstacles):

    vec_x=0
    vec_y=0

    for i in range(len(obstacles)):

        poly=obstacles[i]

        dist,w,ind=computeDistancePointToPolygon(poly,*current_pos)

        if ind==len(poly)-1:

            if w==0:
                a,b,c=computeLineThroughTwoPoints(*poly[ind],*poly[0])

                x_perpendicular= -a*(a*current_pos_x+b*current_pos_y+c)/(a**2+b**2)+current_pos_x
                y_perpendicular= -b*(a*current_pos_x+b*current_pos_y+c)/(a**2+b**2)+current_pos_y

                grad_vec_x=current_pos_x-x_perpendicular
                grad_vec_y=current_pos_y-y_perpendicular

            elif w==1:
                grad_vec_x=current_pos_x-poly[ind][0]
                grad_vec_y=current_pos_y-poly[ind][1]

            elif w==2:
                grad_vec_x=current_pos_x-poly[0][0]
                grad_vec_y=current_pos_y-poly[0][1]


        else:

            if w==0:
                a,b,c=computeLineThroughTwoPoints(*poly[ind],*poly[ind+1])

                x_perpendicular=-a*(a*current_pos_x+b*current_pos_y+c)/(a**2+b**2)+current_pos_x
                y_perpendicular=-b*(a*current_pos_x+b*current_pos_y+c)/(a**2+b**2)+current_pos_y

                grad_vec_x=current_pos_x-x_perpendicular
                grad_vec_y=current_pos_y-y_perpendicular

            elif w==1:
                grad_vec_x=current_pos_x-poly[ind][0]
                grad_vec_y=current_pos_y-poly[ind][1]

            elif w==2:
                grad_vec_x=current_pos_x-poly[ind+1][0]
                grad_vec_y=current_pos_y-poly[ind+1][1]


        if dist<=2 :
            factor=0.8*((1/2)-(1/dist))*(1/(dist**2))
            vec_x=vec_x+factor*grad_vec_x
            vec_y=vec_y+factor*grad_vec_y
        
        else:
            vec_x=vec_x
            vec_y=vec_y

    
    vec=[vec_x,vec_y]
    

    return vec


def compute_gradient(current_pos,goal,obstacles):

    grad_att=compute_gradient_att(current_pos,goal)
    grad_rep=compute_gradient_rep(current_pos,goal,obstacles)

    vec_x=grad_att[0]+grad_rep[0]
    vec_y=grad_att[1]+grad_rep[1]

    vec_mag=sqrt(vec_x**2+vec_y**2)
    if vec_mag==0:
        vec=[vec_x,vec_y]
        return vec
    else:
        # vec_x=vec_x/vec_mag
        # vec_y=vec_y/vec_mag
        vec=[vec_x,vec_y]
        return vec




rospy.init_node('test', anonymous= True)

#Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

#read input file

input_file=open(r"/home/shantanu/catkin_ws/src/sc627_assignments/assignment_1/input.txt","r")
input_text=input_file.readlines()
# print(input_text)

start_x,start_y=map(float,input_text[0].split(","))
start=(start_x,start_y)
goal_x,goal_y=map(float,input_text[1].split(","))
goal=[goal_x,goal_y]
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


#setting result as initial location
result = MoveXYResult()
result.pose_final.x = 0
result.pose_final.y = 0
result.pose_final.theta = 0 #in radians (0 to 2pi)

step_size_orig=step_size


while (abs(compute_gradient(current_pos,goal,obstacles)[0])>10**(-4)) and (abs(compute_gradient(current_pos,goal,obstacles)[1])>10**(-4)):

    grad=compute_gradient(current_pos,goal,obstacles)

    pre_current_x=current_pos_x
    pre_current_y=current_pos_y

 
    current_pos_x=current_pos_x-step_size*grad[0]
    current_pos_y=current_pos_y-step_size*grad[1]


    vec_sg_x=current_pos_x-pre_current_x
    vec_sg_y=current_pos_y-pre_current_y

    wp = MoveXYGoal()
    wp.pose_dest.x = current_pos_x
    wp.pose_dest.y = current_pos_y
    wp.pose_dest.theta = calculate_angle(vec_sg_x,vec_sg_y) #theta is the orientation of robot in radians (0 to 2pi)

    #send waypoint to turtlebot3 via move_xy server
    client.send_goal(wp)

    client.wait_for_result()

    #getting updated robot location
    result = client.get_result()

    #write to output file (replacing the part below)
    # print(result.pose_final.x, result.pose_final.y, result.pose_final.theta)

    current_pos_x=result.pose_final.x
    current_pos_y=result.pose_final.y

    # step_size=step_size_orig

    if (current_pos_x-pre_current_x <= 0.001) and (current_pos_y-pre_current_y<=0.001):
        step_size=1.1*step_size

    current_pos=[current_pos_x,current_pos_y]
    print("current_position ",current_pos)
    print("gradient ", grad)
    path.append(current_pos)

file_output=open(r"assgn2.txt","w")
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



# while True: #replace true with termination condition

    #determine waypoint based on your algo
    #this is a dummy waypoint (replace the part below)
    # wp = MoveXYGoal()
    # wp.pose_dest.x = 1 + result.pose_final.x
    # wp.pose_dest.y = 0
    # wp.pose_dest.theta = 0 #theta is the orientation of robot in radians (0 to 2pi)

    # #send waypoint to turtlebot3 via move_xy server
    # client.send_goal(wp)

    # client.wait_for_result()

    # #getting updated robot location
    # result = client.get_result()

    # #write to output file (replacing the part below)
    # print(result.pose_final.x, result.pose_final.y, result.pose_final.theta)