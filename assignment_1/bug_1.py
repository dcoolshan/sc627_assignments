#!/usr/bin/env python3

from sc627_helper.msg import MoveXYAction, MoveXYGoal, MoveXYResult
import rospy
import actionlib
from helper import *
import matplotlib.pyplot as plt
import time
#import other helper files if any


rospy.init_node('test', anonymous= True)

#Initialize client
client = actionlib.SimpleActionClient('move_xy', MoveXYAction)
client.wait_for_server()

#read input file
input_file=open(r"input.txt","r")
input_text=input_file.readlines()
# print(input_text)

start_x,start_y=map(float,input_text[0].split(","))
start=(start_x,start_y)
goal_x,goal_y=map(float,input_text[1].split(","))
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
    dist_lis=[]
    w_lis=[]
    ind_lis=[]
    for i in range(len(obstacles)):
        dist,w,ind=computeDistancePointToPolygon(obstacles[i],current_pos_x,current_pos_y)
        dist_lis.append(dist)
        w_lis.append(w)
        ind_lis.append(ind)
        # consider corner case when ind is equal to len(poly)-1

    min_ind=np.argmin(dist_lis)
    
    
    min_dist_obs=[]
    temp_lis=[]

    # lis_time=[]
    # lis_time.append(0)

    if dist_lis[min_ind]<step_size:
        
        initial_x=current_pos_x
        initial_y=current_pos_y
        # print(initial_x)
        # print(initial_y)
        dist=computeDistancePointToPoint(current_pos_x,current_pos_y,goal_x,goal_y)
        min_dist_obs.append(dist)
        temp_lis.append([current_pos_x,current_pos_y])

        while(True):
        
            vec_x,vec_y=computeTangentVectorToPolygon(obstacles[min_ind],current_pos_x,current_pos_y)
            # print("loop entered ",j)
            current_pos_x=current_pos_x+step_size*vec_x
            current_pos_y=current_pos_y+step_size*vec_y

            # start_time=time.time()
            wp = MoveXYGoal()
            wp.pose_dest.x = current_pos_x
            wp.pose_dest.y = current_pos_y

            wp.pose_dest.theta = calculate_angle(vec_x,vec_y) #theta is the orientation of robot in radians (0 to 2pi)

            #send waypoint to turtlebot3 via move_xy server
            client.send_goal(wp)

            client.wait_for_result()

            #getting updated robot location
            result = client.get_result()

            # time_lapsed=lis_time[-1]+time.time() - start_time

            # lis_time.append(time_lapsed)

            current_pos_x=result.pose_final.x
            current_pos_y=result.pose_final.y


            
            dist=computeDistancePointToPoint(current_pos_x,current_pos_y,goal_x,goal_y)
            # print(dist)
            min_dist_obs.append(dist)
            temp_lis.append([current_pos_x,current_pos_y])
            path.append([current_pos_x,current_pos_y])

            if (start_x<current_pos_x<initial_x and current_pos_y<initial_y and path[-2][1]>initial_y) or (start_x>current_pos_x>initial_x and current_pos_y>initial_y and path[-2][1]<initial_y):
          
                break


        index_val=np.argmin(min_dist_obs)
        # print("loop left")

        while(True):
            vec_x,vec_y=computeTangentVectorToPolygon(obstacles[min_ind],current_pos_x,current_pos_y)
            current_pos_x=current_pos_x+step_size*vec_x
            current_pos_y=current_pos_y+step_size*vec_y

            # start_time=time.time()

            wp = MoveXYGoal()
            wp.pose_dest.x = current_pos_x
            wp.pose_dest.y = current_pos_y

            wp.pose_dest.theta = calculate_angle(vec_x,vec_y) #theta is the orientation of robot in radians (0 to 2pi)

            #send waypoint to turtlebot3 via move_xy server
            client.send_goal(wp)

            client.wait_for_result()

            #getting updated robot location
            result = client.get_result()

            # time_lapsed=lis_time[-1]+time.time() - start_time

            # lis_time.append(time_lapsed)

            current_pos_x=result.pose_final.x
            current_pos_y=result.pose_final.y


            path.append([current_pos_x,current_pos_y])
            # print("loop entered")

            # if current_pos_x>temp_lis[index_val][0] and current_pos_y>temp_lis[index_val][1]:
            if computeDistancePointToPoint(current_pos_x,current_pos_y,*temp_lis[index_val])<step_size*5:
                vec_sg_x=goal_x-current_pos_x
                vec_sg_y=goal_y-current_pos_y
                vec_magnitude=sqrt(vec_sg_x**2+vec_sg_y**2)
                vec_sg_x=step_size*vec_sg_x/vec_magnitude
                vec_sg_y=step_size*vec_sg_y/vec_magnitude
                vec_sg=(vec_sg_x,vec_sg_y)
                break


    else:
        current_pos_x=current_pos_x+vec_sg_x
        current_pos_y=current_pos_y+vec_sg_y

        # start_time=time.time()

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

        # time_lapsed=lis_time[-1]+time.time() - start_time

        # lis_time.append(time_lapsed)


        # print("current_pos updated")
        path.append([current_pos_x,current_pos_y])











# print(path)
file_output=open(r"output_1.txt","w")
for i in range(len(path)):
    data_pt=str(path[i][0])+","+str(path[i][1])
    file_output.write(data_pt)
    file_output.write("\n")
file_output.close()
dist_from_goal=[]
lis_x=[]
lis_y=[]
for i in range(len(path)):
    lis_x.append(path[i][0])
    lis_y.append(path[i][1])

path_length=0

for i in range(len(path)):
    dist_from_goal.append(computeDistancePointToPoint(*path[i],goal_x,goal_y))

for i in range(1,len(path)):
    path_length=path_length+computeDistancePointToPoint(*path[i-1],*path[i])

print("total path length is ",path_length)
# print("Total time --- %s seconds ---" % (time.time() - start_time))
plt.plot(lis_x,lis_y)
# plt.savefig("path_bot.png")
plt.show()
# figure
plt.plot([i for i in range(len(path))],dist_from_goal)
plt.ylabel("Distance from goal")
plt.xlabel("No. of steps")
plt.show()
# plt.savefig("dist_vs_iter.png")

# plt.plot(lis_time,dist_from_goal)
# plt.show()
# plt.savefig("dist_vs_time.jpg")
