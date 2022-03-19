#!/usr/bin/env python3
import matplotlib.pyplot as plt
from helper import *
from math import *

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

if __name__ == '__main__':

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



	while (abs(compute_gradient(current_pos,goal,obstacles)[0])>10**(-6)) and (abs(compute_gradient(current_pos,goal,obstacles)[1])>10**(-6)):

		print("loop entered")
		grad=compute_gradient(current_pos,goal,obstacles)
		print(current_pos)
		print(grad)
		current_pos_x=current_pos_x-step_size*grad[0]
		current_pos_y=current_pos_y-step_size*grad[1]
		current_pos=[current_pos_x,current_pos_y]
		path.append(current_pos)

	lis_x=[]
	lis_y=[]

	for i in range(len(path)):
		lis_x.append(path[i][0])
		lis_y.append(path[i][1])

	plt.plot(lis_x,lis_y)
	plt.show()
