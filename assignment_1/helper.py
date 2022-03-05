#!/usr/bin/env python3


from math import *
import numpy as np


# # 	return False
# def check_side(p,a,b,c):
# 	return a*p[0]+b*p[1]+c
# def check_intersection(p1,q1,p2,q2):
# 	a,b,c=computeLineThroughTwoPoints(*p1,*q1)
# 	a1,b1,c1=computeLineThroughTwoPoints(*p2,*q2)
# 	if check_side(p2,a,b,c)*check_side(q2,a,b,c)<0 and check_side(p1,a1,b1,c1)*check_side(q1,a1,b1,c1)<0:
# 		return True 
# 	return False

# def calculate_angle(vec_sg_x,vec_sg_y):
# 	if vec_sg_x>0 and vec_sg_y>0:
#         val=atan(vec_sg_y/vec_sg_x)

# 	elif (vec_sg_x<0 and vec_sg_y>0) or (vec_sg_x<0 and vec_sg_y<0):
# 	    val=pi+atan(vec_sg_y/vec_sg_x)

# 	else:
# 	    val=2*pi+atan(vec_sg_y/vec_sg_x)

# 	return val

def calculate_angle(vec_sg_x,vec_sg_y):

	if vec_sg_x==0:
		
		if vec_sg_y>0:
			val=pi/2
		else:
			val=3*pi/2

	elif vec_sg_x>0 and vec_sg_y>0:
		val=atan(vec_sg_y/vec_sg_x)

	elif (vec_sg_x<0 and vec_sg_y>0) or (vec_sg_x<0 and vec_sg_y<0):
		val=pi+atan(vec_sg_y/vec_sg_x)

	else:
		val=2*pi+atan(vec_sg_y/vec_sg_x)

	return val

def computeDistancePointToPoint(x1,y1,x2,y2):
	return sqrt((x1-x2)**2+(y1-y2)**2)
	
def computeLineThroughTwoPoints(x1,y1,x2,y2):
	if(x1==x2):
		a=1
		b=0
		c=-x1

	elif (y1==y2):
		a=0
		b=1
		c=-y1
	else:
		a=(y2-y1)/(x2-x1)
		b=-1
		c=y2-x2*(y2-y1)/(x2-x1)

	return a,b,c

def computeDistancePointToLine(x1,y1,x2,y2,x,y):
	a,b,c=computeLineThroughTwoPoints(x1,y1,x2,y2)

	dist=abs((a*x+b*y+c)/(sqrt(a**2+b**2)))
	return dist

def computeDistancePointToSegment(x1,y1,x2,y2,x,y):

	cond1=(x2-x1)*(x-x2)+(y2-y1)*(y-y2)
	cond2=(x2-x1)*(x-x1)+(y2-y1)*(y-y1)

	# print(cond1)
	# print(cond2)

	if(cond1>0 and cond2>0):
		dist=sqrt((x-x2)**2+(y-y2)**2)
		w=2
	elif(cond1<0 and cond2<0):
		dist=sqrt((x-x1)**2+(y-y1)**2)
		w=1
	else:
		dist=computeDistancePointToLine(x1,y1,x2,y2,x,y)
		w=0
	return dist,w


def computeDistancePointToPolygon(poly,x,y):
	# print(len(poly))
	dist=inf
	lis_dist=[]
	lis_w=[]

	for i in range(len(poly)):

		if(i==len(poly)-1):
			d,w=computeDistancePointToSegment(*poly[i],*poly[0],x,y)
			lis_dist.append(d)
			lis_w.append(w)

		else:
			d,w=computeDistancePointToSegment(*poly[i],*poly[i+1],x,y)
			lis_dist.append(d)
			lis_w.append(w)

	# print(lis_dist)
	ind=np.argmin(lis_dist)
	return lis_dist[ind],lis_w[ind],ind


def computeTangentVectorToPolygon(poly,x,y):
	edge=[]
	dist,w,ind=computeDistancePointToPolygon(poly,x,y)

	# if ind==len(poly)-1:
	# 	edge.append(poly[ind])
	# 	edge.append(poly[0])

	# else:
	# 	edge.append(poly[ind])
	# 	edge.append(poly[ind+1])

	if w==0:

		# w=0 is considering the case where polygon vertex input is already in anticlockwise sense
		if ind==len(poly)-1:
			vec_x=poly[0][0]-poly[ind][0]
			vec_y=poly[0][1]-poly[ind][1]
		else:
			vec_x=poly[ind+1][0]-poly[ind][0]
			vec_y=poly[ind+1][1]-poly[ind][1]

		vec_magnitude=sqrt(vec_x**2+vec_y**2)
		vec_x=vec_x/vec_magnitude
		vec_y=vec_y/vec_magnitude

	elif w==1:
			centre_x=poly[ind][0]
			centre_y=poly[ind][1]

			vec_x=(y-centre_y)
			vec_y=-(x-centre_x)

			norm_x=x-centre_x
			norm_y=y-centre_y

			if norm_x*vec_y-norm_y*vec_x < 0:
				vec_x=-vec_x
				vec_y=-vec_y
			vec_magnitude=sqrt(vec_x**2+vec_y**2)
			vec_x=vec_x/vec_magnitude
			vec_y=vec_y/vec_magnitude

	else:
		if ind==len(poly)-1:
			centre_x=poly[0][0]
			centre_y=poly[0][1]

		else:
			centre_x=poly[ind+1][0]
			centre_y=poly[ind+1][1]

		vec_x=(y-centre_y)
		vec_y=-(x-centre_x)

		norm_x=x-centre_x
		norm_y=y-centre_y

		if norm_x*vec_y-norm_y*vec_x < 0:
			vec_x=-vec_x
			vec_y=-vec_y
		vec_magnitude=sqrt(vec_x**2+vec_y**2)
		vec_x=vec_x/vec_magnitude
		vec_y=vec_y/vec_magnitude

			



	return vec_x,vec_y





if __name__ == '__main__':

	n=int(input("Enter the number of vertices of polygon"))
	poly=[]
	for i in range(n):
		val=list(map(float,input().strip().split(" ")))
		print(val)
		poly.append(val)

	x,y=map(float,input("enter the isolated point").strip().split())

	print(poly)
	print(x," ",y)
	# dist,w,ind=computeDistancePointToPolygon(poly,x,y)
	vec_x,vec_y=computeTangentVectorToPolygon(poly,x,y)
	# print(dist)	
	# print(w)
	# print(ind)
	print(vec_x)
	print(vec_y)