import cv2
from math import *
import math
import numpy as np


def deg2rad(deg):
	return deg/180.0*pi
	
#假定路面为平面
#根据投影关系计算像素点对应的位置坐标
#相机坐标系(向前为Z,向右为X,向下为Y)
#像素坐标系(向右为u,向下为v)
#colCnt,rowCnt 为相机分辨率
#h 摄像头安装高度
#theta 摄像头安装向下倾斜角度(rad)
#fx,fy 相机归一化焦距
#cx,cy 相机光心坐标
def pixel2disTable(colCnt,rowCnt,h,theta,fx,fy,cx,cy):

	table = np.zeros((colCnt,rowCnt,2))
	
	for row in range(rowCnt):
		#当前像素行与焦点所形成的平面为plane，
		#theta_i 为 plane与光心轴的夹角
		theta_i = math.atan2(row-cy,fy)
		#当前像素超出地面范围
		if(theta_i <= -theta): 
			continue
		#当前像素行超出前方范围
		elif(theta_i+theta >= math.pi/2):
			continue
		#当前像素行所显示的地面距离摄像头在地面垂直投影点的距离
		z = h/math.tan(theta_i+theta)
		#像素点(cy,row)所显示地面点到摄像头的距离(斜线段L)
		l = math.sqrt(z*z+h*h)
		for col in range(colCnt):
			#当前像素点(col,row)所显示的地面与摄像头的连线为L1
			#alpha为L1与斜线段L的夹角
			alpha = math.atan2(col-cx,fx)
			x = l*math.tan(alpha)
			table[col][row][0] = x
			table[col][row][1] = z
	return table

def roll2Matrix(roll):
	return np.matrix([[1,0,0],
				  [0,cos(roll),-sin(roll)],
				  [0,sin(roll), cos(roll)]])
				  
def pitch2Matrix(pitch):
	return np.matrix([[cos(pitch),0,sin(pitch)],
				  [0,1,0],
				  [-sin(pitch),0,cos(pitch)]])
				  
def yaw2Matrix(yaw):
	return np.matrix([[cos(yaw),-sin(yaw),0],
				  [sin(yaw), cos(yaw),0],
				  [0,0,1]])

#欧拉角转旋转矩阵 绕固定系的旋转
def RPY2Matrix(roll,pitch,yaw):
	Rx = roll2Matrix(roll)
	Ry = pitch2Matrix(pitch)
	Rz = yaw2Matrix(yaw)
	return Rz*Ry*Rx

#欧拉角转旋转矩阵 绕自身系的旋转
def RPY2Matrix2(roll,pitch,yaw):
	Rx = roll2Matrix(roll)
	Ry = pitch2Matrix(pitch)
	Rz = yaw2Matrix(yaw)
	return Rx*Ry*Rz

#世界坐标系到相机坐标系的旋转矩阵
#theta为相机的安装俯仰角，向下倾斜为正
def world2cameraMatrix(theta):
	#按固定坐标系旋转
	Rx = roll2Matrix(-pi/2)
	Rz = yaw2Matrix(-pi/2)
	Ry = pitch2Matrix(theta)
	return Ry*Rz*Rx

#世界坐标点转到相机坐标系
def world2cameraPoint(theta,xyz):
	point = np.mat([[xyz[0]],[xyz[1]],[xyz[2]]])
	R = world2cameraMatrix(theta)
	point = R.I*point
	return (point[0,0],point[1,0],point[2,0])
	
#相机坐标转像素坐标
def xyz2pixel(xyz,fx,fy,cx,cy):
	x = xyz[0]
	y = xyz[1]
	z = xyz[2]
	u = x/z*fx+cx
	v = y/z*fy+cy
	return(int(u),int(v))


class RegionDivide:
	def __init__(self):
		self.mask = None
		pass
	
	#设置相机参数
	def setCameraParams(self,fx,fy,cx,cy,size,h,theta):
		self.fx = fx
		self.fy = fy
		self.cx = cx
		self.cy = cy
		self.size = size
		self.height = h  #摄像头安装高度
		self.theta = theta#摄像头安装倾斜角,下倾为正
	
	#设置区域划分参数
	def setRegionParams(self,L1,L2,L3,L4,L5):
		self.L1 = L1
		self.L2 = L2
		self.L3 = L3
		self.L4 = L4
		self.L5 = L5
		
		#x,y,z,区域分割点世界坐标
		A1 = [L1,L4/2,-self.height]
		A2 = [L1,-L4/2,-self.height]
		A3 = [L1+L2,L4/2,-self.height]
		A4 = [L1+L2,-L4/2,-self.height]
		A5 = [L1+L2+L3,L4/2,-self.height]
		A6 = [L1+L2+L3,-L4/2,-self.height]

		B1 = [L1,L4/2+L5,-self.height]
		B2 = [L1,-L4/2-L5,-self.height]
		B3 = [L1+L2,L4/2+L5,-self.height]
		B4 = [L1+L2,-L4/2-L5,-self.height]
		B5 = [L1+L2+L3,L4/2+L5,-self.height]
		B6 = [L1+L2+L3,-L4/2-L5,-self.height]
		
		#print("世界坐标: ",A1,A2,A3,A4,A5,A6)
		
		#区域分割点相机坐标
		A1 = world2cameraPoint(self.theta,A1)
		A2 = world2cameraPoint(self.theta,A2)
		A3 = world2cameraPoint(self.theta,A3)
		A4 = world2cameraPoint(self.theta,A4)
		A5 = world2cameraPoint(self.theta,A5)
		A6 = world2cameraPoint(self.theta,A6)
		#print("相机坐标: ",A1,A2,A3,A4,A5,A6)
		
		B1 = world2cameraPoint(self.theta,B1)
		B2 = world2cameraPoint(self.theta,B2)
		B3 = world2cameraPoint(self.theta,B3)
		B4 = world2cameraPoint(self.theta,B4)
		B5 = world2cameraPoint(self.theta,B5)
		B6 = world2cameraPoint(self.theta,B6)
		
		#区域分割点像素坐标
		self.a1 = xyz2pixel(A1,self.fx,self.fy,self.cx,self.cy)
		self.a2 = xyz2pixel(A2,self.fx,self.fy,self.cx,self.cy)
		self.a3 = xyz2pixel(A3,self.fx,self.fy,self.cx,self.cy)
		self.a4 = xyz2pixel(A4,self.fx,self.fy,self.cx,self.cy)
		self.a5 = xyz2pixel(A5,self.fx,self.fy,self.cx,self.cy)
		self.a6 = xyz2pixel(A6,self.fx,self.fy,self.cx,self.cy)
		
		self.m1 = ((self.a1[0]+self.a2[0])//2,self.a1[1])
		self.m2 = ((self.a5[0]+self.a6[0])//2,self.a5[1])
		
		
		self.b1 = xyz2pixel(B1,self.fx,self.fy,self.cx,self.cy)
		self.b2 = xyz2pixel(B2,self.fx,self.fy,self.cx,self.cy)
		self.b3 = xyz2pixel(B3,self.fx,self.fy,self.cx,self.cy)
		self.b4 = xyz2pixel(B4,self.fx,self.fy,self.cx,self.cy)
		self.b5 = xyz2pixel(B5,self.fx,self.fy,self.cx,self.cy)
		self.b6 = xyz2pixel(B6,self.fx,self.fy,self.cx,self.cy)
	
	#空间位置求区域
	def whatArea(self,x,z):
		if(x>=-self.L4/2-self.L5 and x<-self.L4/2): #1,5
			if(z>=self.L1 and z<self.L1+self.L2):
				return 1
			elif(z>=self.L1+self.L2 and z <self.L1+self.L2+self.L3):
				return 5
			else:
				return 0
		elif(x>=-self.L4/2 and x<0): #2,6
			if(z>=self.L1 and z<self.L1+self.L2):
				return 2
			elif(z>=self.L1+self.L2 and z <self.L1+self.L2+self.L3):
				return 6
			else:
				return 0
		elif(x>=0 and x<self.L4/2): #3,7
			if(z>=self.L1 and z<self.L1+self.L2):
				return 3
			elif(z>=self.L1+self.L2 and z <self.L1+self.L2+self.L3):
				return 7
			else:
				return 0
		elif(x>=self.L4/2 and x<self.L4/2+self.L5): #4,8
			if(z>=self.L1 and z<self.L1+self.L2):
				return 4
			elif(z>=self.L1+self.L2 and z <self.L1+self.L2+self.L3):
				return 8
			else:
				return 0
		else:
			return 0
		

	#区域划线
	def drawLine(self,img,color=(0,255,0),w=2):
		cv2.line(img,self.b1,self.b2,color,w)
		cv2.line(img,self.b3,self.b4,color,w)
		cv2.line(img,self.b5,self.b6,color,w)
		
		cv2.line(img,self.a1,self.a5,color,w)
		cv2.line(img,self.m1,self.m2,color,w)
		cv2.line(img,self.a2,self.a6,color,w)
		return img

	def draw(self,img):
		if(self.mask is None):
			size = (self.size[1],self.size[0],3)
			self.mask = np.zeros(size,dtype='uint8')
			self.drawLine(self.mask,(0,0,255))
		img = cv2.addWeighted(img,1.0,self.mask,0.2,0)
		return img

	#捕获鼠标按键
	def openMouseCapture(self,windowName):
		self._pixel2disTable = pixel2disTable(self.size[0],self.size[1],self.height,
											  self.theta,self.fx,self.fy,self.cx,self.cy)

		cv2.setMouseCallback(windowName, self.onMouse)
		
	#地面像素点转空间距离和区域信息
	def onMouse(self,event,x,y,flags,param):
		if(event == cv2.EVENT_LBUTTONDOWN):
			loc = self._pixel2disTable[x,y] #像素求空间位置
			print("像素坐标uv(%4d,%4d)  位置坐标xz(%4.1fm,%4.1fm)" %(x,y,loc[0],loc[1]))
			print("区域： %d\n" %self.whatArea(loc[0],loc[1])) #空间位置求区域


def main():	
	regionDivide = RegionDivide()
	regionDivide.setCameraParams(721.5,721.5,609.5,172.85,(1242,375),1.65,0.0)
	regionDivide.setRegionParams(10,5,5,3,3)
	
	img = cv2.imread("a.png")
	if img is None:
		print("No image!")
		exit()
	
	img = regionDivide.draw(img)
	windowName = "image"
	cv2.namedWindow(windowName,0)
	regionDivide.openMouseCapture(windowName)
	cv2.imshow(windowName,img)
	
	cv2.waitKey()
	

if __name__ == '__main__':
	main()
	pass
	
