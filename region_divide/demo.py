#coding=utf-8

import cv2
from region_divide import RegionDivide

def main():
	#实例化区域划分工具
	regionDivide = RegionDivide() 
	#配置相机参数,fx,fy,cx,cy,(size),h,theta 
	#h:相机安装高度 theta:相机俯仰角
	#regionDivide.setCameraParams(721.5,721.5,609.5,172.85,(1242,375),1.65,0.0)
	
	#从文件载入相机参数
	if(not regionDivide.loadCameraInfo("1.yaml")):
		return
	
	#设置区域划分参数L1,L2,L3,L4,L5
	regionDivide.setRegionParams(6,10,10,3.5,3)
	
	windowName = "image"
	cv2.namedWindow(windowName,0)
	#开启鼠标捕获功能，通过点击地面可以获得对应的距离
	regionDivide.openMouseCapture(windowName)
	
	img = cv2.imread("a.png")
	if img is None:
		print("No image!")
		exit()
	
	#在图片上绘制区域划分线段
	#img = regionDivide.drawLine(img)
	img = regionDivide.draw(img)
	
	cv2.imshow(windowName,img)
	cv2.imwrite("b.png",img)
	
	cv2.waitKey()

if __name__ == '__main__':
	main()
	
	
	
	

	
