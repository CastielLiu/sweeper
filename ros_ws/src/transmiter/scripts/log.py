#!/usr/bin/env python
# coding=UTF-8

import rospy
from can_msgs.msg import FrameArray
from transmiter.msg import AreasInfo

class Logger:
	def __init__(self):
		self.sub_area_info = None
		self.sub_can = None
		self.areas_info = None
		self.file = None
		
	def __del__(self):
		if(self.file is not None):
			self.file.close()
	
	def init(self):
		self.sub_area_info = rospy.Subscriber("sweeper/area_info", AreasInfo, self.areasInfoCallback)
		self.sub_can = rospy.Subscriber("/to_can_topic", FrameArray, self.canFrameArrayCallback)
		self.file = open("log.txt",'w')
		if(self.file is None):
			print("open log file failed!")
			return False
		return True
	
	#1
	def areasInfoCallback(self, msg):
		self.areas_info = msg.infos
	
	#2
	def canFrameArrayCallback(self, msg):
		if(msg.header.frame_id != "ch1"):
			return
		if(self.areas_info is None):
			return;
		
		if(self.file is None):
			print("log file is not opened!")
			return
		
		self.dataCheck(msg.frames[0], self.areas_info)
		
		stamp = rospy.get_rostime().to_sec()
		
		line = "%.3f\t" %(stamp)
		for areaInfo in self.areas_info:
			line += "%d,%d,%d,%d " %(areaInfo.area_id, areaInfo.rubbish_grade, \
									areaInfo.has_person, areaInfo.vegetation_type)
		line += "\t"
		can_frame = msg.frames[0]
		for i in range(can_frame.len):
			line += "0x%02x " %(ord(can_frame.data[i]))
		
		line += "\n"
		
		self.file.write(line)
		print("log: %s" %line)
		
	def dataCheck(self, canFrame, areasInfo):
		ok = True
		
		# 将ascii转换为int  int = ord(ascii)
		datas = [ord(data) for data in canFrame.data]
		
		for i in range(len(areasInfo)):
			areaInfo = areasInfo[i]
			areaId = areaInfo.area_id
			
			has_person = (datas[4] & (1<<areaId-1)) > 0
			if(areaInfo.has_person != has_person):
				print("areaId: %d has_person value false!" %areaId)
				ok = False

			rubbish_grade = (datas[(areaInfo.area_id-1)/2] >> (4*((areaInfo.area_id-1)%2))) & 0xf
			if(areaInfo.rubbish_grade != rubbish_grade):
				print("areaId: %d rubbish_grade value false!" %areaId)
				ok = False

			vegetation_type = (datas[(areaInfo.area_id-1)/4+5] >> 2*((areaInfo.area_id-1)%4)) & 0x03
			
			if(areaInfo.vegetation_type != vegetation_type):
				print("areaId: %d vegetation_type value false!" %areaId)
				ok = False
		return ok

def main():
	rospy.init_node('logger_node', anonymous=True)
	logger = Logger()
	if(logger.init()):
		rospy.spin()

if __name__ == '__main__':
	try:
		main()
	except rospy.ROSInterruptException:
		pass
