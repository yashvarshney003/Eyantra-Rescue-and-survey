#***********************************************************
#Due to some technical problem, monitor.pyc lagged at the runtime, we tried everything posible but nothing could be done also arduino could not ne connected
#


#!/usr/bin/env python
from __future__ import print_function
import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from survey_and_rescue.msg import *
from cv_bridge import CvBridge, CvBridgeError
import random
import pickle
import imutils
import copy


class sr_determine_colors():

	def __init__(self):
		
		self.check_dictionary = {
							'A1':"",'A2':"",'A3':"",'A4':"",'A5':"",'A6':"",'B1':"",'B2':"",'B3':"",'B4':"",'B5':"",'B6':"",'C1':"",'C2':"",'C3':"",'C4':"",'C5':"",'C6':"",
								'D1':"",'D2':"",'D3':"",'D4':"",'D5':"",'D6':"",'E1':"",'E2':"",'E3':"",'E4':"",'E5':"",'E6':"",'F1':"",'F2':"",'F3':"",'F4':"",'F5':"",'F6':""
										}

		self.detect_info_msg = SRInfo()
		self.bridge = CvBridge()
		self.detect_pub = rospy.Publisher("/detection_info",SRInfo,queue_size=10) 
 		self.image_sub = rospy.Subscriber("/usb_cam/image_rect_color",Image,self.image_callback)
 		self.serviced_sub = rospy.Subscriber('/serviced_info',SRInfo,self.serviced_callback)          # This is giving a feedback generated by monitor.pyc
 																									  # that wether the service was succfull or a failure in
 																									  # this format location: "A2"
																									  #				info: "FAILURE"
		


		self.msg = SRInfo()
		self.msg.location=""
		self.msg.info = ""
		
		self.area = 0
		self.b = 0
		self.cx = 0
		self.cy = 0
		self.M = 0

		self.lower_red = np.array([122,114,000]) #np.array([116,41,0])
		self.upper_red = np.array([255,255,255]) #np.array([255,255,255])

		self.lower_green = np.array([26,76,40]) #np.array([0,92,0])
		self.upper_green = np.array([80,255,255]) #np.array([93,255,255])

		self.lower_blue = np.array([101,230,070]) #np.array([65,188,54])
		self.upper_blue = np.array([149,255,255]) #np.array([134,255,255])

		self.img = np.array([0,0,0])
		self.hsv = self.img

		self.mask_red = 0
		self.mask_green = 0
		self.mask_blue = 0

		self.cnts_red = 0
		self.cnts_green = 0
		self.cnts_blue = 0

		self.cnts2_red = []
		self.cnts2_green = []
		self.cnts2_blue = []

		self.event_red= ""
		self.event_green= ""
		self.event_blue= ""

		self.rect_list = []
 		

	def load_rois(self, file_path = 'rect_info.pkl'):
		try:

			with open("contours2.pickle", 'rb') as input:
   				self.rect_list = pickle.load(input)
   		except IOError, ValueError:
			print("File doesn't exist or is corrupted")


 	def image_callback(self, data):
 		try:
 			self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
 			self.hsv = cv2.cvtColor(self.img,cv2.COLOR_BGR2HSV)
 			cv2.waitKey(10)
			
 		except CvBridgeError as e:
 			print(e)


 	def serviced_callback(self, msg):
 		pass
 	
	def find_color_contour_centers(self):

		

		self.mask_red = cv2.inRange(self.hsv, self.lower_red, self.upper_red)
		self.mask_green = cv2.inRange(self.hsv, self.lower_green, self.upper_green)
		self.mask_blue = cv2.inRange(self.hsv, self.lower_blue, self.upper_blue)

		_, self.cnts_red,_ = cv2.findContours(self.mask_red,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		_, self.cnts_green,_ = cv2.findContours(self.mask_green,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
		_, self.cnts_blue,_ = cv2.findContours(self.mask_blue,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)

		#****************************************


		for self.cnts_red in self.cnts_red:
			self.area = cv2.contourArea(self.cnts_red)
			if (self.area >200 and self.area <1000 ):
				self.cnts2_red.append(self.cnts_red)
				
		for self.b in self.cnts2_red:
			self.M =cv2.moments(self.b)
			self.cx = int (self.M["m10"]/self.M["m00"])
			self.cy = int (self.M["m01"]/self.M["m00"])
			cv2.drawContours(self.img,self.cnts2_red,-1,(0,0,255),1)
			cv2.circle(self.img,(self.cx,self.cy),1,(0,0,255),-1)
		
			cv2.line(self.img,(0,0),(self.cx,self.cy),(0,255,255),3)
			self.cnts2_red = []
			for self.x in self.rect_list:
			
				self.dist = cv2.pointPolygonTest(self.rect_list[self.x],(self.cx,self.cy),False)
				
				if(self.dist==1 or self.dist==0):
						self.event_red ="RESCUE"
						if(self.event_red == self.check_dictionary[self.x]):
							pass
						else:
							self.check_dictionary[self.x]=self.event_red
							

							self.msg.location= self.x
							self.msg.info = self.check_dictionary[self.x]
						
							self.detect_pub.publish(self.msg)
				
			

			


		for self.cnts_green in self.cnts_green:
			self.area = cv2.contourArea(self.cnts_green)
			if (self.area >500 and self.area <2500 ):
				self.cnts2_green.append(self.cnts_green)
		for self.b in self.cnts2_green:
			self.M =cv2.moments(self.b)
			self.cx = int (self.M["m10"]/self.M["m00"])
			self.cy = int (self.M["m01"]/self.M["m00"])
			cv2.drawContours(self.img,self.cnts2_green,-1,(0,255,0),1)
			cv2.circle(self.img,(self.cx,self.cy),1,(0,0,255),-1)
			cv2.line(self.img,(0,0),(self.cx,self.cy),(0,255,0),3)
			self.cnts2_green = []

			for self.x in self.rect_list:

				self.dist = cv2.pointPolygonTest(self.rect_list[self.x],(self.cx,self.cy),False)
			
				if(self.dist==1 or self.dist==0):
						self.event_green ="FOOD"
						if(self.event_green == self.check_dictionary[self.x]):
							pass
						else:
							self.check_dictionary[self.x]=self.event_green
							
							self.msg.location= self.x
							self.msg.info = self.check_dictionary[self.x]
							self.detect_pub.publish(self.msg)
				

			


		for self.cnts_blue in self.cnts_blue:
			self.area = cv2.contourArea(self.cnts_blue)
			#print("cnts Blue ::",self.area)
			if (self.area >300 and self.area <1550):
				self.cnts2_blue.append(self.cnts_blue)
		for self.b in self.cnts2_blue:
			self.M =cv2.moments(self.b)
			self.cx = int (self.M["m10"]/self.M["m00"])
			self.cy = int (self.M["m01"]/self.M["m00"])
			cv2.drawContours(self.img,self.cnts2_blue,-1,(255,0,0),1)
			cv2.circle(self.img,(self.cx,self.cy),1,(0,0,255),-1)
			
			cv2.line(self.img,(0,0),(self.cx,self.cy),(255,0,0),3)
			for self.x in self.rect_list:
				self.dist = cv2.pointPolygonTest(self.rect_list[self.x],(self.cx,self.cy),False)				
				if(self.dist==1 or self.dist==0):
						self.event_blue ="Medicine"
						if(self.event_blue == self.check_dictionary[self.x]):
							pass
						else:
							self.check_dictionary[self.x]=self.event_blue
							
							self.msg.location= self.x
							self.msg.info = self.check_dictionary[self.x]
							
							self.detect_pub.publish(self.msg)
				
		self.cnts2_blue = []
				

			


		#****************************************

	def check_whether_lit(self):
		pass


def main(args):
	
	try:
		rospy.init_node('sr_beacon_detector', anonymous=False)
		s = sr_determine_colors()
		'''You may choose a suitable rate to run the node at.
		Essentially, you will be proceesing that many number of frames per second.
		Since in our case, the max fps is 30, increasing the Rate beyond that
		will just lead to instances where the same frame is processed multiple times.'''
		rate = rospy.Rate(10)
		s.load_rois()
		while s.img is None:
			print("Can't get the image")
	except KeyboardInterrupt:
		cv2.destroyAllWindows()
	while not rospy.is_shutdown():
		try:
			s.find_color_contour_centers()
			s.check_whether_lit()
			rate.sleep()
		except KeyboardInterrupt:
			cv2.destroyAllWindows()
		rospy.sleep(1)

if __name__ == '__main__':
    main(sys.argv)

 