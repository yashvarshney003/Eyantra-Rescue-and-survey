'''
* Team id: 7415
*Author list : Yash varshney ,  Aman Tyagi and  Anurag Saxena 
* file name : 
* functions name : 
* Global variables : All variables  are global
*Logic  : WE have clicked a image  of  flex and then we have apllied image processing functions to find contours and save rois in pickle file
'''
import cv2
import numpy as np 
import imutils
from imutils import contours
import json
import pickle
# empty dictionary
contours2=[]
# labels list
labels_list=['A1','B1','C1','D1','E1','F1','A2','B2','C2','D2','E2','F2','A3','B3','C3','D3','E3','F3','A4','B4','C4','D4','E4','F4','A5','B5','C5','D5','E5','F5','A6','B6','C6','D6','E6','F6']
img = cv2.imread('final.png',1)# reading image from file
contours_label ={}
#print (img.shape)
#print(img.size)
#print(img.dtype)
b_img,g_img,r_img=cv2.split(img)# splitting the image  inti three components
img = cv2.merge((b,g,r))#  merging again 

gaussian_img = cv2.GaussianBlur(g,(3,3),0)# applying gaussian blur on green as pixels are varying so much  in rois and outside it 

return_value,masked_image =cv2.threshold(g,85,255,cv2.THRESH_BINARY_INV)# apllying threshold  value which are  in  below become  white   and which are above of become of max value specified
var1,contours,hierarchy=cv2.findContours(masked_image  ,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)# finding contours of rois i,e  cellls

for contours in contours:# iterating over contours and sorting contours according to area and storing contours which are in range and transferring inti another numoy list
	
	area = cv2.contourArea(contours)
	print("cnts ::",area)
	if (area >1400 and area <4999 ):
		contours2.append(contours)
		#x,y,w,h = cv2.boundingRect(cnts)
		#cv2.rectangle(img, (x,y), (x+w, y+h), (255,0,0),2)
contours=contours2
(contours2, boundingBoxes) = contours.sort_contours(contours2, method="left-to-right") # sorting contours left to right 
(contours2, boundingBoxes) = contours.sort_contours(contours2, method="top-to-bottom")  # again sorting contours top-to-botttom

i=0

for in contours2:
	print(i)
	# compute the center of the contour
	x_coordinate,y_coordinate,width,height = cv2.boundingRect(b)# making rectangles  nd contours
	M = cv2.moments(b)
	x_coordinate_center = int(M["m10"] / M["m00"])# finding centers
	y_coordinate_center = int(M["m01"] / M["m00"])
	contours_label[labels_list[i]] = b
 
	# draw the contour and center of the shape on the image
	cv2.rectangle(img, (x,y), (x+w, y+h), (255,0,0),2)
	#cv2.circle(img, (cX, cY), 7, (255, 255, 255), -1)# labelling them 
	cv2.putText(img,labelling_dictionary[i], (x_coordinate_center -15 , y_coordinate_center+5),cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
	i=i+1
#for (i, c) in enumerate(cnts2):
		#img = contours.label_contour(img, c, i, color=(240, 0, 159))
#cv2.imshow("left-to-right",img)
pickle_out = open("contours.pickle","wb")# open a file 
pickle.dump(contours_label,pickle_out)# dumping a dictionary  into pickle file 
pickle_out.close()


#img = cv2.drawContours(img, cnts2, -1, (0,255,0), 1)

#cv2.imshow("green threshold",mask)
cv2.imshow("processed",img)
#cv2.imshow('red',r)
cv2.waitKey(0)
cv2.destroyAllWndows()
