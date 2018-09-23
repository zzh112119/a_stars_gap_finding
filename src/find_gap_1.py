#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Vector3
from a_stars_gap_finding.msg import gaps

import math
import numpy as np
from time import time
from sklearn.cluster import DBSCAN

found_gaps = gaps()
gap_center = Vector3()  
dist_fil_ranges=[]
dist_fil_angles=[]

# Callback that receives LIDAR data on the /scan topic.
# data: the LIDAR data, published as sensor_msgs::LaserScan
def scan_callback(data):
    found_gaps.widths = []
    found_gaps.depths = []
    found_gaps.angles = []


    ranges = np.asarray(data.ranges)
    angles = np.arange(data.angle_min, data.angle_max, data.angle_increment)
    dist_index = np.where((0.1 < ranges) &  (ranges< 5))
    dist_fil_ranges = ranges[dist_index]
    dist_fil_angles = angles[dist_index] 
    start = time()
    features = np.column_stack((dist_fil_angles, dist_fil_ranges))


#    print(features.shape)
    predictions = DBSCAN(eps=0.3, min_samples=2).fit(features).labels_
    
    #Find the number of separate regions found by seeing how many different numbers there are

    obstacleIndices = set(predictions) #Added

    #print(obstacleIndices)
    #print(len(obstacleIndices))
    for obstacle in range(0,len(obstacleIndices)-1): 
        start_gap = np.argmax(np.where(predictions == obstacle)[0]) #Changed 
        end_gap = np.argmin(np.where(predictions == obstacle+1)[0]) #Changed
        
        width = np.abs(angles[end_gap] - angles[start_gap])
        
        # Find center
        centerIndex = (start_gap+end_gap)/2
        center_range = ranges[centerIndex]
        center_angle = angles[centerIndex]

      
        #add width
        found_gaps.widths.append(width)
        found_gaps.depths.append(center_range)
        found_gaps.angles.append(center_angle)

	#Now, with all of the found_gaps determine the widest one
	largestGap = np.argmax(found_gaps.widths)
	gap_center_ang = found_gaps.angles[largestGap]
    center_x = center_range * math.cos(gap_center_ang)
    center_y = center_range * math.sin(gap_center_ang)
    gap_center.x=center_x
    gap_center.y=center_y

    #gap_center= Vector3(center_x, center_y, 0)



def find_gap():
    rospy.init_node('find_gap', anonymous=True)
    rospy.Subscriber('scan', LaserScan, scan_callback)
    gaps_pub = rospy.Publisher('lidar_gap', gaps, queue_size=1000)
    center_pub = rospy.Publisher('gap_center', Vector3, queue_size=1000)
    rate = rospy.Rate(10) # 10hz
    print("The code is rolling!")
    while not rospy.is_shutdown():
        gaps_pub.publish(found_gaps)
        center_pub.publish(gap_center)
        rate.sleep()

if __name__ == '__main__':
    try:
        find_gap()
    except rospy.ROSInterruptException:
        pass
