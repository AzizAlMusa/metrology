#!/usr/bin/env python
import numpy as np
import pdb
import sys
import rospy
import ros_numpy
import tf2_ros
import tf2_py as tf2
import cv2 as cv
import tf2_geometry_msgs
import tf2_sensor_msgs
from realsense_cam.srv import *
from sensor_msgs.msg import Image, PointCloud2
import sensor_msgs.point_cloud2 as pc2
import geometry_msgs.msg
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import quaternion_from_euler

tf_buffer = None
tf_listener = None
num = 0
box_pcl_pub = None
log_images = False

def hough_lines_orientation(img):

    global num
    kernel = np.ones((3,3), np.uint8)
    original_img = np.copy(img)
    img = cv.erode(img, kernel,iterations=1)    ## Erode to remove some noise where possible

    ## Find remaining contours in image and select the largest to represent our workspace box
    im2, contours, hierarchy = cv.findContours(img, cv.RETR_TREE, cv.CHAIN_APPROX_NONE)
    zimg = np.zeros_like(img)
    c = max(contours, key = cv.contourArea)
    cv.drawContours(zimg, c, -1, color=255, thickness=-1)

    ## Blur box outline to improve hough performance
    dst = zimg
    dst = cv.GaussianBlur(dst, (7,7), 0)
    dst = cv.GaussianBlur(dst, (7,7), 0)
    dst = dst+dst

    ## Copy image to draw lines over it later
    cdstP = cv.cvtColor(dst, cv.COLOR_GRAY2BGR)
    idx = np.nonzero(dst>80)
    min_x = min(idx[1])
    max_x = max(idx[1])
    min_y = min(idx[0])
    max_y = max(idx[0])

    # TODO: Log hough_lines pictures
    ## Find lines using Hough transform, parameters will have to be tweaked extensively.
    linesP = cv.HoughLinesP(dst, rho=20, theta=np.pi / 360, threshold=np.int32((max_x - min_x)), minLineLength = np.int32((max_x - min_x)*.7), maxLineGap=40)

    mean_dst = np.copy(cdstP)

    if linesP is not None:
        lengths = []
        thetas = []
        bottom_l = []
        top_l = []
        left_l = []
        right_l = []
        ## Merge lines together 
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            l_f = np.float32(l)

            ## Calculate slope to assist in categorizing lines
            ## In this case, we can assume that the lines will not be horrifically skewed and will only require
            ## minor adjustments
            slope = np.arctan((l_f[1]-l_f[3])/(l_f[0] - l_f[2]))

            ## These are vertical lines, horizontal/nearly horizontal lines present with a slope much closer to 0
            if np.abs(slope) > 1:
                ## p1 and p2 of the line are on the left half of the image
                if l[0] < img.shape[1]/2 and l[2] < img.shape[1]/2:
                    left_l.append(l)
                    cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (255,0,0), 3, cv.LINE_AA)        # Blue
                    print "blue: ", slope
                else:
                ## p1 and p2 of the line are on the right half of the image
                    right_l.append(l)
                    cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (255,255,255), 3, cv.LINE_AA)    # White
                    print "white: ", slope
            ## These are horizontal lines
            else:
                ## p1 and p2 of the line are on the bottom half of the image
                if l[1] > img.shape[0]/2 and l[3] > img.shape[0]/2:
                    bottom_l.append(l)
                    cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,0,255), 3, cv.LINE_AA)        # Red
                    print "red: ", slope
                ## p1 and p2 of the line are on the top half of the image
                elif l[1] < img.shape[0]/2 and l[3] < img.shape[0]/2:
                    top_l.append(l)
                    cv.line(cdstP, (l[0], l[1]), (l[2], l[3]), (0,255,0), 3, cv.LINE_AA)        # Green
                    print "green: ", slope
        cv.imwrite("all_lines" + str(num) + ".png", cdstP)

        ## Combine the groups of lines into arrays
        bottom_l = np.asarray(bottom_l)
        top_l = np.asarray(top_l)
        right_l = np.asarray(right_l)
        left_l = np.asarray(left_l)

        ## For each array, calculate the average line 
        mean_left = np.array([np.mean(left_l[:,0]), np.min(left_l[:,1]), np.mean(left_l[:,2]), np.max(left_l[:,3])])
        mean_left = np.int32(mean_left)

        mean_right = np.array([np.mean(right_l[:,0]), np.min(right_l[:,1]), np.mean(right_l[:,2]), np.max(right_l[:,3])])
        mean_right = np.int32(mean_right)

        mean_top = np.array([np.min(top_l[:,0]), np.mean(top_l[:,1]), np.max(top_l[:,2]), np.mean(top_l[:,3])])
        mean_top = np.int32(mean_top)

        mean_bottom = np.array([np.min(bottom_l[:,0]), np.mean(bottom_l[:,1]), np.max(bottom_l[:,2]), np.mean(bottom_l[:,3])])
        mean_bottom = np.int32(mean_bottom)

        # mean_left = np.mean(left_l,0, np.int32)
        # mean_top = np.mean(top_l,0,np.int32)
        # mean_bottom = np.mean(bottom_l,0,np.int32)
        # mean_right = np.mean(right_l,0,np.int32)
        print "mean left:\n", mean_left
        print "mean right:\n", mean_right
        print "mean top:\n", mean_top
        print "mean bottom:\n", mean_bottom
        ## Draw these newly calculated lines on the mean_dst image
        cv.line(mean_dst, (mean_bottom[0], mean_bottom[1]), (mean_bottom[2], mean_bottom[3]), (0,0,255), 3, cv.LINE_AA)    # Red
        cv.line(mean_dst, (mean_top[0], mean_top[1]), (mean_top[2], mean_top[3]), (0,255,0), 3, cv.LINE_AA)                # Green
        cv.line(mean_dst, (mean_left[0], mean_left[1]), (mean_left[2], mean_left[3]), (255,0,0), 3, cv.LINE_AA)            # Blue
        cv.line(mean_dst, (mean_right[0], mean_right[1]), (mean_right[2], mean_right[3]), (255,255,255), 3, cv.LINE_AA)        # White

        ## Calculate the slopes of these newly calculated lines
        mean_left = np.float64(mean_left)
        mean_right = np.float64(mean_right)
        mean_top = np.float64(mean_top)
        mean_bottom = np.float64(mean_bottom)
        l_slope = np.arctan((mean_left[3]-mean_left[1])/(mean_left[2] - mean_left[0]))
        r_slope = np.arctan((mean_right[3]-mean_right[1])/(mean_right[2] - mean_right[0]))
        t_slope = np.arctan((mean_top[3]-mean_top[1])/(mean_top[2] - mean_top[0]))
        b_slope = np.arctan((mean_bottom[3]-mean_bottom[1])/(mean_bottom[2] - mean_bottom[0]))
        print mean_bottom
        print "final left: ", np.degrees(l_slope)
        print "final right: ", np.degrees(r_slope)
        print "final top: ", np.degrees(t_slope)
        print "final bottom: ", np.degrees(b_slope)

        mean_left = np.int32(mean_left)
        mean_right = np.int32(mean_right)
        mean_top = np.int32(mean_top)
        mean_bottom = np.int32(mean_bottom)
        ## Annotate each line with its slope
        ## Left
        pos_left = ((mean_left[0] + mean_left[2])/2,(mean_left[1]+mean_left[3])/2)
        pos_left = (pos_left[0]+ 20, pos_left[1])
        print pos_left[1]
        # pos_left[1] = pos_left[1] - 20
        cv.putText(mean_dst,  str(np.rad2deg(l_slope))+" deg", pos_left, cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,0,0), 2, cv.LINE_AA)

        ## Right
        pos_right = ((mean_right[0] + mean_right[2])/2,(mean_right[1]+mean_right[3])/2)
        pos_right = (pos_right[0]+20, pos_right[1])
        print pos_right[1]
        # pos_right[1] = pos_right[1] - 20
        cv.putText(mean_dst,  str(np.rad2deg(r_slope)) + " deg", pos_right, cv.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,255), 2, cv.LINE_AA)

        ## Top
        pos_top = ((mean_top[0] + mean_top[2])/2,(mean_top[1]+mean_top[3])/2)
        pos_top = (pos_top[0], pos_top[1]-20)
        print pos_top[1]
        # pos_top[1] = pos_top[1] - 20
        cv.putText(mean_dst,  str(np.rad2deg(t_slope)) + " deg", pos_top, cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2, cv.LINE_AA)

        ## Bottom
        pos_bottom = ((mean_bottom[0] + mean_bottom[2])/2,(mean_bottom[1]+mean_bottom[3])/2)
        pos_bottom = (pos_bottom[0], pos_bottom[1]+20)
        print pos_bottom[1]
        # pos_bottom[1] = pos_bottom[1] - 20
        cv.putText(mean_dst,  str(np.rad2deg(b_slope)) + " deg", pos_bottom, cv.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 2, cv.LINE_AA)

        cv.imwrite("all_lines" + str(num) + ".png", cdstP)
        cv.imwrite("merged_lines" + str(num) +".png", mean_dst)
        num=num+1
        z_change = b_slope
        x_change = (np.pi/2 - np.abs(l_slope)) + (np.pi/2 - np.abs(r_slope))/2
        print "z_change ", z_change
        print "x_change", x_change
        return (z_change, x_change)
        # return (x_change, y_change, z_change) TODO
        return z_change
def find_workspace_center(req):

    global tf_buffer, tf_listener,num
    print 'Finding workspace center'

    # Initialize CV, TF, and services
    bridge = CvBridge()
    print 'Setting up proxies'
    rospy.wait_for_service('realsense_services/next_rgb')
    rgb_fetch = rospy.ServiceProxy('realsense_services/next_rgb', FetchOneRGB)
    rospy.wait_for_service('realsense_services/next_pointcloud')
    pcl_fetch = rospy.ServiceProxy('realsense_services/next_pointcloud', FetchOnePointCloud)
    print 'Performing image operations'

    # Retrieve and convert images from the realsense
    res = rgb_fetch()
    img = res.rgb
    cv_img = bridge.imgmsg_to_cv2(img, "bgr8")
    
    img_hsv = cv.cvtColor(cv_img, cv.COLOR_BGR2HSV)
    mask1 = cv.inRange(img_hsv, (0,50,20), (5,255,255))
    mask2 = cv.inRange(img_hsv, (175,50,20), (180,255,255))

    mask = cv.bitwise_or(mask1, mask2)
    cropped = cv.bitwise_and(cv_img, cv_img, mask=mask)

    ret, mask_thresh = cv.threshold(mask, 127, 255, 0)
    im2, contours, hierarchy = cv.findContours(mask_thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    c = max(contours, key = cv.contourArea)
    blank = np.zeros(cv_img.shape)
    cv.drawContours(blank, c, -1, (0,255,0), 3)
    idx = np.nonzero(blank>80)
    min_x = min(idx[1])
    max_x = max(idx[1])
    min_y = min(idx[0])
    max_y = max(idx[0])

    poi = ((min_y + max_y)/2, (min_x + max_x)/2)
    blank[poi[0]-5:poi[0]+5,poi[1]-5:poi[1]+5] = 255

    if log_images:
        cv.imwrite("blank" + str(num) + ".png", blank)
        cv.imwrite("cropped" +str(num) + ".png", cropped)
        cv.imwrite("cv_img" +str(num) + ".png", cv_img)
        cv.imwrite("mask"+str(num)+".png",mask)
        cv.imwrite("img_hsv"+str(num)+".png",img_hsv)
    # num=num+1
    print "min_x: %i max_x: %i min_y: %i max_y: %i poi: %i,%i" % (min_x, max_x, min_y, max_y, poi[0], poi[1])
    # Calculate center of box

    pcl_res = pcl_fetch()
    pcl = pcl_res.pointcloud
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pcl, False)    

    # Obtain pointcloud point for the center of the box
    test_point = geometry_msgs.msg.PointStamped()
    test_point.point.x = xyz_array[poi[0]][poi[1]][0]
    test_point.point.y = xyz_array[poi[0]][poi[1]][1]
    test_point.point.z = xyz_array[poi[0]][poi[1]][2]
    test_point.header = pcl.header
    print xyz_array[poi[0]][poi[1]]    # Transform point to base_link coordinates
    transform = tf_buffer.lookup_transform("base_link", test_point.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
    new_point = tf2_geometry_msgs.do_transform_point(test_point, transform)
    print new_point

    angle_changes = hough_lines_orientation(mask)

    return FindWorkspaceCenterResponse(True, new_point, angle_changes[0], angle_changes[1])

def find_box_points(req):

    global tf_buffer, tf_listener, num, box_pcl_pub, log_images

    print 'Finding box points'

    # Initialize CV, TF, and services
    bridge = CvBridge()
    rospy.wait_for_service('realsense_services/next_rgb')
    rgb_fetch = rospy.ServiceProxy('realsense_services/next_rgb', FetchOneRGB)
    rospy.wait_for_service('realsense_services/next_pointcloud')
    pcl_fetch = rospy.ServiceProxy('realsense_services/next_pointcloud', FetchOnePointCloud)

    # Grab the RGB image
    res = rgb_fetch()
    img = res.rgb
    cv_img = bridge.imgmsg_to_cv2(img, "bgr8")

    # Identify tape covering the top of the box
    img_hsv = cv.cvtColor(cv_img, cv.COLOR_BGR2HSV)
    mask1 = cv.inRange(img_hsv, (0,50,20), (5,255,255))
    mask2 = cv.inRange(img_hsv, (175,50,20), (180,255,255))

    mask = cv.bitwise_or(mask1, mask2)
    cropped = cv.bitwise_and(cv_img, cv_img, mask=mask)

    ret, mask_thresh = cv.threshold(mask, 127, 255, 0)

    im2, contours, hierarchy = cv.findContours(mask_thresh, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE)
    c = max(contours, key = cv.contourArea)
    blank = np.zeros(cv_img.shape)
    cv.drawContours(blank, c, -1, (0,255,0), 3)
    idx = np.nonzero(blank>80)
    min_x = min(idx[1])
    max_x = max(idx[1])
    min_y = min(idx[0])
    max_y = max(idx[0])
    poi = ((min_y + max_y)/2, (min_x + max_x)/2)
    blank[poi[0]-5:poi[0]+5,poi[1]-5:poi[1]+5] = 255

    if log_images:
        cv.imwrite("blank" + str(num) + ".png", blank)
        cv.imwrite("cropped" +str(num) + ".png", cropped)
        cv.imwrite("cv_img" +str(num) + ".png", cv_img)
        cv.imwrite("mask"+str(num)+".png",mask)
        cv.imwrite("img_hsv"+str(num)+".png",img_hsv)

    num=num+1
    print "min_x: %i max_x: %i min_y: %i max_y: %i poi: %i,%i" % (min_x, max_x, min_y, max_y, poi[0], poi[1])

    # Retrieve pointcloud and convert to numpy array
    pcl_res = pcl_fetch()
    pcl = pcl_res.pointcloud
    xyz_array = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(pcl, False)
    points = xyz_array[min_y:max_y,min_x:max_x]
    print(points.shape)
    print(points.dtype)
    points = points.astype(np.single)
    print(points.dtype)
    box_points = np.atleast_2d(points)

    ## Find workspace boundaries
    min_point = geometry_msgs.msg.PointStamped()
    max_point = geometry_msgs.msg.PointStamped()
    min_point.point.x = xyz_array[min_y][min_x][0]
    min_point.point.y = xyz_array[min_y][min_x][1]
    min_point.point.z = xyz_array[min_y][min_x][2]

    max_point.point.x = xyz_array[max_y][max_x][0]
    max_point.point.y = xyz_array[max_y][max_x][1]
    max_point.point.z = xyz_array[max_y][max_x][2]

    # print("min_point:", min_point)
    # print('max_point:', max_point)
    
    min_point.header = pcl.header
    transform = tf_buffer.lookup_transform("base_link", min_point.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
    new_min_point = tf2_geometry_msgs.do_transform_point(min_point, transform)
    max_point.header = pcl.header
    transform = tf_buffer.lookup_transform("base_link", max_point.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
    new_max_point = tf2_geometry_msgs.do_transform_point(max_point, transform)

    print("min_point:", new_min_point)
    print('max_point:', new_max_point)

    ## End Workspace boundary finding

    # Take numpy array containing points inside the box and turn it back into a pointcloud for octomap optimization
    box_pcl = PointCloud2()
    
    box_pcl.header = pcl.header
    box_pcl.height = box_points.shape[0]
    box_pcl.width = box_points.shape[1]
    box_pcl.fields = [sensor_msgs.msg.PointField('x',0,sensor_msgs.msg.PointField.FLOAT32,1),
                      sensor_msgs.msg.PointField('y',4,sensor_msgs.msg.PointField.FLOAT32,1),
                      sensor_msgs.msg.PointField('z',8,sensor_msgs.msg.PointField.FLOAT32,1)]
    box_pcl.point_step = 12
    box_pcl.is_bigendian = pcl.is_bigendian
    box_pcl.row_step = box_pcl.point_step*box_points.shape[1]
    box_pcl.is_dense = True
    box_pcl.data = box_points.tostring()
    # pdb.set_trace()

    box_pcl_pub.publish(box_pcl)

    print points.shape
    results = []
    
    boundary_results = []
    boundary_results.append(new_min_point)
    boundary_results.append(new_max_point)


    points[:,1::2] = points[::-1,1::2]
    print "points shape: ", points.shape
    for i in range(points.shape[1]):
        for j in range(points.shape[0]):
            # rospy.loginfo("i: %i j: %i" % (i,j))
            test_point = geometry_msgs.msg.PointStamped()
            test_point.point.x = points[j][i][0]
            test_point.point.y = points[j][i][1]
            test_point.header = pcl.header
            transform = tf_buffer.lookup_transform("base_link", test_point.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            new_point = tf2_geometry_msgs.do_transform_point(test_point, transform)
            results.append(new_point)
            # pdb.set_trace()
            # vacuum = 5.5cm across
            # Check the next few neighbors for the highest height to try to avoid collisions.
            # for k in range(2,30):
            #     # pdb.set_trace()
            #     if ((j*2)-k < 0) or ((i*19)-k)<0:
            #         test_point.point.z = points[j*2][i*19][2]
            #     else:
            #         try:
            #             # print "testing points[", (j*2)-k, ":", (j*2)+k, "][",(i*19)-k,":", (i*19)+k,"][2]"
            #             neighbor_zs = points[(j*2)-k:((j*2)+k),(i*19)-k:(i*19)+k,2]
            #             test_point.point.z = np.max(neighbor_zs)
            #         except IndexError:
            #             # print("Avoiding index errors")
            #             if (k==1):
            #                 test_point.point.z = points[j*2][i*19][2]
            #             # print "got to ", k, " before stopping local search for point ", j*2, ",", i*19
            #             pdb.set_trace()
            #             if (test_point.point.z != points[j*2][i*19][2]):
            #                 # print "changed z level!"
            #                 pass
            #             break
                
            #     # print "got to k=",k

            # if (test_point.point.z != points[j*2][i*19][2]):
            #     # print "changed z level!"
            #     pass
            # # pdb.set_trace()
            # test_point.header = pcl.header
            # transform = tf_buffer.lookup_transform("base_link", test_point.header.frame_id, rospy.Time(0), rospy.Duration(1.0))
            # new_point = tf2_geometry_msgs.do_transform_point(test_point, transform)
            # results.append(new_point)

    return FindBoxPointsResponse(True,box_pcl, boundary_results)

def main():


    global tf_buffer, tf_listener, box_pcl_pub, log_images
    rospy.init_node('im_processing', anonymous=True)
    log_images = rospy.get_param("log_images", False)   # TODO: Test Parameters
    print 'Waiting for realsense_services'
    rospy.wait_for_service('realsense_services/next_rgb')
    print 'Setting up my own services'
    workspace_center_srv = rospy.Service('realsense_services/find_workspace_center', FindWorkspaceCenter, find_workspace_center)
    box_points_srv = rospy.Service('realsense_services/find_box_points', FindBoxPoints, find_box_points)

    # For runtime purposes, we publish a pointcloud consisting of points exclusively inside the work area.
    box_pcl_pub = rospy.Publisher('/D415/depth/color/box_points', PointCloud2, queue_size=5, latch=True)

    print 'Initializing TF buffer'
    tf_buffer = tf2_ros.Buffer()
    tf_listener = tf2_ros.TransformListener(tf_buffer)
    rospy.spin()

if __name__ == "__main__":
    main()