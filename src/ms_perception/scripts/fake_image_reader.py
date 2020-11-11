#!/home/msasrock/.virtualenvs/ros-melodic-venv/bin/python3.6
import rospy
import numpy as np
import random
from ms_msgs.srv import FindObjects
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge, CvBridgeError

MyCvBridge = CvBridge()

def callService(service, image_cv2, dvr, channel):
	image_msg = MyCvBridge.cv2_to_imgmsg(image_cv2, encoding="passthrough")
	response = service(image_msg, dvr, channel)
	rospy.loginfo("[{}] {}".format(rospy.get_name(), response))
	return response

def drawObjects(image, objects):
	for obj in objects:
		pt1 = (obj.box[0], obj.box[1])
		pt2 = (obj.box[0] + obj.box[2], obj.box[1] + obj.box[3])
		cv2.rectangle(image, pt1, pt2, (0, 255, 0), 1)
		label = "{} {:.3f}%".format(obj.name, obj.probability)
		image = cv2.putText(image, label, (obj.box[0], obj.box[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 1)

	return image

def main():
	rospy.init_node('fake_image_reader', anonymous=True)
	service_name = rospy.get_param('cctv_service_name', 'cctv_detection')
	rospy.loginfo("[{}] Set service name: {}".format(rospy.get_name(), service_name))
	rospy.wait_for_service(service_name)
	try:
		find_objects = rospy.ServiceProxy(service_name, FindObjects)
		rate = rospy.Rate(1) # 10hz
		img = cv2.imread('/mnt/mydata/colgate_projects/data/dvr3_random/XVR_ch1_20201022124306_T.jpg', cv2.IMREAD_COLOR)
	
		start_time = rospy.Time.now()

		while not rospy.is_shutdown():
			
			# Request
			dvr = random.randint(0, 5)
			channel = random.randint(10, 20)
			elapsed = (rospy.Time.now() - start_time)
			
			# Response
			img_out = cv2.putText(img.copy(), "[{} ns]".format(elapsed), (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
			response = callService(find_objects, img_out, dvr, channel)
			
			# Show outputs on image labels
			img_out = cv2.putText(img_out.copy(), "(Request)  dvr: {}, ch: {}".format(dvr, channel), (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 1)
			img_out = cv2.putText(img_out.copy(), "(Response) type: {}, duration: {:.3f}, objects_size: {}".format(response.type, response.duration, len(response.objects)), (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0), 1)

			# Draw detected objects
			drawObjects(img_out, response.objects)

			cv2.imshow('cctv', img_out)
			cv2.waitKey(3)

			rospy.loginfo("Waiting {} sec".format(elapsed))
			rate.sleep()
		cv2.destroyAllWindows()
	except rospy.ServiceException as exp:
		print("Service call '{}' failed: {}".format(service_name, exp))

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass