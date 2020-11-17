#!/home/msasrock/.virtualenvs/ros-melodic-venv/bin/python3.6
import rospy
import numpy as np
import random
import yaml

from ms_msgs.msg import LoadingBay, LoadingBayArray, RectangleInt
from ms_msgs.srv import FindObjects
from sensor_msgs.msg import Image

import threading
import cv2
from cv_bridge import CvBridge, CvBridgeError

MyCvBridge = CvBridge()

### Parallel Threading functions

class StatusManager(threading.Thread):
	def __init__(self, pub, update_freq, verbose):
		self.verbose = verbose
		self.pub = pub
		self.delay = 1.0 / update_freq
		rospy.loginfo('[{}] Set update delay: {:.3f}'.format(rospy.get_name(), self.delay))
		self.all_bays = []
		self.event = threading.Event()
		super(StatusManager, self).__init__()

	def update(self, response, confidence_threshold):
		index = -1
		if len(self.all_bays) > 0:
			for k in range(len(self.all_bays)):
				if response.id == self.all_bays[k].id:
					index = k
					continue
		
		bay = LoadingBay()
		bay.id = response.id
		bay.inner_items = []
		bay.outer_items = []
		bay.inner_ids = []
		bay.outer_ids = []

		for obj in response.objects:
			# Only select a detected item with high probability
			if obj.probability > confidence_threshold:
				if response.type == 0:
					# inner image
					bay.inner_items.append(RectangleInt(obj.box))
					bay.inner_ids.append(obj.id)
				elif response.type == 1:
					# outer image
					bay.outer_items.append(RectangleInt(obj.box))
					bay.outer_ids.append(obj.id)

		if index == -1:
			if self.verbose == 1:
				rospy.loginfo("Add new item: ", response.id)
			self.all_bays.append(bay)
		else:
			if self.verbose == 1:
				rospy.loginfo("Update existing item: ", response.id)
			prev_bay = self.all_bays[index]
			if response.type == 0:
				# update only inner bay
				self.all_bays[index].inner_items = bay.inner_items
				self.all_bays[index].inner_ids = bay.inner_ids
			elif response.type == 1:
				# update only outer bay
				self.all_bays[index].outer_items = bay.outer_items
				self.all_bays[index].outer_ids = bay.outer_ids

	def run(self):
		while not self.event.wait(self.delay) and not rospy.is_shutdown():
			msg = LoadingBayArray()
			msg.header.frame_id = rospy.get_name()
			msg.header.stamp = rospy.Time.now()
			msg.bays = self.all_bays
			self.pub.publish(msg)
		

### Main & subroutines

def callService(service, image_cv2, dvr, channel):
	image_msg = MyCvBridge.cv2_to_imgmsg(image_cv2, encoding="passthrough")
	response = service(image_msg, dvr, channel)
	rospy.loginfo_once("[{}] {}".format(rospy.get_name(), response))
	return response

def drawObjects(image, objects, class_ids, confidence_threshold):
	for obj in objects:
		color = (0, 0, 255)
		thickness = 1
		if obj.probability > confidence_threshold:
			color = (0, 255, 0)
			thickness = 2

		pt1 = (obj.box[0], obj.box[1])
		pt2 = (obj.box[0] + obj.box[2], obj.box[1] + obj.box[3])
		cv2.rectangle(image, pt1, pt2, color, thickness)

		label = "undefined"
		if obj.id >= 0 and obj.id < len(class_ids):
			label = "{} {:.3f}%".format(class_ids[obj.id], obj.probability)
	
		image = cv2.putText(image, label, (obj.box[0], obj.box[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1)
	return image

def test():
	all_bays = []
	for k in range(5):
		bay = LoadingBay()
		bay.id = random.randint(0, 10)
		bay.inner_items = []
		bay.outer_items = []
		# for each location [inner-door, outer-door]
		inner = []
		outer = []
		for i in range(2):
			for j in range(random.randint(1, 3)):
				box = RectangleInt([5, 5, random.randint(100, 200), random.randint(100, 200)])
				print(box)
				if i == 0:
					bay.inner_items.append(box)
				elif i == 1:
					bay.outer_items.append(box)
		all_bays.append(bay)

	pub = rospy.Publisher("bays", LoadingBayArray, queue_size=1)
	rate = rospy.Rate(1) # 10hz
	while not rospy.is_shutdown():
		msg = LoadingBayArray()
		msg.header.frame_id = rospy.get_name()
		msg.header.stamp = rospy.Time.now()
		msg.bays = all_bays
		pub.publish(msg)
		print(msg)
		rate.sleep()

def loadConfig():
	config_file = rospy.get_param("~object_descriptions_file", "")
	print("Reading from {}".format(config_file))
	if config_file == "":
		rospy.logwarn("[{}] Shutting down due to invalid config file".format(rospy.get_name()))
		return None

	config_data = None
	with open(config_file) as file:
		config_data = yaml.load(file, Loader=yaml.FullLoader)

	proceed = True
	if config_data is None:
		proceed = False

	if config_data['class_ids'] is None:
		proceed = False

	if not proceed:
		rospy.logwarn("[{}] Shutting down due to invalid config data: {}".format(rospy.get_name()), config_data)
		return None

	return config_data

def main():
	rospy.init_node('fake_image_reader', anonymous=True)

	config_data = loadConfig()

	if config_data is None:
		rospy.signal_shutdown("Failed loading config file")
		return

	rospy.loginfo("[{}] Loaded class_ids: {}".format(rospy.get_name(), config_data['class_ids']))

	## Test msg publishing
	#test()
	#return

	verbose = rospy.get_param('~verbose', 1)
	bay_array_pub = rospy.Publisher("all_bays", LoadingBayArray, queue_size=1)
	service_name = rospy.get_param('cctv_service_name', 'cctv_detection')

	confidence_threshold = rospy.get_param("confidence_threshold", 0.5)
	status_update_freq = rospy.get_param("status_update_freq", 0.5)
	
	manager = StatusManager(bay_array_pub, status_update_freq, verbose)
	manager.start()

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
			manager.update(response, confidence_threshold)
			drawObjects(img_out, response.objects, config_data['class_ids'], confidence_threshold)

			cv2.imshow('cctv', img_out)
			cv2.waitKey(3)

			rospy.loginfo_once("Waiting {} sec".format(elapsed))
			rate.sleep()
		cv2.destroyAllWindows()
	except rospy.ServiceException as exp:
		print("Service call '{}' failed: {}".format(service_name, exp))

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
