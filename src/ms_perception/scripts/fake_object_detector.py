#!/home/msasrock/.virtualenvs/ros-melodic-venv/bin/python3.6
import rospy
import numpy as np
import random
import yaml
from ms_msgs.msg import DetectedObject
from ms_msgs.srv import FindObjects, FindObjectsResponse

class MyRobot:
	def __init__(self, class_ids):
		self.classnames = class_ids
		service_name = rospy.get_param('cctv_service_name', 'cctv_detection')
		self.detector_server = rospy.Service(service_name, FindObjects, self.__detect)
		self.verbose = rospy.get_param("~verbose", 1)
		rospy.loginfo('Welcome to {}'.format(rospy.get_name()))

	def __detect(self, req):
		if self.verbose == 1:
			rospy.loginfo("[{}] Get request for dvr {}, ch {}".format(rospy.get_name(), req.dvr, req.channel))
		
		duration, objects = self.__getObjects(req.image)
		response = FindObjectsResponse(
			req.dvr + req.channel,
			random.randint(0, 1),
			duration,
			objects
		)
		return response

	def __getObjects(self, image):
		objects = []
		N = random.randint(1, 4)
		for index in range(N):
			object_id = random.randint(0, len(self.classnames) - 1)
			obj = DetectedObject()
			obj.box = [
				random.randint(0, image.width / 3 * 2), # x
				random.randint(0, image.height / 2), # y
				random.randint(200, 400), # width
				random.randint(200, 400)  # length
			]
			obj.probability = random.random()
			obj.name = self.classnames[object_id]
			obj.id = object_id
			objects.append(obj)
		return random.random(), objects

def loadConfig():
	config_file = rospy.get_param("~object_descriptions_file", "")
	print("Reading from {}".format(config_file))
	if config_file == "":
		rospy.logwarn("[{}] Shutting down due to invalid config file".format(rospy.get_name()))
		rospy.signal_shutdown()
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
	rospy.init_node('fake_object_detector')
	config_data = loadConfig()
	if config_data is None:
		return

	robot = MyRobot(config_data['class_ids'])
	rospy.spin()

if __name__ == "__main__":
	main()