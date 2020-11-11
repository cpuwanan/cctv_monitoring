#!/home/msasrock/.virtualenvs/ros-melodic-venv/bin/python3.6
import rospy
import numpy as np
import random
from ms_msgs.msg import DetectedObject
from ms_msgs.srv import FindObjects, FindObjectsResponse

class MyRobot:
	def __init__(self):
		self.classnames = ["pallet", "forklift", "worker", "door", "cart"]
		service_name = rospy.get_param('cctv_service_name', 'cctv_detection')
		self.detector_server = rospy.Service(service_name, FindObjects, self.__detect)
		rospy.loginfo('Welcome to {}'.format(rospy.get_name()))

	def __detect(self, req):
		rospy.loginfo("[{}] Get request for dvr {}, ch {}".format(rospy.get_name(), req.dvr, req.channel))
		duration, objects = self.__getObjects(req.image)
		response = FindObjectsResponse(
			req.dvr + req.channel,
			random.randint(0, 2),
			duration,
			objects
		)
		return response

	def __getObjects(self, image):
		objects = []
		N = random.randint(1, 4)
		for index in range(N):
			obj = DetectedObject()
			obj.box = [
				random.randint(0, image.width / 3 * 2), # x
				random.randint(0, image.height / 2), # y
				random.randint(200, 400), # width
				random.randint(200, 400)  # length
			]
			obj.probability = random.random()
			obj.name = self.classnames[random.randint(0, len(self.classnames) - 1)]
			objects.append(obj)
		return random.random(), objects

def main():
	rospy.init_node('fake_object_detector')
	robot = MyRobot()
	rospy.spin()

if __name__ == "__main__":
	main()