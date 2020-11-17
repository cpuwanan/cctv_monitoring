#!/home/msasrock/.virtualenvs/ros-melodic-venv/bin/python3.6
import rospy
import numpy as np
import random
import yaml
from ms_msgs.msg import LoadingBayArray


import matplotlib.pyplot as plt

## Helper

class PolicyManager:
	def __init__(self, class_ids, bay_num):
		self.sub = rospy.Subscriber("/all_bays", LoadingBayArray, self.callback)
		self.class_ids = class_ids
		rows = bay_num
		cols = len(class_ids) * 2
		self.heatmap = np.zeros(rows * cols)
		self.heatmap = self.heatmap.reshape((rows, cols))
		self.heatmap.fill(0.5)
		print("Shape: ", self.heatmap.shape)

	def callback(self, data):
		# rospy.loginfo("[{}] {}".format(rospy.get_name(), data))
		start_index1 = 0;
		start_index2 = self.heatmap.shape[1] // 2;
		for bay in data.bays:
			x = bay.id
			# Clear all to zero
			for y in range(len(self.heatmap[x])):
				self.heatmap[x][y] = 0.5
			for id1 in bay.inner_ids:
				self.heatmap[x][id1] = 1
			for id2 in bay.outer_ids:
				self.heatmap[x][start_index2 + id2] = 1

		title = "{}: Seq {}".format(rospy.get_name(), data.header.seq)

		plt.imshow(self.heatmap, cmap='cool', interpolation='nearest')
		plt.title(title)
		plt.yticks(np.arange(0, self.heatmap.shape[0], step=1))
		plt.xticks(np.arange(0, self.heatmap.shape[1], step=1), ("i-plt", "i-fork", "i-wkr", "i-door", "i-cart", "o-plt", "o-fork", "o-wkr", "o-door", "o-cart"), rotation=90)
		plt.grid(True)
		plt.pause(0.1)

## Main

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
	rospy.init_node('fake_policy_manager', anonymous=True)

	config_data = loadConfig()

	if config_data is None:
		rospy.signal_shutdown("Failed loading config file")
		return

	rospy.loginfo("[{}] Loaded class_ids: {}".format(rospy.get_name(), config_data['class_ids']))

	policy_manager = PolicyManager(config_data['class_ids'], 28)
	rospy.spin()

if __name__ == "__main__":
	try:
		main()
	except rospy.ROSInterruptException:
		pass
