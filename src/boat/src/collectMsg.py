#!/usr/bin/env python3
import rospy
import paho.mqtt.client as mqtt
import struct
import socket
from sensor_msgs.msg import NavSatFix
import string 


class GPS():
	def __init__(self):
		self.address = ('192.168.0.11', 4004)
		self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
		self.latitude = 0.0
		self.longitude = 0.0
		self.heading = 0.0
		self.pub = rospy.Publisher('/unionstrong/gpfpd', NavSatFix, queue_size=1)
		rospy.init_node("talker", anonymous=True)
	
	def on_connect(self):
		self.sock.connect(self.address)
		print("connect succsee")

	def on_message(self):
		while True:
			data = self.sock.recv(1024)
			data=data.decode("utf-8")
			data.replace("\x00","")
			print(data)
			msg = data.split(",")
			#print(msg)
			if len(msg)==18 and '\x00' not in data:
				current_fix = NavSatFix()
				current_fix.latitude = float(msg[6])
				current_fix.longitude = float(msg[7])
				current_fix.position_covariance[1] = float(msg[3])
				print("latitude ", current_fix.latitude)
				print("longitude ", current_fix.longitude)
				print("heading ", current_fix.position_covariance[1])
				self.pub.publish(current_fix)
				print("finish publish")


	def worker(self):
		self.on_connect()
		self.on_message()

gps = GPS()
gps.worker()

# class GPS():
# 	def __init__(self):
# 		self.flag1 = False
# 		self.flag2 = False
# 		self.latitude = 0.0
# 		self.longitude = 0.0
# 		self.heading = 0.0
# 		self.client = mqtt.Client()
# 		self.pub = rospy.Publisher('/unionstrong/gpfpd', NavSatFix, queue_size=1)
# 		rospy.init_node("talker", anonymous=True)
	
# 	def on_connect(self, client, userdata, flags, rc):
# 		self.client.subscribe("/gps")
# 		self.client.subscribe("/hdt")
# 		print("connect succsee")

# 	def on_message(self, client, userdata, msg):
# 		#print("recevie")
# 		if msg.topic == "/gps":
# 			print("gps")
# 			info = struct.unpack('>dd',msg.payload)
# 			self.longitude = float(info[1])
# 			self.latitude = float(info[0])
# 			self.flag1 = True
# 			#self.flag2 = True

# 		if msg.topic == "/hdt":
# 			print("hdt")
# 			info = struct.unpack('>f', msg.payload)
# 			self.heading = float(info[0])
# 			self.flag2 = True
# 		#print(self.flag1," ",self.flag2)
# 		if self.flag1 and self.flag2:
# 			self.flag1 = False
# 			self.flag2 = False
# 			current_fix = NavSatFix()
# 			current_fix.latitude = self.latitude
# 			current_fix.longitude = self.longitude
# 			current_fix.position_covariance[1] = self.heading
# 			print("latitude ", self.latitude)
# 			print("longitude ", self.longitude)
# 			print("heading ", self.heading)
# 			self.pub.publish(current_fix)
# 			print("finish publish")


# 	def worker(self):
# 		self.client.on_connect = self.on_connect
# 		self.client.on_message = self.on_message
# 		self.client.connect("192.168.0.11", 4004, 600)
# 		self.client.loop_forever()

# gps = GPS()
# gps.worker()
