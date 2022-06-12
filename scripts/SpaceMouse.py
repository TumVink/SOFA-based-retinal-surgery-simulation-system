import sys
import Sofa
import spacenavigator
import time
from multiprocessing.connection import Client
from array import array

address = ('localhost', 6000)
conn = Client(address, authkey='secret password')


class KeyboardControl(Sofa.PythonScriptController):
	# called once graph is created, to init some stuff...
	def initGraph(self,node):
		print 'initGraph called (python side)'
		self.MechanicalState = node.getObject('DOFs')
		return 0
	 
	 
	# key and mouse events; use this to add some user interaction to your scripts 
	# def onKeyPressed(self,k):
	#
	# 	# free_position is a scalar array : [tx,ty,tz,rx,ry,rz,rw]
	# 	free_position=self.MechanicalState.free_position
	#
	# 	# translation speed
	# 	speed = 0.1
	#
	# 	# UP key : front
	# 	if k == "B":
	# 		free_position[0][2] += speed
	# 	if ord(k)==19:
	# 		free_position[0][2]+=speed
	# 	# DOWN key : rear
	# 	if ord(k)==21:
	# 		free_position[0][2]-=speed
	# 	# LEFT key : left
	# 	if ord(k)==18:
	# 		free_position[0][0]-=speed
	# 	# RIGHT key : right
	# 	if ord(k)==20:
	# 		free_position[0][0]+=speed
	# 	# PAGEUP key : up
	# 	if ord(k)==22:
	# 		free_position[0][1]-=speed
	# 	# PAGEDN key : down
	# 	if ord(k)==23:
	# 		free_position[0][1]+=speed
	#
	# 	self.MechanicalState.free_position=free_position
	# 	return 0

	def onBeginAnimationStep(self, dt):
		arr = conn.recv()
		#t = time.time()*1000

		self.scale = 10
		# force = self.CFF.findData('totalForce').value
		# force[0][0] = force[0][0] + arr[0]
		# force[0][1] = force[0][1] + arr[1]
		# force[0][2] = force[0][2] + arr[2]
		# force[0][3] = force[0][3] + arr[3]
		# force[0][4] = force[0][4] + arr[4]
		# force[0][5] = force[0][5] + arr[5]
		#force = self.Pose.findData('velocity').value
		#self.Pose.findData('velocity').value = force
		free_position = self.MechanicalState.position
		free_position[0][0] = free_position[0][0] + arr[0]/self.scale
		free_position[0][1] = free_position[0][1] + arr[1]/self.scale
		free_position[0][2] = free_position[0][2] + arr[2]/self.scale
		free_position[0][3] = free_position[0][3] + arr[3]/self.scale
		free_position[0][4] = free_position[0][4] + arr[4]/self.scale
		free_position[0][5] = free_position[0][5] + arr[5]/self.scale

		ms = t-arr[6]
		print ms
		self.MechanicalState.position = free_position

		#print self.MechanicalState.position
		return 0
	 
	 
