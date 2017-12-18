import triad_openvr
import socket
import time
import os, sys

## Initializing Global Variables
TCP_IP = '128.46.125.46' # The static IP of Ubuntu computer
TCP_PORT = 5000 # Both server and client should have a common IP and Port
BUFFER_SIZE = 1024 # in bytes. 1 charecter is one byte.
INITIAL_MESSAGE = 'Handshake'

## Initializing Global VIVE variables
left_controller = 'controller_1'
right_controller = 'controller_2'
left_controller_id = 3
right_controller_id = 4

class Client():
	def __init__(self):
		self.sock = socket.socket()
		self.sock.connect((TCP_IP, TCP_PORT))
		self.sock.send(INITIAL_MESSAGE)
		self.connect_status = False
		data = self.sock.recv(32) # Blocking call
		if data:
			print('Handshake successfull ! ! !')
			self.connect_status = True
    
	def send_data(self, data):	
		if self.connect_status:
			self.sock.send(data)
			return bool(self.sock.recv(32))
		else:
			return None

class VR():
	def __init__(self):
		self.vr = triad_openvr.triad_openvr()
		self.vr.print_discovered_objects()   

	def get_controller_data_backup(self):
		try:
			left_data = self.vr.devices[left_controller].get_pose_euler()
			right_data = self.vr.devices[right_controller].get_pose_euler()

			_, left_state_info = self.vr.vr.getControllerState(left_controller_id)
			left_state = [left_state_info.ulButtonPressed, left_state_info.ulButtonTouched, left_state_info.unPacketNum]
			
			_, right_state_info = self.vr.vr.getControllerState(right_controller_id)
			right_state = [right_state_info.ulButtonPressed, right_state_info.ulButtonTouched, right_state_info.unPacketNum]
		except Exception as exp:
			print('Please turn on the controllers. They are off.')
			print(exp)
			left_data, left_state, right_data, right_state = False, False, False, False
		return left_data, left_state, right_data, right_state

	def get_controller_data(self):
		left_data = self.vr.devices[left_controller].get_pose_euler()
		right_data = self.vr.devices[right_controller].get_pose_euler()

		_, left_state_info = self.vr.vr.getControllerState(left_controller_id)
		left_state = [left_state_info.ulButtonPressed, left_state_info.ulButtonTouched, left_state_info.unPacketNum]
		
		_, right_state_info = self.vr.vr.getControllerState(right_controller_id)
		right_state = [right_state_info.ulButtonPressed, right_state_info.ulButtonTouched, right_state_info.unPacketNum]
		return left_data, left_state, right_data, right_state

##### ---- Main ---- #####
while True:
	try:
		client = Client()
		vr = VR()
		while True:
			if not client.connect_status:
				print('Not connected to the server . . . .')
				break
			else:
				left_data, left_state, right_data, right_state = vr.get_controller_data()
				if not left_data: # If one is False, all of them are False anyway
					print '------- Controllers are off ----------'; break;
				# There should be 18 floats. 9 [] for each controller. 
				final_data = ' '.join(map(str,left_data)) + ' ' + ' '.join(map(str,left_state)) + ' ' + ' '.join(map(str,right_data)) + ' ' + ' '.join(map(str,right_state))
				flag = client.send_data(final_data)
				# print final_data
				if not flag:
					print('Success message not received from the server')
				time.sleep(0.05) # 0.05 is working initially
				
				# print final_data
		client.close()
	except Exception as exp:
		print exp
		print 'There is either problem with client connection or vive controllers'
	time.sleep(0.5)