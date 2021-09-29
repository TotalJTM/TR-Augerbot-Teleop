#JTM 2021
#the main function to run the P3-DX robot in a teleoperated mode

from network import network_sock
from serial_communications import serial_port
import time, json
from math import pi, sqrt

from _thread import *
import threading

#host socket IP address and port
host_ip = '192.168.0.103'
server_port = 12345
#determines whether socket tries to re-establish connection if lost
autoreconnect_socket = True

#function to constrain a number
def constrain(n, minn, maxn):
    return max(min(maxn, n), minn)

#class that acts as a timer
#timer object should be created, then the timer can be started with start func
#check_timer will return False until timer has expired, where it will return True
class Timer:
	#takes an interval in seconds
	def __init__(self, interval):
		self.interval = interval
		self.start_time = 0

	def start(self):
		self.start_time = time.perf_counter()

	def check_timer(self):
		if (self.start_time+self.interval) <= time.perf_counter():
			return True
		else:
			return False

#socket robot command class
#used to format basic commands into dictionary objects to be sent over network socket
class commands:
    #create an "OK" message dictionary object
    #should be used as an acknowledgement
    def ok():
        return [{"OK": "OK"}]

    #create a "STOP" command
    #should be used to signify the end of a socket connection
    def stop():
        return [{"STOP":"STOP"}]

    #function to format an array of commands into a json field "arr"
    #and convert it to a byte string object
    #byte string object is returned
    def format_arr(pay_arr):
        msg = json.dumps({"arr":pay_arr})
        return bytes(msg, 'utf-8')

#class to store TR-Augerbot robot instance
#stores many of the variables and functions that facilitate communication between
#RPI and arduino.
class TR_Augerbot:
	#takes no arguments other than serial_port object
	def __init__(self, robot_controller = None):
		#store arduino serial connection as robot_controller
		self.robot_controller = robot_controller
		#create timer objects that generate messages to send to arduino
		self.robot_values_timer = Timer(0.1)
		self.feedback_update_timer = Timer(0.05)
		#variables to store robot message variables that will be sent to arduino
		self.left_speed = 0
		self.right_speed = 0 		
		self.auger_lift = 0 		#0: STOP, 1: UP, 2: DOWN
		self.auger_slide = 0 		#0: STOP, 1: Down track, 2: Up track (backward)
		self.auger_drive = 0  		#0: STOP, 1: UP, 2: DOWN
		self.belt_lift = 0 			#0: STOP, 1: UP, 2: DOWN
		self.belt_drive = 0 		#0: STOP, 1: UP, 2: DOWN
		#variables holding robot specific information, used for local calculations
		self.wheel_distance = 13.0
		self.wheel_diam = 7.65
		self.enc_ticks_per_rev = 19105.0
		self.wheel_dist_circum = pi*self.wheel_distance
		self.ticks_per_inch = self.enc_ticks_per_rev/(self.wheel_diam*pi)

	#function to update robot values with an array of json objects
	#will iter through arr looking for dict keys with the same name
	#sets a value or calls a function
	def update_values_with_json(self, arr):
		for item in arr:
				#handle motor keys
				if "left_speed" in item:
					self.left_speed = item["left_speed"]
				if "right_speed" in item:
					self.right_speed = item["right_speed"]
				if "auger_lift" in item:
					self.auger_lift = item["auger_lift"]
				if "auger_slide" in item:
					self.auger_slide = item["auger_slide"]
				if "auger_drive" in item:
					self.auger_drive = item["auger_drive"]
				if "belt_lift" in item:
					self.belt_lift = item["belt_lift"]
				if "belt_drive" in item:
					self.belt_drive = item["belt_drive"]

	#function starts all the update timers, should be done after creating this object
	def start_robot_update_timers(self):
		self.robot_values_timer.start()
		#self.feedback_update_timer.start()

	#arduino feedback message class,
	#struct FeedbackMessage {
  	#	long left_front_speed, left_back_speed, right_front_speed, right_back_speed, stepper_pos;
	#};


	#function to send a message to the arduino after the timer has expired
	#sends a message and restarts timer after expiration
	#takes a forced argument to force a message to be sent regardless of timer state
	def update_robot_values(self, forced=False):
		if self.robot_values_timer.check_timer() or forced:
			self.send_message()
			self.robot_values_timer.start()

	#function to get a message from the arduino after the timer has expired
	#sends a message and restarts timer after expiration
	#takes a forced argument to force a message to be sent regardless of timer state
	def update_feedback_values(self, forced=False):
		if self.feedback_update_timer.check_timer() or forced:
			self.send_message([0])
			response = self.get_arduino_message()
			if response is not None:
				x = 1
			self.feedback_update_timer.start()

	#function to send a message to the arduino
	#arguments are: a command integer and an optional array of values
	def send_message(self, vals=None):
		#if a connection to the arduino exists
		if self.robot_controller is not None:
			msg = '<'
			if vals is None:
				#create a message with proper formatting
				msg = f'<1,{int(self.left_speed)},{int(self.right_speed)},{self.auger_lift},{self.auger_slide},{self.auger_drive},{self.belt_lift},{self.belt_drive}>'
			else:
				for item in vals:
					msg += f'{item},'
				msg = msg[0:-1]
				msg += '>'

			#print, send message and return true if message sent successfully, return false if message not sent
			print(msg)
			if not self.robot_controller.send(msg):
				return False
			#print(self.robot_controller.receive())
			return True
		#if not controller exists, print "not controller" and return false
		else:
			print(f'message could not be sent, no controller')
			return False

	#function to get incoming arduino messages
	#returns the decoded message received from the robot controller or none when no message is received
	#return is formatted (cmd, data)
	def get_arduino_message(self):
		message = self.robot_controller.receive()
		if message:
			return self.decode_received_arduino_message(message)
		else:
			return None

	#function to decode an arduino message
	#removes formatting and isolates cmd and data
	def decode_received_arduino_message(self, response):
		resp = response.strip('\r\n').strip('<').strip('>')
		resp = resp.split(',')
		return resp


#function to handle socket message commands
#returns items if all works properly, returns none if something fails
#this function can only handle json strings formatted like '{"arr":[{more json entries}]},'
#if multiple json objects are in the string, this function can handle it so long as there is not more than one list
def handle_message_commands(message):
	#try statement in case json string really isnt json
	try:
		#decode message and build some starting variables/lists
		message = message.decode()
		combined_message = []
		split_message = []
		last_ind = 0
		#iteratee through message chars looking for end of json list ']},'
		for ind, c in enumerate(message):
			if c == ',':
				if message[ind-1] == '}' and message[ind-2] == ']':
					#split the message into seperate json strings
					split_message.append(message[last_ind:ind])
					last_ind = ind+1
		#iter through split messages
		for arr_entry in split_message:
			#try to load message fragment as a json object, append it to list if possible
			try:
				arr_entry = json.loads(arr_entry)
				for item in arr_entry["arr"]:
					combined_message.append(item)
			except:
				print(f'couldnt handle {arr_entry}')
		#return the combined message
		return combined_message
	except:
		print("failed combination")
		return None

#
#arduino_queue = []
#
#def arduino_thread(robot):
#	robot.update_motor_speed()

#
#socket_queue = []
#

#function that updates robot values (mainly left and right speeds) from incoming socket data 
#this is run in a thread
def socket_thread(socket, robot):
	socket_connected = True
	#run while socket is connected and flag is true
	while socket_connected:
		try:
			#receive data and check if it is valid (not '' and not none)
			message = socket.receive()
			if message.decode() == '':
				terminate()#will crash program, intentional for testing
			if message is not None:
				print(f'received {message}')
				#get formatted message commands
				message_items = handle_message_commands(message)
				print(f'message items {message_items}')
				#update robot values if message items are valid
				if message_items is not None:
					robot.update_values_with_json(message_items)
		except KeyboardInterrupt:
			socket_connected = False
			#xczmkcmkzm()
		except:
			socket_connected = False
			print('Not connected to basestation')
	#close socket when thread flag is stopped
	socket.close()



if __name__ == '__main__':

	#start arduino serial connection on port 0 with linux device prefix
	arduino = serial_port(9600, port=0, prefix='/dev/ttyACM')
	#start arduino serial connection on port 24 with windows device prefix
	#arduino = serial_port(9600,port=24,prefix='COM')
	#arduino = serial_port(9600,port=23,prefix='COM')
	#print controller assigned and receive "CONTROL STARTED" message from arduino
	print("controller assigned")
	print(arduino.receive())

	#create robot object with arduino connection
	robot = TR_Augerbot(robot_controller=arduino)

	#create network socket object and start server as host
	s = network_sock()
	s.bind(host=host_ip, port=server_port)

	#set run_flag as true
	run_flag = True

	#create socket thread with socket and robot object arguments
	sock_thread = threading.Thread(target=socket_thread, args=(s, robot))
	#print the socket object, start the thread and start the robot timers
	print(sock_thread)
	sock_thread.start()

	robot.start_robot_update_timers()
	robot.update_robot_values(forced=True)

	#while the run_flag is true
	while run_flag:

		#if socket thread stops
		if not sock_thread.is_alive():
			#update speeds to 0 and force the robot to stop
			robot.left_speed = 0
			robot.right_speed = 0
			robot.auger_lift = 0
			robot.auger_slide = 0
			robot.auger_drive = 0
			robot.belt_lift = 0
			robot.belt_drive = 0
			robot.update_robot_values(forced=True)
			#try to reconnect if autoreconnect_socket is set
			if autoreconnect_socket:
				print("looking for new socket connection")
				s = network_sock()
				s.bind(host=host_ip, port=server_port)
				print("new socket connected")
				sock_thread = threading.Thread(target=socket_thread, args=(s, robot))
				sock_thread.start()
			#otherwise, stop the robot loop
			else:
				run_flag = False

		#send robot messages and receive data from robot
		robot.update_robot_values()
		#robot.update_feedback_values()