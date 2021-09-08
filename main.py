#JTM 2021
#the main function to run the P3-DX robot in a teleoperated mode

from network import network_sock
from serial_communications import serial_port
import time, json
from math import pi, sqrt

from _thread import *
import threading

#host socket IP address and port
server_ip = '192.168.1.5'
server_port = 12345
#determines whether socket tries to re-establish connection if lost
autoreconnect_socket = False

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
    #create dictionary objects with specified left and right motor values
    #return formatted dictionary objects in an array
    def motor(left_motor_val=None, right_motor_val=None):
        arr = []
        if left_motor_val is not None:
            arr.append({"left_motor_speed": left_motor_val})
        if right_motor_val is not None:
            arr.append({"right_motor_speed": right_motor_val})
        return arr

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

#class to store P3-DX robot instance
#stores many of the variables and functions that facilitate communication between
#RPI and arduino.
class P3_DX_robot:
	#takes no arguments other than serial_port object
	def __init__(self, robot_controller = None):
		#store arduino serial connection as robot_controller
		self.robot_controller = robot_controller
		#create timer objects that generate messages to send to arduino
		self.motor_update_timer = Timer(0.1)
		self.encoder_update_timer = Timer(0.05)
		self.button_update_timer = Timer(0.1)
		#variables to store left, right speeds and encoder values
		self.left_motor_speed = 0
		self.right_motor_speed = 0
		self.last_left_enc = 0
		self.last_right_enc = 0
		#variables to store robot button states
		self.last_reset_button_state = 1
		self.last_motor_button_state = 1
		self.last_aux1_switch_state = 1
		self.last_aux2_switch_state = 1
		#variables holding robot specific information, used for local calculations
		self.wheel_distance = 13.0
		self.wheel_diam = 7.65
		self.enc_ticks_per_rev = 19105.0
		self.wheel_dist_circum = pi*self.wheel_distance
		self.ticks_per_inch = self.enc_ticks_per_rev/(self.wheel_diam*pi)
		#robot PID constants
		self.kp = 0.85
		self.ki = 0.85
		self.kd = 1
		#music box object/variable for fun
		self.robot_music_box = None
		self.last_buzzer_freq = 0

	#function to update robot values with an array of json objects
	#will iter through arr looking for dict keys with the same name
	#sets a value or calls a function
	def update_values_with_json(self, arr):
		for item in arr:
				#handle motor keys
				if "left_motor_speed" in item:
					self.left_motor_speed = item["left_motor_speed"]
				if "right_motor_speed" in item:
					self.right_motor_speed = item["right_motor_speed"]

	#function starts all the update timers, should be done after creating this object
	def start_robot_update_timers(self):
		self.motor_update_timer.start()
		self.encoder_update_timer.start()
		self.button_update_timer.start()

	#arduino message commands
	#<10, left_motor_val, right_motor_val>										|send motor values
	#<11>, <11,left_encoder_val, right_encoder_val>								|sends request for encoder values, encoder values are received
	#<12,left_encoder_reset, right_encoder_reset>								|resets encoder count if 1
	#<20>, <20,reset_sw_state, motor_sw_state, aux1_sw_state, aux2_sw_state>	|sends request for button states, button states are 0 or 1
	#<21, buzzer_freq>															|sets buzzer frequency
	#<22, LED_HIGH_time_ms, LED_LOW_time_ms>									|sets the LED high and low period
	#<91, kp, ki, kd>															|updates PID values
	#<92,direct_drive_if_not_zero>												|updates direct drive state

	#function to update the motor speed after the timer has expired
	#sends a message and restarts timer after expiration
	#takes a forced argument to force a message to be sent regardless of timer state
	def update_motor_speed(self, forced=False):
		if self.motor_update_timer.check_timer() or forced:
			self.send_message(10, [int(self.left_motor_speed),int(self.right_motor_speed)])
			self.motor_update_timer.start()

	#function to update the encoder values
	#sends a message asking for encoder data, receives data and checks if none
	#update the local encoder counts and restart timer if valid data
	def get_encoder_values(self):
		if self.encoder_update_timer.check_timer():
			self.send_message(11)
			returned = self.get_arduino_message()
			if returned is not None:
				cmd = returned[0]
				data = returned[1]
				print(f'cmd: {cmd}. data: {data}')
				self.last_left_enc = int(data[0])
				self.last_right_enc = int(data[1])
				self.encoder_update_timer.start()

	#function to update the button values
	#sends a message asking for button states, receives data and checks if none
	#update the local button states and restart timer if valid data
	def get_button_values(self):
		if self.button_update_timer.check_timer():
			self.send_message(20)
			returned = self.get_arduino_message()
			if returned is not None:
				cmd = returned[0]
				data = returned[1]
				print(f'cmd: {cmd}. data: {data}')
				self.last_reset_button_state = int(data[0])
				self.last_motor_button_state = int(data[1])
				self.last_aux1_switch_state = int(data[2])
				self.last_aux2_switch_state = int(data[3])
				self.button_update_timer.start()

	#function to handle button states
	#will run in main loop, constantly being checked
	def handle_buttons(self):
		#if self.last_reset_button_state == 0:
			
		#if the motorbutton state is pressed, reset motor speeds and stop script
		#this is a runaway estop method in case something goes wrong
		if self.last_motor_button_state == 0:
			self.left_motor_speed = 0
			self.right_motor_speed = 0
			print("Motors reset by robot")
			exit()
		#if self.last_aux1_button_state == 0:
			
		#if self.last_aux2_button_state == 0:

	#function to start the music box object
	def start_music_box(self, song_number=0, tempo=160):
		self.robot_music_box = music_box(song=songs[song_number], tempo=tempo)

	#function handles the music box workings while robot_music_box object exists
	#note frequency is constantly checked against last note freq, when a change occurs the new freq is sent to arduino
	#and last_freq is updated
	def handle_music_box(self,):
		if self.robot_music_box is not None:
			f = self.robot_music_box.get_note()
			if f != self.last_buzzer_freq and f != None:
				self.send_message(21, [f])
				self.last_buzzer_freq = f
		#if the music box is on its last note, set the object to be none
		if self.robot_music_box.notes == self.robot_music_box.current_note and self.robot_music_box.note_on == False:
			self.robot_music_box = None
					
	#function to calculate the distance the left and right wheel have moved since last encoder reset
	#returns the distance traveled in inches
	def distance_moved(self):
		return (self.last_left_enc/self.ticks_per_inch), (self.last_right_enc/self.ticks_per_inch)

	#function to reset the encoder values on the arduino
	#takes a left_enc or right_enc boolean allowing for one encoder to be reset without changing the other count
	def reset_encoder_values(self, left_enc=False, right_enc=False):
		arr = [0,0] #create temp array with states that dont change encoder count
		#update left and right enc positions in arr
		if left_enc:
			arr[0] = 1
		if right_enc:
			arr[1] = 1
		#send message
		self.send_message(13, arr)

	#function to send a message to the arduino
	#arguments are: a command integer and an optional array of values
	def send_message(self, cmd, vals=[]):
		#if a connection to the arduino exists
		if self.robot_controller is not None:
			#start a message with correct formatting
			msg = f'<{cmd},'
			#iter through the array of values, add them to the message string
			for index, vals in enumerate(vals):
				msg += f'{vals},'
			#cap off the message
			msg = msg[:-1] + '>'
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
		return (resp[0], resp[1:])


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
	#close socket when thread flag is stopped
	socket.close()



if __name__ == '__main__':

	#start arduino serial connection on port 0 with linux device prefix
	arduino = serial_port(115200, port=0, prefix='/dev/ttyACM')
	#start arduino serial connection on port 24 with windows device prefix
	#arduino = serial_port(115200,port=24,prefix='COM')
	#arduino = serial_port(115200,port=23,prefix='COM')
	#print controller assigned and receive "CONTROL STARTED" message from arduino
	print("controller assigned")
	print(arduino.receive())

	#create robot object with arduino connection
	robot = P3_DX_robot(robot_controller=arduino)

	#upodate PID variables and set robot to direct drive (without PID controllers)
	#robot.send_message(91, [robot.kp,robot.ki,robot.kd])
	robot.send_message(92, [1])

	#create network socket object and then connect to the host server
	s = network_sock()
	s.connect(server_ip, server_port)

	#set run_flag as true
	run_flag = True

	#create socket thread with socket and robot object arguments
	sock_thread = threading.Thread(target=socket_thread, args=(s, robot))
	#print the socket object, start the thread and start the robot timers
	print(sock_thread)
	sock_thread.start()
	robot.start_robot_update_timers()

	#start a music box object
	#robot.start_music_box(0, tempo=160)
	#robot.start_music_box(2, tempo=114)
	#robot.start_music_box(3, tempo=144)

	#while the run_flag is true
	while run_flag:

		#if socket thread stops
		if not sock_thread.is_alive():
			#update speeds to 0 and force the robot to stop
			robot.left_motor_speed = 0
			robot.right_motor_speed = 0
			robot.update_motor_speed(forced=True)
			#try to reconnect if autoreconnect_socket is set
			if autoreconnect_socket:
				print("looking for new socket connection")
				s = network_sock()
				s.connect(server_ip, server_port)
				print("new socket connected")
				sock_thread = threading.Thread(target=socket_thread, args=(s, robot))
				sock_thread.start()
			#otherwise, stop the robot loop
			else:
				run_flag = False

		#send robot messages and receive data from robot
		robot.update_motor_speed()