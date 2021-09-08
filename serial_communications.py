#JTM 2021
#custom serial port class

#import serial as Serial
from serial import Serial

class serial_port:
	#serial port object contains the serial object allowing multiple serial instances to work simultaneousl
	#input arguments are: baudrate, serial port number, serial port prefix (specific to the system running this script), and max search range
	#default values are optimized for RPI4 so values should be set when using this script on other systems 
	def __init__(self, baud=115200, port=None, prefix='/dev/ttyACM', serial_search_range=4):
		#define serial port placeholder as nonetype
		self.sp = None
		#if a port is specified, try to connect
		#print "invalid serial port" if connection failed
		#failure could also occur from bad baudrate and serial prefix
		if port is not None:
			try:
				self.sp = Serial(f'{prefix}{port}', baud)
			except:
				print(f'Invalid serial port {prefix}{port}')
		#otherwise, attempt to open serial ports, with the specified prefix, through a range of number (0 to search_range)
		else:
			ports = []
			#look for the serial ports, add successful port address to ports array 
			for port in range(0,serial_search_range):
				try:
					temp = Serial(f'{prefix}{port}', baud)
					temp.close()
					ports.append(f'{prefix}{port}')
				except:
					continue
			#if the ports array is greater than 1, get user input on which port should be used and assign serial port
			#(note, i dont like this implementation. the correct port address (or arr of addresses) should be passed back in a seperate function)
			if len(ports) >= 2:
				print(f'The following serial ports are available:\n\t{ports}\nEnter the robot port number:')
				resp = input()
				self.sp = Serial(f'{prefix}{resp}', baud)
			#if the port array contains 1 entry, open the serial port and assign it to the serial port var
			elif len(ports) == 1:
				self.sp = Serial(f'{prefix}{port}', baud)
			#if no serial ports available, print "no ports detected"
			else:
				print("No available serial port detected")

	#function to send a message to the serial device
	#takes a string message, returns true or false depending on whether a serial port object exists
	def send(self, message):
		if self.sp is not None:
			self.sp.write(message.encode())
			return True
		else:
			return False

	#function to receive a message from the serial device
	#returns the received message (as a utf-8 string), None (if no response) or false (if a serial port object does not exist)
	def receive(self):
		if self.sp is not None:
			response = self.sp.readline()
			print(response)
			if response:
				return response.decode('utf-8')
			else:
				return None
		else:
			return False